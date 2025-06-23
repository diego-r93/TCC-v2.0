#include "DRV8243.h"

#include "Communication.h"

/**
   @brief Builds a command byte according to the DRV8243 SPI protocol.

   @param addr Register address (lower 6 bits).
   @param rd   Set to true for read, false for write.
   @return Ready‑to‑send command byte.
 */
static inline uint8_t cmdByte(uint8_t addr, bool rd) {
   return (rd ? 0x40 : 0x00) | (addr & 0x3F);
}

/**
   @brief Initializes SPI, reads DEVICE_ID + STATUS, clears faults if needed.
**/
bool DRV8243_Init(void) {
   SPI_Init();

   uint8_t id1, id0, st1, st0;
   bool ok = DRV8243_DaisyRead(DRV8243_REG_DEVICE_ID,
                               DRV8243_REG_DEVICE_ID,
                               id1, id0, st1, st0);

   printf("[DRV1] DEVICE_ID = 0x%02X | STATUS = 0x%02X\n", id1, st1);
   printf("[DRV0] DEVICE_ID = 0x%02X | STATUS = 0x%02X\n", id0, st0);

   /* If any fault bits set, send global CLR_FLT ------------------------- */
   if ((st1 & 0x3F) || (st0 & 0x3F)) {
      printf("Clearing fault flags…\n");
      const uint8_t clr[6] = {
          0x82,  // HDR1 N = 2
          0xA0,  // HDR2 CLR_FLT = 1
          cmdByte(DRV8243_REG_DEVICE_ID, true),
          cmdByte(DRV8243_REG_DEVICE_ID, true),
          0x00,  // Dummy data for read
          0x00   // Dummy data for read
      };
      SPI_TransferFrame(clr, nullptr, 6);

      /* Re‑read after clear */
      DRV8243_DaisyRead(DRV8243_REG_DEVICE_ID,
                        DRV8243_REG_DEVICE_ID,
                        id1, id0, st1, st0);
      printf("[DRV1] STATUS = 0x%02X | [DRV0] STATUS = 0x%02X\n", st1, st0);
   }

   return ok && (id1 == 0x36) && (id0 == 0x36);
}

/**
 * @brief Writes two registers in a 2‑device daisy chain frame.
 *
 * Frame layout (tx):
 *   0: 0x82  // HDR1 = 0b1000_0010 → N = 2 devices
 *   1: 0x80  // HDR2 = 0b1000_0000 → CLR_FLT = 0
 *   2: cmdByte(addrDev1,false)
 *   3: dataDev1
 *   4: cmdByte(addrDev0,false)
 *   5: dataDev0
 *
 * @brief Writes to two registers in daisy-chain mode.
 * @param addrDev1 Register address for far device.
 * @param dataDev1 Data byte for far device.
 * @param addrDev0 Register address for near device.
 * @param dataDev0 Data byte for near device.
 * @return true if both devices acknowledged with valid status.
 */
bool DRV8243_DaisyWrite(uint8_t addrDev1, uint8_t dataDev1,
                        uint8_t addrDev0, uint8_t dataDev0) {
   const uint8_t tx[6] = {
       0x82,  // HDR1 0b1000_0010
       0x80,  // HDR2 0b1000_0000 (CLR_FLT = 0)
       cmdByte(addrDev1, false),
       cmdByte(addrDev0, false),
       dataDev1,  // Data for far device
       dataDev0   // Data for near device
   };
   uint8_t rx[6];

   SPI_TransferFrame(tx, rx, 6);

   bool ok1 = (rx[0] & 0xC0) == 0xC0;  // S2 (far device)
   bool ok0 = (rx[1] & 0xC0) == 0xC0;  // S1 (near device)
   return ok1 && ok0;
}

/**
 * @brief Reads two registers in a 2‑device daisy chain frame.
 *
 * Frame A: sends read commands to both devices (clocked-in later).
 * Frame B: clocks out the responses from devices using NOP commands.
 *
 * @param addrDev1 Register address for far device.
 * @param addrDev0 Register address for near device.
 * @param repDev1 Reference to store response from far device.
 * @param repDev0 Reference to store response from near device.
 * @param statDev1 Reference to store status from far device.
 * @param statDev0 Reference to store status from near device.
 * @return true if both devices responded with valid status (bits 7‑6 == 11).
 */
bool DRV8243_DaisyRead(uint8_t addrDev1, uint8_t addrDev0,
                       uint8_t &repDev1, uint8_t &repDev0,
                       uint8_t &statDev1, uint8_t &statDev0) {
   const uint8_t tx[6] = {
       0x82, 0x80,               // HDR1, HDR2
       cmdByte(addrDev1, true),  // Read command for far device
       cmdByte(addrDev0, true),  // Read command for near device
       0x00,                     // Dummy data for far device
       0x00                      // Dummy data for near device
   };
   uint8_t rx[6];
   SPI_TransferFrame(tx, rx, 6);

   // ordem: S2, S1, HDR1, HDR2, R2, R1
   statDev1 = rx[0];
   statDev0 = rx[1];
   repDev1 = rx[4];
   repDev0 = rx[5];

   return ((statDev1 & 0xC0) == 0xC0) && ((statDev0 & 0xC0) == 0xC0);
}

/**
 * @brief Enables SPI control of half‑bridge outputs and unlocks SPI_IN.
 *
 * CONFIG4 needs the following for each motor channel:
 *   ‑ PH/IN2  control = OR (external pin OR SPI)  → bit[7]=0, bit[6]=1
 *   ‑ EN/IN1  control = OR                          bit[5]=0, bit[4]=1
 *   ‑ DRVOFF  control = OR                          bit[3]=0, bit[2]=1
 *   Remaining bits default (0).
 *   Binary: 0b0101_0100  = 0x54
 *
 * COMMAND – set SPI_IN_EN (bit 2) so SPI writes to SPI_IN take effect.
 *   Binary: 0b0000_0100  = 0x04  (CLR_FLT=0, REG_LOCK=00, SPI_IN_EN=1)
 */
bool DRV8243_SetupHalfBridge(void) {
   /* Write COMMAND = 0x04 to enable SPI_IN */
   if (!DRV8243_DaisyWrite(DRV8243_REG_COMMAND, 0x90,
                           DRV8243_REG_COMMAND, 0x90)) {
      printf("[ERROR] COMMAND write failed\n");
      return false;
   }

   /* Write CONFIG4 = 0x54 to both devices */
   if (!DRV8243_DaisyWrite(DRV8243_REG_CONFIG4, 0x54,
                           DRV8243_REG_CONFIG4, 0x54)) {
      printf("[ERROR] CONFIG4 write failed\n");
      return false;
   }

   printf("[SETUP] Half-bridge control configured via SPI\n");
   return true;
}

/**
 * @brief Sets the state of each motor individually in SPI_IN combined mode.
 *
 * Maps each DRV pair (OUT1, OUT2) to the appropriate SPI_IN value based on Table 8-8:
 *   00: OUT1=L, OUT2=L
 *   01: OUT1=L, OUT2=H
 *   10: OUT1=H, OUT2=L
 *   11: OUT1=H, OUT2=H
 *
 * @param m1 OUT1 (DRV0)
 * @param m2 OUT2 (DRV0)
 * @param m3 OUT1 (DRV1)
 * @param m4 OUT2 (DRV1)
 * @return true if SPI frame sent successfully
 */
bool DRV8243_SetMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4) {
   uint8_t drv0_bits = 0;
   uint8_t drv1_bits = 0;

   // DRV0: P1 = m1, P2 = m2
   switch ((m1 << 1) | m2) {
      case 0b00:
         drv1_bits = 0x08;
         break;  // 0x08 = 1000b -> P1 Z | P2 Z
      case 0b01:
         drv1_bits = 0x03;
         break;  // 0x03 = 0011b -> P1 OFF | P2 ON
      case 0b10:
         drv1_bits = 0x02;
         break;  // 0x02 = 0010b -> P1 ON  | P2 OFF
      case 0b11:
         drv1_bits = 0x04;
         break;  // 0x04 = 0100b -> P1 ON  | P2 ON
   }

   // DRV1: P1 = m3, P2 = m4
   switch ((m3 << 1) | m4) {
      case 0b00:
         drv0_bits = 0x08;
         break;  // 0x08 = 1000b -> P1 Z | P2 Z
      case 0b01:
         drv0_bits = 0x03;
         break;  // 0x03 = 0011b -> P1 OFF | P2 ON
      case 0b10:
         drv0_bits = 0x02;
         break;  // 0x02 = 0010b -> P1 ON  | P2 OFF
      case 0b11:
         drv0_bits = 0x04;
         break;  // 0x04 = 0100b -> P1 ON  | P2 ON
   }

   return DRV8243_DaisyWrite(DRV8243_REG_SPI_IN, drv1_bits,
                             DRV8243_REG_SPI_IN, drv0_bits);
}

/**
 * @brief Reads COMMAND, CONFIG4 and SPI_IN from both devices and prints hex values.
 */
void DRV8243_DumpRegisters(void) {
   uint8_t status1, status0;
   uint8_t cmd_val1, cmd_val0;
   uint8_t cfg_val1, cfg_val0;
   uint8_t spi_val1, spi_val0;

   // Leia COMMAND dos dois dispositivos
   DRV8243_DaisyRead(
       DRV8243_REG_COMMAND, DRV8243_REG_COMMAND,
       cmd_val1, cmd_val0, status1, status0);

   // Leia CONFIG4 dos dois dispositivos
   DRV8243_DaisyRead(
       DRV8243_REG_CONFIG4, DRV8243_REG_CONFIG4,
       cfg_val1, cfg_val0, status1, status0);

   // Leia SPI_IN dos dois dispositivos
   DRV8243_DaisyRead(
       DRV8243_REG_SPI_IN, DRV8243_REG_SPI_IN,
       spi_val1, spi_val0, status1, status0);

   // Exiba os conteúdos reais
   printf("[DRV1] CMD 0x%02X  CFG4 0x%02X  SPI_IN 0x%02X\n", cmd_val1, cfg_val1, spi_val1);
   printf("[DRV0] CMD 0x%02X  CFG4 0x%02X  SPI_IN 0x%02X\n", cmd_val0, cfg_val0, spi_val0);
}
