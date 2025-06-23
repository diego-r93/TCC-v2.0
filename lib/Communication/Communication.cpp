#include "Communication.h"

void SPI_Init(void) {
   DRV8243_CS_PIN_INIT;
   DRV8243_CS_PIN_OUT;
   DRV8243_CS_HIGH;
   SDO_PIN_IN;

   SPI.begin(SCK_PIN, SDO_PIN, SDI_PIN, DRV8243_CS_PIN);
   Serial.println("[SPI INIT] SPI configurado e iniciado");
}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8_address - ACC register address
   @param ui32_data - value to be written
   @param ui8_nr_bytes - nr of bytes to be written

   @return none

**/
void SPI_Write(uint8_t ui8address, uint32_t ui32data, uint8_t ui8bytes) {
   uint8_t ui8counter, ui8write[ui8bytes];

   DRV8243_CS_LOW;
   SPI.beginTransaction(AD7793_SPI_SETTINGS);

   if (ui8bytes != 4) {
      for (ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {
         ui8write[ui8counter - 1] = (ui32data >> ((ui8bytes - ui8counter) * 8));  // Separate data into 8 bits values
      }
   }

   if (ui8bytes != 4) {  // Check if want to write ADC register, not reset the ADC
      SPI.transfer(ui8address);
   }

   for (ui8counter = 1; ui8counter <= ui8bytes; ui8counter++) {
      if (ui8bytes != 4) {  // Check if want to write ADC register, not reset the ADC
         SPI.transfer(ui8write[ui8counter - 1]);
      } else {
         SPI.transfer(0xFF);  // Write 4 bytes = 0xFF
      }
   }

   SPI.endTransaction();
   DRV8243_CS_HIGH;
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes) {
   uint32_t ui32AdcCodes = 0;

   DRV8243_CS_LOW;
   SPI.beginTransaction(AD7793_SPI_SETTINGS);

   /*  Send read command */
   SPI.transfer(ui8address);

   /*  Send dummy byte in order to receive the register value */
   for (uint8_t i = 0; i < ui8bytes; i++) {
      ui32AdcCodes = (ui32AdcCodes << 8) | SPI.transfer(0xAA);
   }

   SPI.endTransaction();
   DRV8243_CS_HIGH;

   return ui32AdcCodes;
}

/**
 @brief Full‑duplex transfer of an arbitrary SPI frame.

 @param tx   Pointer to bytes to transmit.
 @param rx   Pointer to buffer to receive (may be nullptr for write‑only).
 @param len  Number of bytes to shift.

**/
void SPI_TransferFrame(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    DRV8243_CS_LOW;
    SPI.beginTransaction(DRV8243_CS_SPI_SETTINGS);
    for (uint8_t i = 0; i < len; ++i)
        rx ? rx[i] = SPI.transfer(tx[i]) : SPI.transfer(tx[i]);
    SPI.endTransaction();
    DRV8243_CS_HIGH;
}
