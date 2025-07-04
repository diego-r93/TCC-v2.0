#include "Communication.h"

#include "freertos/semphr.h"
extern SemaphoreHandle_t xSPIMutex;

void SPI_Init(void) {
   SPI.begin();

   // DRV8243 CS
   DRV8243_CS_PIN_OUT;
   DRV8243_CS_HIGH;

   // AD7793 CS
   AD7793_CS_PIN_OUT;
   AD7793_CS_HIGH;

   // AD5683 CS
   AD5683_CS_PIN_OUT;
   AD5683_CS_HIGH;

   // AD7124 CS
   AD7124_CS_PIN_OUT;
   AD7124_CS_HIGH;

   SDO_PIN_IN;

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

   AD7793_CS_LOW;
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
   AD7793_CS_HIGH;
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes) {
   uint32_t ui32AdcCodes = 0;

   AD7793_CS_LOW;
   SPI.beginTransaction(AD7793_SPI_SETTINGS);

   /*  Send read command */
   SPI.transfer(ui8address);

   /*  Send dummy byte in order to receive the register value */
   for (uint8_t i = 0; i < ui8bytes; i++) {
      ui32AdcCodes = (ui32AdcCodes << 8) | SPI.transfer(0xAA);
   }

   SPI.endTransaction();
   AD7793_CS_HIGH;

   return ui32AdcCodes;
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - register address
   @param ui8_nr_bytes - register number of bytes

   @return reading result

**/
int32_t SPI_Read_Buffer(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
                        uint8_t ui8_nr_bytes) {
   int32_t ret = 0;

   DRV8243_CS_HIGH;
   AD7793_CS_HIGH;

   /*Clear Slave based on ID */

   switch (ui8_slave_id) {
      case 0:
         digitalWrite(AD7124_CS_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
         break;
      case 1:
         digitalWrite(AD5683_CS_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
         break;
   }

   SPI.transfer(ui8_buffer, ui8_nr_bytes);

   SPI.endTransaction();

   /*Set Slave based on ID */
   switch (ui8_slave_id) {
      case 0:
         digitalWrite(AD7124_CS_PIN, HIGH);
         break;
      case 1:
         digitalWrite(AD5683_CS_PIN, HIGH);
         break;
   }

   if (ui8_nr_bytes == 0)
      ret = -1;

   return ret;
}

/**
   @brief Writes a register to the Converter via SPI.

   @param ui8_slave_id - slave id for chip select
   @param ui8_address - ACC register address
   @param ui32_data - value to be written
   @param ui8_nr_bytes - nr of bytes to be written

   @return none

**/
int32_t SPI_Write_Buffer(uint8_t ui8_slave_id, uint8_t ui8_buffer[],
                         uint8_t ui8_nr_bytes) {
   int32_t ret = 0;

   if (ui8_nr_bytes > 4) {
      ui8_nr_bytes = 4;
   }

   /*Clear Slave based on ID */
   switch (ui8_slave_id) {
      case 0:
         digitalWrite(AD7124_CS_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
         break;
      case 1:
         digitalWrite(AD5683_CS_PIN, LOW);
         SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE2));
         break;
   }

   SPI.transfer(ui8_buffer, ui8_nr_bytes);
   SPI.endTransaction();

   /*Set Slave based on ID */
   switch (ui8_slave_id) {
      case 0:
         digitalWrite(AD7124_CS_PIN, HIGH);
         break;
      case 1:
         digitalWrite(AD5683_CS_PIN, HIGH);
         break;
   }

   if (ui8_nr_bytes == 0)
      ret = -1;

   return ret;
}

/**
 @brief Full‑duplex transfer of an arbitrary SPI frame.

 @param tx   Pointer to bytes to transmit.
 @param rx   Pointer to buffer to receive (may be nullptr for write‑only).
 @param len  Number of bytes to shift.

**/
void SPI_TransferFrame(const uint8_t *tx, uint8_t *rx, uint8_t len) {
   if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {
      DRV8243_CS_LOW;
      SPI.beginTransaction(DRV8243_SPI_SETTINGS);
      for (uint8_t i = 0; i < len; ++i)
         rx ? rx[i] = SPI.transfer(tx[i]) : SPI.transfer(tx[i]);
      SPI.endTransaction();
      DRV8243_CS_HIGH;
      xSemaphoreGive(xSPIMutex);
   }
}
