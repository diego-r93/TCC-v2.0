/***************************** Include Files **********************************/

#include "AD7793Communication.h"

#include <Arduino.h>
#include <SPI.h>

#include "AD7793.h"

/***************************************************************************
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - idle state for SPI clock is low.
 *	                          0x1 - idle state for SPI clock is high.
 * @param clockPha - SPI clock phase (0 or 1).
 *                   Example: 0x0 - data is latched on the leading edge of SPI
 *                                  clock and data changes on trailing edge.
 *                            0x1 - data is latched on the trailing edge of SPI
 *                                  clock and data changes on the leading edge.
 *******************************************************************************/

void SPI_Init() {
   ADI_PAR_CS_PIN;
   ADI_PART_CS_PIN_OUT;
   ADI_PART_CS_HIGH;
   SDO_PIN;
   SDO_PIN_IN;

   SPI.begin(SCK_PIN, SDO_PIN, SDI_PIN, ADI_PAR_CS_PIN);
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

   ADI_PART_CS_LOW;
   SPI.beginTransaction(SPI_SETTINGS);

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
   ADI_PART_CS_HIGH;
}

/**
   @brief Reads a specified register address in the converter via SPI.

   @param ui8address - register address
   @param enRegs - register number

   @return reading result

**/
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes) {
   uint32_t ui32AdcCodes = 0;

   ADI_PART_CS_LOW;
   SPI.beginTransaction(SPI_SETTINGS);

   /*  Send read command */
   SPI.transfer(ui8address);

   /*  Send dummy byte in order to receive the register value */
   for (uint8_t i = 0; i < ui8bytes; i++) {
      ui32AdcCodes = (ui32AdcCodes << 8) | SPI.transfer(0xAA);
   }

   SPI.endTransaction();
   ADI_PART_CS_HIGH;

   return ui32AdcCodes;
}