/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "AD7793.h"  // AD7793 definitions.

#include <Arduino.h>
#include <SPI.h>

#include "Communication.h"  // Communication definitions.
#include "freertos/semphr.h"
extern SemaphoreHandle_t xSPIMutex;

/********************************* Global data ********************************/

const uint8_t reg_size[8] = {1, 2, 2, 3, 1, 1, 3, 3};

/**************************** Global functions *******************************/

/**
   @brief Initialization

   @return none
**/
void AD7793_Init(void) {
   uint32_t ui32reg_value;

   AD7793_Reset();  // Reset ADC converter.

   ui32reg_value = (uint32_t)(AD7793_GAIN << 8);  // Set ADC gain
   ui32reg_value |= (uint32_t)AD7793_REFSEL;      // Select internal reference source
   ui32reg_value |= (uint32_t)AD7793_BUF;         // Configure buffered mode of operation

   AD7793_WriteRegister(AD7793_REG_CONF, ui32reg_value);  // Set configuration options

   // ui32reg_value = AD7793_ReadRegister(AD7793_REG_CONF);

   // AD7793_WriteRegister(AD7793_REG_CONF, ui32reg_value);

   AD7793_WriteRegister(AD7793_REG_IO, 0x02);  // Set IOUT2 to 210 uA
}

/*******************************************************************************
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @return  None.
 *******************************************************************************/
void AD7793_Reset(void) {
   SPI_Write(0, 0, 4);  // Write 4 bytes = 0xFF
}

/**
   @brief Read register value

   @param ui8address - register address

   @return uint32_t - register value
**/
uint32_t AD7793_ReadRegister(uint8_t ui8address) {
   static uint32_t ui32value;

   uint8_t ui8reg_adrr = (AD7793_COMM_READ | AD7793_COMM_ADR(ui8address));  // Set value (read command + register address) to write in COMM register

   ui32value = SPI_Read(ui8reg_adrr, reg_size[ui8address]);  // Read register value

   return ui32value;
}

/**
   @brief Write data to register

   @param ui8address - register address
   @param ui32data - data to write

   @return none
**/
void AD7793_WriteRegister(uint8_t ui8address, uint32_t ui32data) {
   uint8_t ui8reg_adrr = (AD7793_COMM_WRITE | AD7793_COMM_ADR(ui8address)); /* Set value (write command + register address) to write in COMM register */

   SPI_Write(ui8reg_adrr, ui32data, reg_size[ui8address]); /* Write register value */
}

/**
   @brief Read ADC conversion results

   @param mode - conversion mode: SINGLE_CONV or CONTINUOUS_CONV
   @param ui8channel - ADC channel to scan

   @return uint32_t - conversion result
**/
uint32_t AD7793_Scan(enMode mode, uint8_t ui8channel) {
   static uint32_t ui32result, ui32reg_value;

   if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {      
      AD7793_Reset();
      AD7793_WriteRegister(AD7793_REG_CONF, (AD7793_GAIN << 8) | AD7793_REFSEL | AD7793_BUF);
      AD7793_WriteRegister(AD7793_REG_IO, 0x02);
      
      // AD7793_Calibrate(ui8channel, CAL_INT_FULL_MODE);

      AD7793_SelectChannel(ui8channel); /* Select channel to scan */

      if (mode == SINGLE_CONV) { /* Check if single conversion mode is wanted */

         ui32reg_value = AD7793_ReadRegister(AD7793_REG_MODE);

         ui32reg_value &= AD7793_MODE_MSK;

         ui32reg_value |= (uint32_t)(mode << 13); /* Set single mode operation */

         AD7793_WriteRegister(AD7793_REG_MODE, ui32reg_value);
      }

      if (mode == CONTINUOUS_CONV) {
         AD7793_CS_LOW;
      }

      while ((AD7793_ReadRegister(AD7793_REG_STAT) & RDY_BIT) == RDY_BIT);

      ui32result = AD7793_ReadRegister(AD7793_REG_DATA);

      xSemaphoreGive(xSPIMutex);
   }

   return ui32result;
}

/**
   @brief Select ADC input channel

   @param ui8channel - input channel

   @return none
**/
void AD7793_SelectChannel(uint8_t ui8channel) {
   uint32_t ui32reg_value;

   ui32reg_value = AD7793_ReadRegister(AD7793_REG_CONF); /* Read CONF register */
   ui32reg_value &= AD7793_CONF_MSK;
   ui32reg_value |= (uint32_t)ui8channel;                /* Set set channel */
   AD7793_WriteRegister(AD7793_REG_CONF, ui32reg_value); /* Write CONF register */
}

/**
   @brief Calibrate ADC input channel

   @param ui8channel - input channel
   @param mode - calibration mode: CAL_INT_ZERO_MODE, CAL_INT_FULL_MODE, CAL_SYS_ZERO_MODE, CAL_SYS_FULL_MODE

   @return none
**/
void AD7793_Calibrate(uint8_t ui8channel, enMode mode) {
   uint32_t ui32reg_value, back_up;

   AD7793_SelectChannel(ui8channel); /* Select channel */

   ui32reg_value = back_up = AD7793_ReadRegister(AD7793_REG_MODE); /* Read MODE register */

   ui32reg_value &= AD7793_MODE_MSK;

   ui32reg_value |= (uint32_t)(mode << 13); /* Set mode */

   AD7793_WriteRegister(AD7793_REG_MODE, ui32reg_value); /* Write MODE register */

   while ((AD7793_ReadRegister(AD7793_REG_STAT) & RDY_BIT) == RDY_BIT); /* Wait until RDY bit from STATUS register is high */
}

/**
   @brief Convert ADC output into voltage

   @param u32adcValue - ADC code

   @return int32_t - converted voltage
**/
float AD7793_ConvertToVolts(uint32_t u32adcValue) {
   float f32voltage;

   // 0x800000 = 16.777.216 / 2 = 8.388.608 em decimal | 2^24 = 16.777.216
   // Vref = 1170 [mV]

   int32_t i32adcValue = (int32_t)(u32adcValue - 0x800000);  // Subtração com sinal

   f32voltage = ((float)i32adcValue * 1170) / (float)0x800000;

   return f32voltage;
}