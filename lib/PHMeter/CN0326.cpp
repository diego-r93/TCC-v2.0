#include "CN0326.h"

#include <Arduino.h>

#include "AD7793.h"
#include "Communication.h"

/********************************* Definitions ********************************/
#if (USE_IOUT2 == YES)
float iout2_calibration; /* [mA]  */
#endif

/************************* Variable Definitions ******************************/

/****************************** Global functions *****************************/

/**
   @brief Initialization part

   @return none

**/
void CN0326_Init(void) {
#if (USE_IOUT2 == YES)
   uint32_t ui32result;
   int32_t i32voltage;
#endif

   AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN1P_AIN1M, CAL_INT_FULL_MODE);
   AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN2P_AIN2M, CAL_INT_FULL_MODE);
   AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_ZERO_MODE);
   AD7793_Calibrate(AD7793_CH_AIN3P_AIN3M, CAL_INT_FULL_MODE);

#if (USE_IOUT2 == YES)
   ui32result = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN3P_AIN3M);
   i32voltage = AD7793_ConvertToVolts(ui32result);
   iout2_calibration = i32voltage / (float)5000;
#endif
}

/**
   @brief Calculate temperature value

   @return float - temperature value
**/
float CN0326_CalculateTemp(void) {
   static float temp, res, f32current, f32voltage;

   uint32_t ui32adcValue;

#if (USE_IOUT2 == YES) /* Check which excitation current to use */
   f32current = iout2_calibration;
#else
   f32current = I_EXC;
#endif

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN2P_AIN2M); /* Read ADC output value */

   f32voltage = AD7793_ConvertToVolts(ui32adcValue); /* Convert ADC output value to voltage */

   res = f32voltage / f32current; /* Calculate RTD resistance */
   // printf(" | ADC Value: %li | Voltage: %.4f | Current: %.4f | Resistance: %.4f\n", ui32adcValue, f32voltage, f32current, res);

   temp = ((res - RMIN) / (TEMP_COEFF * RMIN)); /* Calculate temperature value */

   return temp;
}

/**
   @brief Calculate pH value

   @return float - pH value
**/
float CN0326_CalculatePH(void) {
   float temp, ph;

   uint32_t ui32adcValue;
   int32_t i32voltage;

   temp = CN0326_CalculateTemp(); /* Calculate temperature */

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_AIN1P_AIN1M); /* Read ADC output value */

   i32voltage = AD7793_ConvertToVolts(ui32adcValue); /* Convert ADC output value to voltage */

   ph = PH_ISO - (((double)((i32voltage - TOLERANCE) * FARADAY_CONST)) / (PH_CONST * AVOGADRO_NUMBER * (temp + K_DEGREES))); /* Calculate pH value */

   return ph;
}

/**
   @brief Calculate temperature value

   @return float - temperature value
**/
float CN0326_CalculateInternalTemp(void) {
   static float temp;

   uint32_t ui32adcValue;

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_TEMP);

   temp = (((float)(ui32adcValue - 0x800000) / ((float)0x800000)) * 1170 / 0.810) - K_DEGREES;  // Sentitivity is approximately 0.81 mV/Â°K, according  to AD7793 datasheet
   return temp;
}

/**
   @brief Calculate AVDD value

   @return float - AVDD value
**/
float CN0326_CalculateAVDD(void) {
   static float avdd;

   uint32_t ui32adcValue;

   ui32adcValue = AD7793_Scan(SINGLE_CONV, AD7793_CH_AVDD_MONITOR);

   avdd = (1.17 / (1 / 6.0)) * (((float)(ui32adcValue - 0x800000)) / ((float)0x800000));

   return avdd;
}