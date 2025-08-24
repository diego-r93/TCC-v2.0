/*******************************************************************************
 *   @file   CN0411.c
 *   @brief  CN0411 source file
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
 ********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "CN0411.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "AD7124_regs.h"
#include "Communication.h"
#include "Timer.h"

/******************************************************************************/
/*************************** Variable Definitions *****************************/
/******************************************************************************/

uint8_t pwm_status;
uint8_t pwm_index;
uint32_t pwm_tick_count;

uint16_t pwm_2400_freq[6];
uint16_t pwm_100_freq[6];
uint16_t pwm_bit[6];
uint16_t pwm_setclr[6];

uint16_t *pwm_freq;

/******************************************************************************/
/************************* Function Definitions *******************************/
/******************************************************************************/

/**
 * CN0411 set DAC output value
 *
 * @param cn0411_dev - The device structure.
 * @param output_val - voltage value to be written on DAC
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_DAC_set_value(struct cn0411_device *cn0411_dev, float output_val) {
   cn0411_dev->ad5683_dev.dac_reg_value = (uint32_t)(output_val * DAC_FS_VAL /
                                                     VREF);

   return AD5683_write_dac_value(&cn0411_dev->ad5683_dev,
                                 cn0411_dev->ad5683_dev.dac_reg_value);
}

/**
 * CN0411 ADC Operation Mode function
 *
 * @param cn0411_dev - The device structure.
 * @param mode - operation mode of the ADC
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_operation_mode(struct cn0411_device *cn0411_dev,
                                  enum op_mode mode) {
   int32_t mask;

   mask = (AD7124_ADC_CTRL_REG_REF_EN |
           AD7124_ADC_CTRL_REG_DATA_STATUS |
           AD7124_ADC_CTRL_REG_POWER_MODE(LOW_POWER) |
           AD7124_ADC_CTRL_REG_MODE(mode) |
           AD7124_ADC_CTRL_REG_CLK_SEL(INTERNAL_CLK1));

   cn0411_dev->ad7124_dev.regs[AD7124_ADC_Control].value = mask;

   return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                               cn0411_dev->ad7124_dev.regs[AD7124_ADC_Control]);
}

/**
 * CN0411 ADC Setup 0 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_setup(struct cn0411_device *cn0411_dev) {
   int32_t mask;

   mask = (AD7124_CFG_REG_BURNOUT(0) | AD7124_CFG_REG_REF_BUFP |
           AD7124_CFG_REG_REF_BUFM | AD7124_CFG_REG_AIN_BUFP |
           AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0) |
           AD7124_CFG_REG_PGA(0));

   cn0411_dev->ad7124_dev.regs[AD7124_Config_0].value = mask;

   if (AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                            cn0411_dev->ad7124_dev.regs[AD7124_Config_0]) == CN0411_FAILURE)
      return CN0411_FAILURE;

   mask = (AD7124_CFG_REG_BURNOUT(0) |
           AD7124_CFG_REG_REF_BUFP |
           AD7124_CFG_REG_REF_BUFM |
           AD7124_CFG_REG_AIN_BUFP |
           AD7124_CFG_REG_AINN_BUFM |
           AD7124_CFG_REG_REF_SEL(2) |
           AD7124_CFG_REG_PGA(0));

   cn0411_dev->ad7124_dev.regs[AD7124_Config_1].value = mask;

   if (AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                            cn0411_dev->ad7124_dev.regs[AD7124_Config_1]) == CN0411_FAILURE)
      return CN0411_FAILURE;

   return CN0411_SUCCESS;
}

/**
 * CN0411 ADC Channels configuration function
 *
 * @param cn0411_dev - The device structure.
 * @param ch_en - enable/disable ADC channel
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_set_ch(struct cn0411_device *cn0411_dev, uint8_t channel,
                          uint8_t ch_en) {
   uint32_t mask;

   switch (channel) {
      case ADC_CH0:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(0) |
                 AD7124_CH_MAP_REG_AINP(1) | AD7124_CH_MAP_REG_AINM(6));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_0].value = mask;
         break;
      case ADC_CH1:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(7) | AD7124_CH_MAP_REG_AINM(17));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_1].value = mask;
         break;
      case ADC_CH2:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(8) | AD7124_CH_MAP_REG_AINM(17));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_2].value = mask;
         break;
      case ADC_CH3:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(9) | AD7124_CH_MAP_REG_AINM(17));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_3].value = mask;
         break;
      case ADC_CH4:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(10) | AD7124_CH_MAP_REG_AINM(17));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_4].value = mask;
         break;
      case ADC_CH5:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(11) | AD7124_CH_MAP_REG_AINM(17));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_5].value = mask;
         break;
      case ADC_CH6:
         mask = (ADC_SET_CH(ch_en) | AD7124_CH_MAP_REG_SETUP(1) |
                 AD7124_CH_MAP_REG_AINP(12) | AD7124_CH_MAP_REG_AINM(13));
         cn0411_dev->ad7124_dev.regs[AD7124_Channel_6].value = mask;
         break;
      default:
         return CN0411_FAILURE;
   }

   return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                               cn0411_dev->ad7124_dev.regs[AD7124_Channel_0 + channel]);
}

/**
 * CN0411 ADC IO Control 1 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @param ch_gain - select gain channel to be used.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_set_io1(struct cn0411_device *cn0411_dev, uint8_t ch_gain) {
   int32_t mask = 0;

   mask = (((int32_t)(ch_gain + 1) << 20) |
           (AD7124_8_IO_CTRL1_REG_GPIO_CTRL3 |
            AD7124_8_IO_CTRL1_REG_GPIO_CTRL2 |
            AD7124_8_IO_CTRL1_REG_GPIO_CTRL1) |
           AD7124_IO_CTRL1_REG_IOUT0(3) |
           AD7124_IO_CTRL1_REG_IOUT_CH0(0));

   cn0411_dev->ad7124_dev.regs[AD7124_IOCon1].value = mask;

   return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                               cn0411_dev->ad7124_dev.regs[AD7124_IOCon1]);
}

/**
 * CN0411 ADC IO Control 2 configuration function
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 **/
int32_t CN0411_ADC_set_io2(struct cn0411_device *cn0411_dev) {
   /* All VBias off */
   cn0411_dev->ad7124_dev.regs[AD7124_IOCon2].value = 0x0000;

   return AD7124_WriteRegister(&cn0411_dev->ad7124_dev,
                               cn0411_dev->ad7124_dev.regs[AD7124_IOCon2]);
}

/**
 * CN0411 ADC conversion mode initialization
 *
 * @param cn0411_dev - The device structure.
 * @param conv_mod - set conversion mode
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_conv_init(struct cn0411_device *cn0411_dev, uint8_t conv_mod) {
   switch (conv_mod) {
      case ADC_SINGLE_CONV:
         for (uint8_t ch = ADC_CH0; ch <= ADC_CH5; ch++)
            if (CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_DISABLE) ==
                CN0411_FAILURE)
               return CN0411_FAILURE;

         if (CN0411_ADC_operation_mode(cn0411_dev, AD7124_IDLE_MODE) == CN0411_FAILURE)
            return CN0411_FAILURE;

         break;
      case ADC_CONTINUOUS_CONV:
         for (int ch = ADC_CH0; ch <= ADC_CH5; ch++)
            if (CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_ENABLE) ==
                CN0411_FAILURE)
               return CN0411_FAILURE;

         if (CN0411_ADC_operation_mode(cn0411_dev, AD7124_CONTINUOUS_CONV) ==
             CN0411_FAILURE)
            return CN0411_FAILURE;

         break;
      default:
         return CN0411_FAILURE;
   }

   return CN0411_SUCCESS;
}

/**
 * CN0411 read ADC CHANNEL
 *
 * Based on the conversion mode used:
 * 	- continuous: since the function is called by the user and the sequence
 * 	  of channels conversions cannot be controlled individually, the function
 * 	  assures that the conversion data is read properly, gathering the data only
 * 	  after the transition is made on the desired ADC channel.
 * 	- single: read channel one channel individually
 *
 * @param cn0411_dev - The device structure.
 * @param ch - ADC channel
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_read_ch(struct cn0411_device *cn0411_dev, uint8_t ch) {
   int32_t ret, timeout;
   uint32_t status_reg;

   switch (cn0411_dev->conv_type) {
      case ADC_CONTINUOUS_CONV:
         timeout = 4000000; /* 4000000 * 5uS = 20s */
         do {
            if (AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
                                    &cn0411_dev->ad7124_dev.regs[AD7124_Status]) ==
                CN0411_FAILURE)
               return CN0411_FAILURE;

            status_reg = cn0411_dev->ad7124_dev.regs[AD7124_Status].value &
                         ADC_CH_RDY_MSK;
            timer_sleep_5uS(1u);
         } while ((status_reg == ch) && (--timeout));

         if (timeout == 0)
            return CN0411_FAILURE;

         timeout = 4000000;
         do {
            if (AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
                                    &cn0411_dev->ad7124_dev.regs[AD7124_Status]) ==
                CN0411_FAILURE)
               return CN0411_FAILURE;

            status_reg = cn0411_dev->ad7124_dev.regs[AD7124_Status].value &
                         ADC_CH_RDY_MSK;
            timer_sleep_5uS(1u);
         } while (status_reg != ch && (--timeout));

         if (timeout == 0)
            return CN0411_FAILURE;

         if (AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
                                 &cn0411_dev->ad7124_dev.regs[AD7124_Data]) ==
             CN0411_FAILURE)
            return CN0411_FAILURE;

         break;
      case ADC_SINGLE_CONV:
         ret = CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_ENABLE);
         if (ret == CN0411_FAILURE)
            return ret;

         ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_SINGLE_CONV);
         if (ret == CN0411_FAILURE)
            return ret;

         if (AD7124_WaitForConvReady(&cn0411_dev->ad7124_dev, ADC_TIMEOUT) ==
             TIMEOUT) {
            Serial.print(F("TIMEOUT\n"));
            return CN0411_FAILURE;
         }
         ret = AD7124_ReadRegister(&cn0411_dev->ad7124_dev,
                                   &cn0411_dev->ad7124_dev.regs[AD7124_Data]);
         if (ret == CN0411_FAILURE)
            return ret;

         ret = CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_DISABLE);
         if (ret == CN0411_FAILURE)
            return ret;

         break;
      default:
         return CN0411_FAILURE;
   }

   return ret;
}

/**
 * CN0411 read NTC temperature
 *
 * Reads channel 7 of the ADC, calculates the resistance, and then
 * determines the temperature using the Steinhart-Hart equation.
 *
 * Steinhart-Hart equation:
 * 1/T = A + B * ln(R) + C * (ln(R))^3
 * Where:
 * - T is the temperature in Kelvin
 * - R is the resistance of the NTC thermistor
 * - SH_A, SH_B, and SH_C are Steinhart-Hart coefficients (dependent on the NTC model)
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_read_temp(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   uint32_t ch6_data;

   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH6);
   if (ret == CN0411_FAILURE)
      return ret;

   ch6_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

   // Converte a leitura do ADC para tensão (em V)
   float v_r = VREF - (float)(ch6_data)*VREF / 0xFFFFFF;

   // Calcula corrente total no divisor
   float current = v_r / PTC_REF_RES;

   // Calcula a tensão sobre o NTC (diferença de potencial até VREF)
   // float v_ntc = VREF - v_r;
   float v_ntc = (float)(ch6_data)*VREF / 0xFFFFFF;

   // Calcula a resistência do NTC
   float resistance = v_ntc / current;

   /* Steinhart-Hart coefficients for the specific NTC */
   const float SH_A = 1.009249522e-03;
   const float SH_B = 2.378405444e-04;
   const float SH_C = 2.019202697e-07;

   /* Compute temperature using Steinhart-Hart equation */
   float log_r = log(resistance);
   cn0411_dev->temp = 1.0 / (SH_A + (SH_B * log_r) + (SH_C * log_r * log_r * log_r));
   cn0411_dev->temp -= 273.15;  // Convert from Kelvin to Celsius

   return ret;
}

/**
 * CN0411 read peak-to-peak voltage
 *
 * Reads channel 1 and 2 of the ADC
 *
 * @param cn0411_dev - The device structure.
 *
 * @return peak-to-peak voltage value
 */
int32_t CN0411_read_vpp(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   uint32_t ch1_data = 0, ch2_data = 0;

   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH1);
   if (ret == CN0411_FAILURE)
      return ret;

   ch1_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;
   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH2);
   if (ret == CN0411_FAILURE)
      return ret;

   ch2_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

   cn0411_dev->vin_p = (ch1_data * VREF / 0xFFFFFF) / INSTR_AMP_GAIN;
   cn0411_dev->vin_n = (ch2_data * VREF / 0xFFFFFF) / INSTR_AMP_GAIN;
   cn0411_dev->vpp = cn0411_dev->vin_p + cn0411_dev->vin_n;

   return ret;
}

/**
 * CN0411 read DAC value
 *
 * Reads channel 3 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_read_vdac(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   uint32_t ch3_data = 0;

   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH3);
   if (ret == CN0411_FAILURE)
      return ret;

   ch3_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

   cn0411_dev->read_dac = ch3_data * VREF / 0xFFFFFF;

   return ret;
}

/**
 * CN0411 read voltage value hooked to R20S
 *
 * Reads channel 4 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_read_R20S(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   uint32_t ch4_data = 0;

   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH4);
   if (ret == CN0411_FAILURE)
      return ret;

   ch4_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

   cn0411_dev->read_v_r20s = ch4_data * VREF / 0xFFFFFF;

   return ret;
}

/**
 * CN0411 read voltage value hooked to R200S
 *
 * Reads channel 5 of the ADC
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_read_R200S(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   uint32_t ch5_data = 0;

   ret = CN0411_ADC_read_ch(cn0411_dev, ADC_CH5);
   if (ret == CN0411_FAILURE)
      return ret;

   ch5_data = cn0411_dev->ad7124_dev.regs[AD7124_Data].value;

   cn0411_dev->read_v_r200s = ch5_data * VREF / 0xFFFFFF;

   return ret;
}

/**
 * CN0411 compute input resistance
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_compute_rdres(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   float ipp;

   ret = CN0411_read_vpp(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ipp = (2 * cn0411_dev->v_dac - cn0411_dev->vpp) / cn0411_dev->r_gain[cn0411_dev->ch_gain];
   cn0411_dev->rdres = cn0411_dev->vpp / ipp;

   return ret;
}

/**
 * CN0411 compute electric conductivity
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_compute_cond(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   float ipp, g;

   ret = CN0411_premeasurement(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_read_vpp(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ipp = (2 * cn0411_dev->v_exc - cn0411_dev->vpp) / cn0411_dev->r_gain[cn0411_dev->ch_gain];
   cn0411_dev->rdres = cn0411_dev->vpp / ipp - (cn0411_dev->offset_res);
   g = 1 / cn0411_dev->rdres;
   cn0411_dev->cond = cn0411_dev->cell_const * g;

   return ret;
}

/**
 * CN0411 compute offset resistance
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_compute_off_res(struct cn0411_device *cn0411_dev) {
   int32_t ret;
   float ipp;

   ret = CN0411_premeasurement(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_read_vpp(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ipp = (2 * cn0411_dev->v_exc - cn0411_dev->vpp) / cn0411_dev->r_gain[cn0411_dev->ch_gain];
   cn0411_dev->offset_res = cn0411_dev->vpp / ipp - PREC_REF_RES;

   return ret;
}

/**
 * CN0411 temperature compensation of conductivity
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_compensate_cond(struct cn0411_device *cn0411_dev) {
   int32_t ret;

   ret = CN0411_read_temp(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_compute_cond(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   cn0411_dev->comp_cond = cn0411_dev->cond / (1 + cn0411_dev->solution.temp_coeff * (cn0411_dev->temp - TCAL));

   return ret;
}

/**
 * CN0411 compute TDS
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_compute_tds(struct cn0411_device *cn0411_dev) {
   if (CN0411_compensate_cond(cn0411_dev) == CN0411_FAILURE)
      return CN0411_FAILURE;

   cn0411_dev->tds = cn0411_dev->solution.tds_factor * cn0411_dev->comp_cond;

   return CN0411_SUCCESS;
}

/**
 * CN0411 ADC channels internal software calibration
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_int_calibrate(struct cn0411_device *cn0411_dev) {
   int32_t ret;

   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH0, ADC_CH_ENABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_CAL_INT_ZERO_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH0, ADC_CH_DISABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH1, ADC_CH_ENABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_CAL_INT_ZERO_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH1, ADC_CH_DISABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH2, ADC_CH_ENABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_CAL_INT_ZERO_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH2, ADC_CH_DISABLE);

   return ret;
}

/**
 * CN0411 ADC channels system calibration
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_ADC_sys_calibrate(struct cn0411_device *cn0411_dev) {
   int32_t ret;

   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_IDLE_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   for (uint8_t ch = ADC_CH0; ch <= ADC_CH2; ch++) {
      ret = CN0411_ADC_set_ch(cn0411_dev, ch, ADC_CH_DISABLE);
      if (ret == CN0411_FAILURE)
         return ret;
   }
   pwm_status = PWM_SYSCALIB_AIN7;
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH1, ADC_CH_ENABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_CAL_SYS_ZERO_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH1, ADC_CH_DISABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   pwm_status = PWM_SYSCALIB_AIN8;
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH2, ADC_CH_ENABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_operation_mode(cn0411_dev, AD7124_CAL_SYS_ZERO_MODE);
   if (ret == CN0411_FAILURE)
      return ret;

   timer_sleep(1000u);
   ret = CN0411_ADC_set_ch(cn0411_dev, ADC_CH2, ADC_CH_DISABLE);
   if (ret == CN0411_FAILURE)
      return ret;

   pwm_status = PWM_CONVERSION;
   ret = CN0411_ADC_conv_init(cn0411_dev, cn0411_dev->conv_type);

   return ret;
}

/**
 * CN0411 Premeasurement Process
 *
 * Autoset gain resistor channel based on the read DAC value
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_premeasurement(struct cn0411_device *cn0411_dev) {
   int32_t ret;

   // cn0411_dev->ch_gain = CH_GAIN_RES_20M;
   // ret = CN0411_read_vdac(cn0411_dev);
   // if (ret == CN0411_FAILURE)
   //    return ret;

   cn0411_dev->v_exc = cn0411_dev->read_dac;
   ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_exc);
   if (ret == CN0411_FAILURE)
      return ret;

   // while (cn0411_dev->ch_gain >= 1) {
   //    ret = CN0411_ADC_set_io1(cn0411_dev, cn0411_dev->ch_gain);
   //    if (ret == CN0411_FAILURE)
   //       return ret;

   //    ret = CN0411_read_vpp(cn0411_dev);
   //    if (ret == CN0411_FAILURE)
   //       return ret;

   //    if ((cn0411_dev->vpp > 0.3 * 2 * cn0411_dev->v_exc) || (cn0411_dev->ch_gain == 1)) {
   //       cn0411_dev->v_exc = 0.4 * cn0411_dev->v_exc / cn0411_dev->vpp;
   //       ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_exc);
   //       if (ret == CN0411_FAILURE)
   //          return ret;

   //       break;
   //    } else {
   //       cn0411_dev->ch_gain--;
   //    }
   // }

   return ret;
}

/**
 * CN0411 Set Gain Resistance
 *
 * @param cn0411_dev - The device structure.
 * @param gain_res - gain resistor channel value
 * @return 0 in case of success, negative error code otherwise.
 */

int32_t CN0411_set_gain_res(struct cn0411_device *cn0411_dev, int8_t ch_gain) {
   cn0411_dev->ch_gain = ch_gain;

   return CN0411_ADC_set_io1(cn0411_dev, cn0411_dev->ch_gain);
}

/**
 * Set PWM frequency
 *
 * @param freq - frequency value to be set.
 * @return none
 */
void CN0411_pwm_freq(uint16_t freq, uint8_t duty) {
   switch (freq) {
      case PWM_FREQ_94:
         ledcSetup(PWM_CHANNEL_MAIN, freq, 10);
         ledcAttachPin(PWM_MAIN_PIN, PWM_CHANNEL_MAIN);
         ledcWrite(PWM_CHANNEL_MAIN, (duty * 1023) / 100);
         break;
      case PWM_FREQ_2400:
         ledcSetup(PWM_CHANNEL_MAIN, freq, 10);
         ledcAttachPin(PWM_MAIN_PIN, PWM_CHANNEL_MAIN);
         ledcWrite(PWM_CHANNEL_MAIN, (duty * 1023) / 100);
         break;
      default:
         break;
   }
}

/**
 * Generate PWM

 * @return none
*/
void CN0411_pwm_gen(void) {
   pinMode(PWM_MAIN_PIN, OUTPUT);
   pinMode(PWM_POS_PIN, OUTPUT);
   pinMode(PWM_NEG_PIN, OUTPUT);
}

/**
 * CN0411 Initialization
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t CN0411_init(struct cn0411_device *cn0411_dev,
                    struct cn0411_init_params cn0411_init_params) {
   int32_t ret;

   /* Initialize Device Structure */
   cn0411_dev->ch_gain = cn0411_init_params.init_ch_gain;
   cn0411_dev->conv_type = cn0411_init_params.init_conv_type;
   cn0411_dev->rtd_res = cn0411_init_params.init_rtd_res;
   cn0411_dev->r_gain[CH_GAIN_RES_20] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_20];
   cn0411_dev->r_gain[CH_GAIN_RES_200] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_200];
   cn0411_dev->r_gain[CH_GAIN_RES_2K] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_2K];
   cn0411_dev->r_gain[CH_GAIN_RES_20K] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_20K];
   cn0411_dev->r_gain[CH_GAIN_RES_200K] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_200K];
   cn0411_dev->r_gain[CH_GAIN_RES_2M] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_2M];
   cn0411_dev->r_gain[CH_GAIN_RES_20M] =
       cn0411_init_params.init_r_gain[CH_GAIN_RES_20M];
   cn0411_dev->offset_res = cn0411_init_params.init_offset_res;
   cn0411_dev->v_dac = cn0411_init_params.init_v_dac;
   cn0411_dev->read_dac = cn0411_init_params.init_read_dac;
   cn0411_dev->read_v_r20s = cn0411_init_params.init_read_v_r20s;
   cn0411_dev->read_v_r200s = cn0411_init_params.init_read_v_r200s;
   cn0411_dev->rdres = cn0411_init_params.init_rdres;
   cn0411_dev->v_exc = cn0411_init_params.init_v_exc;
   cn0411_dev->cell_const = cn0411_init_params.init_cell_const;
   cn0411_dev->temp = cn0411_init_params.init_temp;
   cn0411_dev->vin_p = cn0411_init_params.init_vin_p;
   cn0411_dev->vin_n = cn0411_init_params.init_vin_n;
   cn0411_dev->cond = cn0411_init_params.init_cond;
   cn0411_dev->comp_cond = cn0411_init_params.init_comp_cond;
   cn0411_dev->tds = cn0411_init_params.init_tds;
   cn0411_dev->solution.tds_factor =
       cn0411_init_params.init_solution.init_tds_factor;
   cn0411_dev->solution.temp_coeff =
       cn0411_init_params.init_solution.init_temp_coeff;

   /* Initialize PWM */
   CN0411_pwm_gen();
   pwm_status = PWM_CONVERSION;

   /* Initial Setup ADC */
   ret = AD7124_Setup(&cn0411_dev->ad7124_dev, CS_AD7124, ad7124_regs);
   if (ret == CN0411_FAILURE)
      return ret;

   /* Initial Setup DAC */
   ret = AD5683_setup(&cn0411_dev->ad5683_dev, CS_AD5683);
   if (ret == CN0411_FAILURE)
      return ret;

   /* Setup ADC */
   ret = CN0411_ADC_setup(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_set_io1(cn0411_dev, cn0411_dev->ch_gain);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_set_io2(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_int_calibrate(cn0411_dev);
   if (ret == CN0411_FAILURE)
      return ret;

   ret = CN0411_ADC_conv_init(cn0411_dev, cn0411_dev->conv_type);
   if (ret == CN0411_FAILURE)
      return ret;

   /* Set DAC output value */
   ret = CN0411_DAC_set_value(cn0411_dev, cn0411_dev->v_dac);
   if (ret == CN0411_FAILURE)
      return ret;

   return ret;
}