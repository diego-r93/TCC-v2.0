#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include "SPI.h"

/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

#define ADI_PAR_CS_PIN 38  // Pin connected to the ADC Chip Select pin (not the SPI library 10 pin, which is not used here)
#define ADI_PART_CS_PIN_OUT pinMode(ADI_PAR_CS_PIN, OUTPUT)
#define ADI_PART_CS_LOW digitalWrite(ADI_PAR_CS_PIN, LOW)
#define ADI_PART_CS_HIGH digitalWrite(ADI_PAR_CS_PIN, HIGH)
#define SDO_PIN 13  // MISO Pin connected to the ADC Dout/Data Ready pin
#define SDO_PIN_IN pinMode(SDO_PIN, INPUT)
#define GPIO1_STATE digitalRead(SDO_PIN)
#define SPI_SETTINGS SPISettings(4000000, MSBFIRST, SPI_MODE3)  // 4 MHz clock, MSB first, mode 0

#define SDI_PIN 11  // MOSI Pin connected to the ADC Din pin
#define SCK_PIN 12  // CLK Pin connected to the ADC SCLK pin

/*************************** Functions prototypes *****************************/
void SPI_Init(void);
void SPI_Write(uint8_t ui8address, uint32_t ui32data, uint8_t ui8bytes);
uint32_t SPI_Read(uint8_t ui8address, uint8_t ui8bytes);

#endif  // _COMMUNICATION_H_
