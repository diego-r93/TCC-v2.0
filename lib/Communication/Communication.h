// -----------------------------------------------------------------------------
//                      DRV8243 Daisy‑Chain Library
//                (two devices, SPI‑P variant, daisy‑chain only)
// -----------------------------------------------------------------------------
// This single file contains the header and implementation for driving two
// DRV8243P‑Q1 devices in daisy‑chain mode.  All low‑level SPI transactions are
// handled by helper functions provided by Communication.h / Communication.cpp
// so that none of the DRV8243 functions call SPI.transfer() directly.
// -----------------------------------------------------------------------------
// Copyright (c) 2025 Diego Rodrigues
// SPDX‑License‑Identifier: MIT
// -----------------------------------------------------------------------------

/******************************  Communication.h  *****************************/
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

#include <Arduino.h>
#include <SPI.h>

#define AD7793_SPI_SETTINGS SPISettings(4000000, MSBFIRST, SPI_MODE3)  // 4 MHz clock, MSB first, mode 3
#define DRV8243_CS_SPI_SETTINGS SPISettings(4000000, MSBFIRST, SPI_MODE1) // 4 MHz clock, MSB first, mode 1

// Pin mapping
#define SCK_PIN 12
#define SDO_PIN 13
#define SDI_PIN 11

#define AD7793_CS_PIN 38
#define AD7793_CS_PIN_OUT pinMode(AD7793_CS_PIN, OUTPUT)
#define AD7793_CS_LOW digitalWrite(AD7793_CS_PIN, LOW)
#define AD7793_CS_HIGH digitalWrite(AD7793_CS_PIN, HIGH)

#define DRV8243_CS_PIN 17
#define DRV8243_CS_PIN_OUT pinMode(DRV8243_CS_PIN, OUTPUT)
#define DRV8243_CS_LOW digitalWrite(DRV8243_CS_PIN, LOW)
#define DRV8243_CS_HIGH digitalWrite(DRV8243_CS_PIN, HIGH)
#define DRV8243_CS_PIN_INIT pinMode(DRV8243_CS_PIN, OUTPUT)

#define SDO_PIN_IN pinMode(SDO_PIN, INPUT)
#define GPIO1_STATE digitalRead(SDO_PIN)

void SPI_Init(void);
void    SPI_Write(uint8_t addr, uint32_t data, uint8_t nbytes);
uint32_t SPI_Read(uint8_t addr, uint8_t nbytes);
void SPI_TransferFrame(const uint8_t *tx, uint8_t *rx, uint8_t len);

#endif