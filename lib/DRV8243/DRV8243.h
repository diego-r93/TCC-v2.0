// -----------------------------------------------------------------------------
// DRV8243 Daisy‑Chain Library (two devices)
// -----------------------------------------------------------------------------
// This single header/implementation pair provides a minimal API focused on the
// SPI daisy‑chain mode of the DRV8243P‑Q1.  All legacy single‑device helpers
// have been removed to keep the codebase lean and purpose‑built.
// -----------------------------------------------------------------------------
// Copyright (c) 2025 Diego Rodrigues
// SPDX‑License‑Identifier: MIT
// -----------------------------------------------------------------------------

/******************************  DRV8243.h  *********************************/
#ifndef _DRV8243_H_
#define _DRV8243_H_

#include <Arduino.h>
#include <stdint.h>

// Number of devices in the daisy‑chain (fixed: 2)
#define NUM_DEVICES 2

/// DRV8243P‑Q1 SPI command byte format
#define DRV_OUT_LL 0b00  // OUT1 = L, OUT2 = L
#define DRV_OUT_LH 0b01  // OUT1 = L, OUT2 = H
#define DRV_OUT_HL 0b10  // OUT1 = H, OUT2 = L
#define DRV_OUT_HH 0b11  // OUT1 = H, OUT2 = H

/* ---- Register addresses ---------------------------------------------------- */
#define DRV8243_REG_DEVICE_ID 0x00      // 0b00000000
#define DRV8243_REG_FAULT_SUMMARY 0x01  // 0b00000001
#define DRV8243_REG_STATUS1 0x02        // 0b00000010
#define DRV8243_REG_STATUS2 0x03        // 0b00000011
#define DRV8243_REG_COMMAND 0x08        // 0b00001000
#define DRV8243_REG_SPI_IN 0x09         // 0b00001001
#define DRV8243_REG_CONFIG1 0x0A        // 0b00001010
#define DRV8243_REG_CONFIG2 0x0B        // 0b00001011
#define DRV8243_REG_CONFIG3 0x0C        // 0b00001100
#define DRV8243_REG_CONFIG4 0x0D        // 0b00001101

/*************************** Functions prototypes *****************************/

bool DRV8243_Init(void);

/* --- Daisy-chain helpers (2 dispositivos) ------------------------------ */
bool DRV8243_DaisyWrite(uint8_t addrDev1, uint8_t dataDev1,
                        uint8_t addrDev0, uint8_t dataDev0);

bool DRV8243_DaisyRead(uint8_t addrDev1, uint8_t addrDev0,
                       uint8_t &repDev1, uint8_t &repDev0,
                       uint8_t &statDev1, uint8_t &statDev0);

bool DRV8243_SetupHalfBridge(void);
bool DRV8243_SetMotors(uint8_t m1, uint8_t m2, uint8_t m3, uint8_t m4);
void DRV8243_DumpRegisters(void);

#endif