// File: DRV8243Controller.cpp
/**
 * @file DRV8243Controller.cpp
 * @brief Implementation of DRV8243Controller class.
 * @date 2025-06-23
 *
 * Uses DRV8243_Init, DRV8243_SetupHalfBridge, and DRV8243_SetMotors
 * from DRV8243 C API for daisy-chain control of two devices.
 *
 * © MIT License
 * @author Diego Rodrigues
 */

#include "DRV8243Controller.h"

// Static motor ID mapping (0=M1,1=M2 on device0; 2=M1,3=M2 on device1)
const uint8_t DRV8243Controller::_motorIDs[NUMBER_OUTPUTS] = {0, 1, 2, 3};

DRV8243Controller::DRV8243Controller() {
   for (uint8_t i = 0; i < NUMBER_OUTPUTS; ++i) {
      _states[i] = 0;  // OFF by default
   }
}

bool DRV8243Controller::begin() {
   if (!DRV8243_Init())
      Serial.println("[ERROR] DRV8243 init failed - check wiring and power");
   else
      Serial.println("DRV8243 init OK");
   
   return DRV8243_SetupHalfBridge();
}

void DRV8243Controller::setMotorState(uint8_t index, bool on) {
   if (index >= NUMBER_OUTPUTS) return;
   _states[index] = on ? 1 : 0;
}

bool DRV8243Controller::getMotorState(uint8_t index) const {
   if (index >= NUMBER_OUTPUTS) return false;
   return (_states[index] != 0);
}

bool DRV8243Controller::apply() {
   // Map internal states to DRV8243_SetMotors parameters
   uint8_t m1 = _states[0];
   uint8_t m2 = _states[1];
   uint8_t m3 = _states[2];
   uint8_t m4 = _states[3];
   return DRV8243_SetMotors(m1, m2, m3, m4);
}
