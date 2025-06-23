// File: DRV8243Controller.h
/**
 * @file DRV8243Controller.h
 * @brief C++ class to control DRV8243P-Q1 via SPI, encapsulating motor states.
 * @date 2025-06-23
 *
 * Encapsulates internal storage of motor IDs and states, and applies them
 * via the DRV8243 C API (daisy-chain mode).
 *
 * © MIT License
 * @author Diego Rodrigues
 */

#ifndef _DRV8243_CONTROLLER_H_
#define _DRV8243_CONTROLLER_H_

#include <Arduino.h>
#include <stdint.h>

#include "DRV8243.h"  // Low-level C API

#define NUMBER_OUTPUTS 4

/**
 * @brief Controller for up to NUMBER_OUTPUTS motors via DRV8243 SPI daisy-chain.
 */
class DRV8243Controller {
  public:
   /**
    * @brief Constructs controller, initializes internal states to OFF.
    */
   DRV8243Controller();

   /**
    * @brief Initialize SPI and configure half-bridge.
    * @return true on success, false otherwise.
    */
   bool begin();

   /**
    * @brief Set a specific motor ON or OFF.
    * @param index Motor index [0..NUMBER_OUTPUTS-1]
    * @param on true to turn ON, false to turn OFF
    */
   void setMotorState(uint8_t index, bool on);

   /**
    * @brief Get last commanded state of a motor.
    * @param index Motor index
    * @return true if ON, false if OFF
    */
   bool getMotorState(uint8_t index) const;

   /**
    * @brief Apply all motor states to hardware via DRV8243_SetMotors.
    * @return true if SPI transaction succeeded.
    */
   bool apply();

  private:
   static const uint8_t _motorIDs[NUMBER_OUTPUTS];
   uint8_t _states[NUMBER_OUTPUTS];
};

#endif  // _DRV8243_CONTROLLER_H_