// File: hydraulicPumpController.h
/**
 * @file hydraulicPumpController.h
 * @brief Class to control hydraulic pumps via GPIO using FreeRTOS timers.
 * @date 2025-06-23
 *
 * Provides methods to start and stop pumps with a specified pulse duration,
 * record activation times, and serialize state to JSON.
 *
 * © MIT License
 * @author Diego Rodrigues
 */

#ifndef _HYDRAULIC_PUMP_CONTROLLER_H_
#define _HYDRAULIC_PUMP_CONTROLLER_H_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <set>

#include "DRV8243Controller.h"
#include "freeRTOSTimerController.h"

class HydraulicPumpController {
  public:
   /**
    * @brief Construct a new HydraulicPumpController object.
    * @param pumperCode     Unique code identifying the pump.
    * @param controller     Reference to the DRV8243Controller instance.
    * @param motorIndex     Motor index [0..NUMBER_OUTPUTS-1] for DRV8243.
    * @param pulseDuration  Duration of pump activation in RTOS ticks.
    */

   TickType_t *pulseDurationPointer = &pulseDuration;

   HydraulicPumpController(const char *pumperCode,
                           DRV8243Controller &controller,
                           uint8_t motorIndex,
                           TickType_t pulseDuration);

   /**
    * @brief Start the pump via DRV8243 for the configured pulse duration.
    */
   void startPump();

   /**
    * @brief Stop the pump immediately via DRV8243.
    */
   void stopPump();

   /**
    * @brief Get current state of the pump.
    * @return true if pump is ON, false otherwise.
    */
   bool getPumpState() const;

   /**
    * @brief Get the configured pulse duration.
    * @return Pulse duration in RTOS ticks.
    */
   TickType_t getPulseDuration();

   /**
    * @brief Set a new pulse duration.
    * @param newPulseDuration New duration for pump activation in RTOS ticks.
    */
   void setPulseDuration(TickType_t newPulseDuration);

   /**
    * @brief Get unique code associated with this pump.
    * @return C-string representing pump code.
    */
   const char *getCode() const;

   /**
    * @brief Get motor index for DRV8243.
    * @return Motor index.
    */
   uint8_t getMotorIndex() const;

   /**
    * @brief Retrieve recorded activation timestamps.
    * @return Set of timestamp strings.
    */
   std::set<String> getDriveTimes() const;

   std::set<String> *getDriveTimesPointer();

   /**
    * @brief Get a copy of the internal JSON document.
    * @return JsonDocument containing pump information.
    */
   JsonDocument getJsonData();

   /**
    * @brief Get pointer to the internal JSON document.
    * @return Pointer to the JsonDocument member.
    */
   JsonDocument *getJsonDataPointer();

  private:
   const char *pumperCode;
   DRV8243Controller &drvController;
   const uint8_t motorIndex;
   FreeRTOSTimer timer;
   TickType_t pulseDuration;
   bool pumpState = false;
   std::set<String> driveTimes;
   JsonDocument jsonData;

   /**
    * @brief Callback invoked by FreeRTOS timer upon expiration.
    * @param xTimer Handle to the expired timer.
    */
   static void pumpControlCallback(TimerHandle_t xTimer);
};

#endif