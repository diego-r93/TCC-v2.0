// File: hydraulicPumpController.cpp
/**
 * @file hydraulicPumpController.cpp
 * @brief Implementation of HydraulicPumpController methods using DRV8243.
 * @date 2025-06-23
 *
 * See hydraulicPumpController.h for class documentation.
 *
 * © MIT License
 */

#include "hydraulicPumpController.h"

HydraulicPumpController::HydraulicPumpController(const char *pumperCode,
                                                 DRV8243Controller &controller,
                                                 uint8_t motorIndex,
                                                 TickType_t pulseDuration)
    : pumperCode(pumperCode),
      drvController(controller),
      motorIndex(motorIndex),
      timer("PumpTimer", pulseDuration, pdFALSE, (void *)this, &HydraulicPumpController::pumpControlCallback),
      pulseDuration(pulseDuration) {}

void HydraulicPumpController::startPump() {
   if (!pumpState) {
      pumpState = true;
      drvController.setMotorState(motorIndex, true);
      drvController.apply();
      timer.changePeriod(pulseDuration);
      timer.start();
      driveTimes.insert(String(millis()));
   }
}

void HydraulicPumpController::stopPump() {
   if (pumpState) {
      timer.stop();
      drvController.setMotorState(motorIndex, false);
      drvController.apply();
      pumpState = false;
   }
}

bool HydraulicPumpController::getPumpState() const {
   return pumpState;
}

TickType_t HydraulicPumpController::getPulseDuration() {
   return pulseDuration;
}

void HydraulicPumpController::setPulseDuration(TickType_t newPulseDuration) {
   pulseDuration = newPulseDuration;
}

const char *HydraulicPumpController::getCode() const {
   return pumperCode;
}

uint8_t HydraulicPumpController::getMotorIndex() const {
   return motorIndex;
}

std::set<String> HydraulicPumpController::getDriveTimes() const {
   return driveTimes;
}

std::set<String> *HydraulicPumpController::getDriveTimesPointer() {
   return &driveTimes;
}

JsonDocument HydraulicPumpController::getJsonData() {
   return jsonData;
}

JsonDocument *HydraulicPumpController::getJsonDataPointer() {
   return &jsonData;
}

void HydraulicPumpController::pumpControlCallback(TimerHandle_t xTimer) {
   HydraulicPumpController *controller = (HydraulicPumpController *)pvTimerGetTimerID(xTimer);

   controller->stopPump();
}