#ifndef _PUMP_
#define _PUMP_

#include <Arduino.h>
#include <ArduinoJson.h>

#include <set>

#include "freeRTOSTimerController.h"

class HydraulicPumpController {
  public:
   const char *pumperCode;
   const uint8_t gpioPin;

   TickType_t *pulseDurationPointer = &pulseDuration;

   HydraulicPumpController(const char *pumperCode, uint8_t gpioPin, TickType_t pulseDuration);

   bool getPumpState();

   void startPump();
   void stopPump();

   std::set<String> getDriveTimes();
   std::set<String> *getDriveTimesPointer();

   JsonDocument getJsonData();
   JsonDocument *getJsonDataPointer();

   TickType_t getPulseDuration();
   void setPulseDuration(TickType_t);

   const char *getCode();
   uint8_t getGpio();

  private:
   std::set<String> driveTimes;
   JsonDocument jsonData;
   FreeRTOSTimer timer;

   TickType_t pulseDuration;

   bool pumpState = false;

   static void pumpControlCallback(TimerHandle_t xTimer);
};

#endif