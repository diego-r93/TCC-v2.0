#include "freeRTOSTimerController.h"

FreeRTOSTimer::FreeRTOSTimer(const char *name, TickType_t period, UBaseType_t auto_reload, void *pvTimerID, TimerCallbackFunction_t callback) {
   timer = xTimerCreate(name, period / portTICK_PERIOD_MS, auto_reload, pvTimerID, callback);
}

FreeRTOSTimer::~FreeRTOSTimer() {
   if (xTimerIsTimerActive(timer)) {
      xTimerStop(timer, 0);
   }
   xTimerDelete(timer, 0);
}

void FreeRTOSTimer::start() {
   if (!xTimerIsTimerActive(timer))
      xTimerStart(timer, 0);
}

void FreeRTOSTimer::stop() {
   if (xTimerIsTimerActive(timer))
      xTimerStop(timer, 0);
}

void FreeRTOSTimer::reset() {
   if (xTimerIsTimerActive(timer))
      xTimerReset(timer, 0);
}

void FreeRTOSTimer::changePeriod(TickType_t newPeriod) {
   if (xTimerIsTimerActive(timer)) {
      xTimerStop(timer, 0);
   }
   xTimerChangePeriod(timer, newPeriod / portTICK_PERIOD_MS, portMAX_DELAY);
}