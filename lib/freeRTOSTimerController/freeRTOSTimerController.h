#ifndef _FREERTOSTIMERCONTROLLER_
#define _FREERTOSTIMERCONTROLLER_

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

class FreeRTOSTimer {
  public:
   FreeRTOSTimer(const char *name, TickType_t period, UBaseType_t auto_reload, void *pvTimerID, TimerCallbackFunction_t callback);

   ~FreeRTOSTimer();

   void start();

   void stop();

   void reset();

   void changePeriod(TickType_t newPeriod);

  private:
   TimerHandle_t timer;
};

#endif