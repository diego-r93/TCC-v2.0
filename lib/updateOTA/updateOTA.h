#ifndef UPDATEOTA
#define UPDATEOTA

#include "Arduino.h"
#include "AsyncTCP.h"
#include "ESPAsyncWebServer.h"
#include "FS.h"
#include "Update.h"
#include "WiFi.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
#include "stdlib_noniso.h"

class AsyncElegantOtaClass {
  public:
   void
   setID(const char* id),
       begin(AsyncWebServer* server, const char* username = "", const char* password = ""),
       restart();

  private:
   AsyncWebServer* _server;

   String getID();

   String _id = getID();
   String _username = "";
   String _password = "";
   bool _authRequired = false;
};

extern AsyncElegantOtaClass AsyncElegantOTA;

#endif