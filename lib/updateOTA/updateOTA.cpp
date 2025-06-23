#include "updateOTA.h"

AsyncElegantOtaClass AsyncElegantOTA;

void AsyncElegantOtaClass::setID(const char *id) {
   _id = id;
}

void AsyncElegantOtaClass::begin(AsyncWebServer *server, const char *username, const char *password) {
   _server = server;

   if (strlen(username) > 0) {
      _authRequired = true;
      _username = username;
      _password = password;
   } else {
      _authRequired = false;
      _username = "";
      _password = "";
   }

   _server->on("/update/identity", HTTP_GET, [&](AsyncWebServerRequest *request) {
      if (_authRequired) {
         if (!request->authenticate(_username.c_str(), _password.c_str())) {
            return request->requestAuthentication();
         }
      }

      request->send(200, "application/json", "{\"id\": \"" + _id + "\", \"hardware\": \"ESP32\"}");
   });   

   _server->on("/update", HTTP_POST, [&](AsyncWebServerRequest *request) {
      if(_authRequired){
         if(!request->authenticate(_username.c_str(), _password.c_str())){
            return request->requestAuthentication();
         }
      }
      // the request handler is triggered after the upload has finished... 
      // create the response, add header, and send response
      AsyncWebServerResponse *response = request->beginResponse((Update.hasError())?500:200, "text/plain", (Update.hasError())?"FAIL":"OK");
      response->addHeader("Connection", "close");
      response->addHeader("Access-Control-Allow-Origin", "*");
      request->send(response);
      restart(); },
       [&](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      // Upload handler chunks in data
      if(_authRequired){
         if(!request->authenticate(_username.c_str(), _password.c_str())){
            return request->requestAuthentication();
         }
      }

      if (!index) {
         if(!request->hasParam("MD5", true)) {
            return request->send(400, "text/plain", "MD5 parameter missing");
         }

         if(!Update.setMD5(request->getParam("MD5", true)->value().c_str())) {
            return request->send(400, "text/plain", "MD5 parameter invalid");
         }

         int cmd = (filename == "firmware.bin") ? U_FLASH : U_SPIFFS;
         if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) { // Start with max available size
            Update.printError(Serial);
            return request->send(400, "text/plain", "OTA could not begin");
         }
      }

      // Write chunked data to the free sketch space
      if(len){
         if (Update.write(data, len) != len) {
            return request->send(400, "text/plain", "OTA could not begin");
         }
      }
            
      if (final) { // if the final flag is set then this is the last frame of data
         if (!Update.end(true)) { //true to set the size to the current progress
            Update.printError(Serial);
            return request->send(400, "text/plain", "Could not end OTA");
         }
      } else{
         return;
      } });
}

void AsyncElegantOtaClass::restart() {
   yield();
   delay(1000);
   yield();
   ESP.restart();
}

String AsyncElegantOtaClass::getID() {
   String id = "";
   id = String((uint32_t)ESP.getEfuseMac(), HEX);
   id.toUpperCase();
   return id;
}