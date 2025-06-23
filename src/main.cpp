#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DallasTemperature.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <FS.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <RtcDS3234.h>
#include <SPI.h>
#include <Update.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>

#include "AD7793.h"
#include "CN0326.h"
#include "Communication.h"
#include "DRV8243Controller.h"
#include "NTPClient.h"
#include "PIDController.h"
#include "SHT3x.h"
#include "SPIFFS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hydraulicPumpController.h"
#include "mongoDbAtlas.h"
#include "wifiCredentials.h"

/*
Task                         Core  Prio                     Descrição
--------------------------------------------------------------------------------------------------------------------------
vTaskUpdate                   0     3     Atualiza as informações através de um POST no MongoDB Atlas
vTaskCheckWiFi                0     2     Verifica a conexão WiFi e tenta reconectar caso esteja deconectado
vTaskMqttReconnect            0     2     Verifica a conexão MQTT e tenta reconectar caso esteja deconectado
vTaskNTP                      0     1     Atualiza o horário com base no NTP
vTaskTurnOnPump               1     3     Liga a bomba quando chegar no seu horário de acionamento
vTaskds18b20SensorRead        1     1     Lê o sensor ds18b20
vTaskds18b20DataProcess       1     2     Processa os dados do sensor de ds18b20 e envia para o servidor
vTaskSHT35SensorRead          1     1     Lê o sensor SHT35
vTaskSHT35DataProcess         1     2     Processa os dados do sensor SHT35 e envia para o servidor
vTaskPhSensorRead             1     1     Lê o sensor de pH
vTaskPhDataProcess            1     2     Processa os dados do sensor de pH e envia para o servidor
vTaskTdsSensorRead            1     1     Lê o sensor de TDS
vTaskTdsDataProcess           1     2     Processa os dados do sensor de TDS e envia para o servidor
--------------------------------------------------------------------------------------------------------------------------
*/

// Tasks delays
#define UPDATE_DELAY 300000
#define TURN_ON_PUMP_DELAY 100

#define DS18B20_SENSOR_READ_DELAY 500
#define DS18B20_DATA_PROCESS_DELAY 1000

#define SHT35_SENSOR_READ_DELAY 500
#define SHT35_DATA_PROCESS_DELAY 1000

#define PH_SENSOR_READ_DELAY 5000
#define PH_DATA_PROCESS_DELAY 5000
#define PH_MOTOR_CONTROL_DELAY 500

#define TDS_SENSOR_READ_DELAY 10000
#define TDS_DATA_PROCESS_DELAY 10000
#define TDS_MOTOR_CONTROL_DELAY 500

// Pump Timers configuration
#define NUMBER_OUTPUTS 4
#define ACTIVE_PUMPS 4

// Motors configuration
const uint8_t motorIDs[NUMBER_OUTPUTS] = {0, 1, 2, 3};  // 0=m1, 1=m2, 2=m3, 3=m4
uint8_t myPumpStates[NUMBER_OUTPUTS] = {0, 0, 0, 0};    // Estados dos motores (0=off, 1=on)

DRV8243Controller drvController;

HydraulicPumpController myPumps[ACTIVE_PUMPS] = {
    HydraulicPumpController("code04", motorIDs[0], 60000),
    HydraulicPumpController("code05", motorIDs[1], 60000),
    HydraulicPumpController("code06", motorIDs[2], 60000),
    HydraulicPumpController("code07", motorIDs[3], 60000),
};

// NTP configuration
#define CHECK_WIFI_DELAY 100
#define NTP_DELAY 600000
#define NTP_UPDATE_INTERVAL 12 * 60 * 60 * 1000  // 12 horas

WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, NTP_UPDATE_INTERVAL);

// Set your Static IP address
// IPAddress local_IP(192, 168, 1, 2);
// Set your Gateway IP address
// IPAddress gateway(192, 168, 1, 254);
// Set your SubnetMask
// IPAddress subnet(255, 255, 255, 0);

// MQTT configuration
#define CHECK_MQTT_DELAY 500

String mqtt_server = "192.168.0.100";
const uint16_t mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// DS3234 configuration
#define PIN_SCK3 39    // SCK
#define PIN_MOSI3 42   // SDI
#define PIN_MISO3 21   // SDO
#define PIN_CS_RTC 45  // CS_RTC

SPIClass SPI3;
#define countof(a) (sizeof(a) / sizeof(a[0]))
RtcDS3234<SPIClass> Rtc(SPI3, PIN_CS_RTC);

// ds18b20 configuration
const int oneWireBus = 4;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// SHT35 configuration
using namespace McciCatenaSht3x;
cSHT3x gSht3x{Wire, cSHT3x::Address_t::A};
cSHT3x::Measurements sht35Data;  // Variável global para armazenar os dados do sensor

// WebServer configuration
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// FreeRTOS configuration
SemaphoreHandle_t xWifiMutex;
SemaphoreHandle_t xSPIMutex;
SemaphoreHandle_t xPIDControllerMutex;

// FreeRTOS Task Handles
TaskHandle_t UpdateTaskHandle = NULL;
TaskHandle_t CheckWiFiTaskHandle = NULL;
TaskHandle_t MqttReconnectTaskHandle = NULL;
TaskHandle_t NTPTaskHandle = NULL;
TaskHandle_t TurnOnPumpTaskHandle = NULL;

TaskHandle_t ds18b20SensorReadTaskHandle = NULL;
TaskHandle_t ds18b20DataProcessTaskHandle = NULL;

TaskHandle_t SHT35SensorReadTaskHandle = NULL;
TaskHandle_t SHT35DataProcessTaskHandle = NULL;

TaskHandle_t PhSensorReadTaskHandle = NULL;
TaskHandle_t PhDataProcessTaskHandle = NULL;
TaskHandle_t PhMotorControlTaskHandle = NULL;

TaskHandle_t TdsSensorReadTaskHandle = NULL;
TaskHandle_t TdsDataProcessTaskHandle = NULL;
TaskHandle_t TdsMotorControlTaskHandle = NULL;

// FreeRTOS Task Functions
void vTaskUpdate(void* pvParameters);
void vTaskCheckWiFi(void* pvParametes);
void vTaskMqttReconnect(void* pvParametes);
void vTaskNTP(void* pvParameters);
void vTaskTurnOnPump(void* pvParametes);

void vTaskds18b20SensorRead(void* pvParameters);
void vTaskds18b20DataProcess(void* pvParameters);

void vTaskSHT35SensorRead(void* pvParameters);
void vTaskSHT35DataProcess(void* pvParameters);

void vTaskPhSensorRead(void* pvParameters);
void vTaskPhDataProcess(void* pvParameters);

void vTaskTdsSensorRead(void* pvParameters);
void vTaskTdsDataProcess(void* pvParameters);

String getDateTime(const RtcDateTime& dt) {
   char datestring[26];

   snprintf_P(datestring,
              countof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              dt.Day(),
              dt.Month(),
              dt.Year(),
              dt.Hour(),
              dt.Minute(),
              dt.Second());
   return String(datestring);
}

// Função para converter Unix timestamp para RtcDateTime
RtcDateTime convertEpochToDateTime(unsigned long epoch) {
   tm* ptm = gmtime((time_t*)&epoch);
   RtcDateTime dt(ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
   return dt;
}

String getFormattedDateTime() {
   ntp.update();
   unsigned long epochTime = ntp.getEpochTime();
   struct tm* ptm = gmtime((time_t*)&epochTime);

   char dateBuffer[20];  // Buffer to store the formatted date and time
   // Format: day/month/year hours:minutes:seconds
   sprintf(dateBuffer, "%02d/%02d/%04d %02d:%02d:%02d",
           ptm->tm_mday, ptm->tm_mon + 1, ptm->tm_year + 1900,
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

   return String(dateBuffer);
}

void callback(char* topic, byte* payload, unsigned int length) {
   Serial.print("Message arrived on topic: ");
   Serial.print(topic);
   Serial.print(". Message: ");
   String messageTemp;

   for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
   }
   Serial.println();

   if (String(topic) == String(WiFi.getHostname()) + "/output") {
      JsonDocument doc;
      deserializeJson(doc, messageTemp);

      for (int indice = 0; indice < NUMBER_OUTPUTS; indice++) {
         String key = String(indice);

         if (doc[key].is<bool>()) {
            bool state = doc[key];

            Serial.print("Changing motor ");
            Serial.print(key);
            Serial.print(" to ");
            Serial.println(state ? "on" : "off");

            myPumpStates[indice] = state ? 1 : 0;
            drvController.setMotorState(indice, state);
         }
      }

      printf("Motor states: M1=%d, M2=%d, M3=%d, M4=%d\n", myPumpStates[0], myPumpStates[1], myPumpStates[2], myPumpStates[3]);

      // Aplica os estados aos motores (a ordem dos índices corresponde aos motores M1 a M4)
      // DRV8243_SetMotors(myPumpStates[0], myPumpStates[1], myPumpStates[2], myPumpStates[3]);

      if (!drvController.apply()) {
         Serial.println("[ERROR] falha ao aplicar estados dos motores");
      }

   } else if (String(topic) == "getdevices") {
      if (messageTemp == "get all") {
         JsonDocument jsonPayload;
         jsonPayload["host"] = WiFi.getHostname();
         jsonPayload["ip"] = WiFi.localIP().toString();
         jsonPayload["mac"] = WiFi.macAddress();
         jsonPayload["rssi"] = WiFi.RSSI();
         jsonPayload["RTCDatetime"] = getDateTime(Rtc.GetDateTime());
         jsonPayload["NTPDateTime"] = getFormattedDateTime();

         JsonObject ports = jsonPayload["ports"].to<JsonObject>();

         for (int indice = 0; indice < NUMBER_OUTPUTS; indice++) {
            JsonObject port = ports[myPumps[indice].getCode()].to<JsonObject>();
            port["gpio"] = myPumps[indice].getGpio();
            port["state"] = myPumpStates[indice] == 1;
            port["pulseDuration"] = myPumps[indice].getPulseDuration();

            // Adicionar os driveTimes ao JSON
            JsonArray driveTimesArray = port["driveTimes"].to<JsonArray>();
            for (const String& time : *(myPumps[indice].getDriveTimesPointer())) {
               driveTimesArray.add(time);
            }
         }

         String jsonStr;
         serializeJson(jsonPayload, jsonStr);
         client.publish("devices", jsonStr.c_str());
      }
   }
}

void restart() {
   yield();
   delay(1000);
   yield();
   ESP.restart();
}

void loadConfigurationCloud(const char* pumperCode, JsonDocument* jsonData) {
   JsonDocument response;
   JsonDocument body;  // Pode ser pequeno pois é o que será enviado
   JsonDocument doc;

   JsonObject object = doc.to<JsonObject>();
   object["pumperCode"] = pumperCode;

   body["dataSource"] = "Tomatoes";
   body["database"] = "tomatoes-database";
   body["collection"] = "boards";
   body["filter"] = object;

   // Serialize JSON document
   String json;
   serializeJson(body, json);

   WiFiClientSecure client;

   client.setCACert(root_ca);

   HTTPClient https;

   https.begin(client, serverName);
   https.addHeader("apiKey", apiKey);
   https.addHeader("Content-Type", "application/json");
   https.addHeader("Accept", "application/json");

   int httpResponseCode = https.POST(json);

   Serial.print("HTTP Response code: ");
   Serial.println(httpResponseCode);

   String payload = https.getString();

   // Disconnect
   https.end();

   DeserializationError error = deserializeJson(response, payload);

   if (error)
      Serial.println("Failed to read document, using default configuration");

   // serializeJsonPretty(response["document"], Serial);

   *jsonData = response["document"];
}

void updateConfiguration(JsonDocument inputDocument, std::set<String>* inputDriveTime, uint32_t* inputDuration) {
   JsonArray tempArray = inputDocument["driveTimes"].as<JsonArray>();

   inputDriveTime->clear();
   for (JsonVariant index : tempArray) {
      JsonObject object = index.as<JsonObject>();
      const char* tempTime = object["time"];
      bool tempState = object["state"];

      if (tempState)
         inputDriveTime->insert(tempTime);
   }

   *inputDuration = inputDocument["pulseDuration"].as<uint32_t>();
}

void saveDriveTimes(const std::set<String>& driveTimes, const String& filename) {
   File file = SPIFFS.open(filename, FILE_WRITE);

   if (!file) {
      Serial.println("Failed to open file for writing");
      return;
   }

   for (const String& time : driveTimes) {
      file.write((uint8_t*)time.c_str(), time.length());
      file.write('\n');  // Adiciona uma quebra de linha para separar os horários
   }

   file.close();
   Serial.println("Drive times saved successfully");
}

void loadDriveTimes(std::set<String>& driveTimes, const String& filename) {
   File file = SPIFFS.open(filename, FILE_READ);
   if (!file) {
      Serial.println("Failed to open file for reading");
      return;
   }

   driveTimes.clear();
   while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();  // Remove whitespace
      if (line.length() > 0) {
         driveTimes.insert(line);
      }
   }

   file.close();
   Serial.println("Drive times loaded successfully");
}

void initSPIFFS() {
   if (!SPIFFS.begin(true)) {
      Serial.println("An error has occurred while mounting SPIFFS");
   }
   Serial.println("SPIFFS mounted successfully");
}

void initWiFi() {
   WiFi.mode(WIFI_STA);

   // Configures static IP address
   // if (!WiFi.config(local_IP, gateway, subnet)) {
   //    Serial.println("STA Failed to configure");
   // }

   Serial.print("Connecting to WiFi ");
   Serial.print(ssid);
   Serial.print(" ...");
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
   }
   Serial.println();
   Serial.print("MAC Address:  ");
   Serial.println(WiFi.macAddress());
   Serial.print("Local IP:  ");
   Serial.println(WiFi.localIP());
   Serial.print("Hostname:  ");
   Serial.println(WiFi.getHostname());
   Serial.print("Gateway padrão da rede: ");
   Serial.println(WiFi.gatewayIP().toString());

   if (!MDNS.begin(WiFi.getHostname())) {
      Serial.println("Error setting up MDNS responder!");
      while (1) {
         delay(1000);
      }
   }
   Serial.println("mDNS responder started");
   Serial.print("mDNS Adress:  ");
   Serial.println(WiFi.getHostname());
}

void initNTP() {
   ntp.begin();
   ntp.forceUpdate();
}

void initMqtt() {
   Serial.print("Attempting MQTT connection on: ");
   Serial.print(mqtt_server);
   Serial.print(":");
   Serial.print(mqtt_port);
   Serial.print(" ...");
   while (!client.connected()) {
      Serial.print('.');

      if (client.connect((WiFi.getHostname()), "diego", "D1993rS*")) {  // Não confundir o id com o server
         Serial.println(" connected.");
         String topic = String(WiFi.getHostname()) + "/output";
         client.subscribe(topic.c_str());
         client.subscribe("getdevices");
      } else {
         Serial.println("Failed, reconnecting ... ");
         Serial.print("Client State: ");
         Serial.println(client.state());
      }

      delay(1000);
   }

   client.setBufferSize(2048);
}

void initds18b20Sensor() {
   sensors.begin();
   Serial.println("DS18B20 sensor initialized");
}

void initSHT35() {
   Wire.begin(1, 2);

   if (!gSht3x.begin()) {
      Serial.println("gSht3x.begin() failed\n");
   }

   Serial.println("SHT35 sensor initialized");
}

void initPhSensor() {
   AD7793_Init();
   CN0326_Init();
}

void initTdsSensor() {
}

void initDS3234() {
   SPI3.begin(PIN_SCK3, PIN_MISO3, PIN_MOSI3, PIN_CS_RTC);
   Rtc.Begin();

   RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

   if (!Rtc.IsDateTimeValid()) {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");

      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      Rtc.SetDateTime(compiled);
   }

   if (!Rtc.GetIsRunning()) {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
   }

   RtcDateTime now = Rtc.GetDateTime();
   if (now < compiled) {
      Serial.println("RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
   } else if (now > compiled) {
      Serial.println("RTC is newer than compile time. (this is expected)");
   } else if (now == compiled) {
      Serial.println("RTC is the same as compile time! (not expected but all is fine)");
   }

   // never assume the Rtc was last configured by you, so
   // just clear them to your needed state
   Rtc.Enable32kHzPin(false);
   Rtc.SetSquareWavePin(DS3234SquareWavePin_ModeNone);

   Serial.println("RTC initialized");
}

void initDRV8243Configuration() {
   if (!drvController.begin()) {
      Serial.println("[ERROR] falha ao inicializar DRV8243Controller");
   } else {
      Serial.println("DRV8243Controller iniciado com sucesso");
   }
}

void initRtos() {
   xWifiMutex = xSemaphoreCreateMutex();
   xSPIMutex = xSemaphoreCreateMutex();
   xPIDControllerMutex = xSemaphoreCreateMutex();

   xTaskCreatePinnedToCore(vTaskCheckWiFi, "taskCheckWiFi", configMINIMAL_STACK_SIZE + 1024, NULL, 2, &CheckWiFiTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskMqttReconnect, "taskMqttReconnect", configMINIMAL_STACK_SIZE + 2048, NULL, 2, &MqttReconnectTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskNTP, "taskNTP", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &NTPTaskHandle, PRO_CPU_NUM);

   for (int indice = 0; indice < ACTIVE_PUMPS; indice++) {
      xTaskCreatePinnedToCore(vTaskUpdate, "taskUpdate", configMINIMAL_STACK_SIZE + 8192, &myPumps[indice], 3, &UpdateTaskHandle, PRO_CPU_NUM);
      xTaskCreatePinnedToCore(vTaskTurnOnPump, "taskTurnOnPump", configMINIMAL_STACK_SIZE + 2048, &myPumps[indice], 3, &TurnOnPumpTaskHandle, APP_CPU_NUM);
   }

   xTaskCreatePinnedToCore(vTaskds18b20SensorRead, "taskds18b20SensorRead", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &ds18b20SensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskds18b20DataProcess, "taskds18b20DataProcess", configMINIMAL_STACK_SIZE + 4096, NULL, 2, &ds18b20DataProcessTaskHandle, APP_CPU_NUM);

   xTaskCreatePinnedToCore(vTaskSHT35SensorRead, "taskSHT35SensorRead", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &SHT35SensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskSHT35DataProcess, "taskSHT35DataProcess", configMINIMAL_STACK_SIZE + 4096, NULL, 2, &SHT35DataProcessTaskHandle, APP_CPU_NUM);

   // xTaskCreatePinnedToCore(vTaskPhSensorRead, "pH Meter Task", 4096, NULL, 1, &PhSensorReadTaskHandle, APP_CPU_NUM);
   // xTaskCreatePinnedToCore(vTaskPhDataProcess, "pH Data Process Task", 4096, NULL, 2, &PhDataProcessTaskHandle, APP_CPU_NUM);

   xTaskCreatePinnedToCore(vTaskTdsSensorRead, "TDS Meter Task", 4096, NULL, 1, &TdsSensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskTdsDataProcess, "TDS Data Process Task", 4096, NULL, 2, &TdsDataProcessTaskHandle, APP_CPU_NUM);

   Serial.println("RTOS initialized");
}

void initServer() {
   // Adicionando tratamento para requisições OPTIONS em /update para suportar CORS
   server.on("/update", HTTP_OPTIONS, [](AsyncWebServerRequest* request) {
      AsyncWebServerResponse* response = request->beginResponse(204);  // No Content
      response->addHeader("Access-Control-Allow-Origin", "*");
      response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
      response->addHeader("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept");
      request->send(response);
   });

   server.on(
       "/update", HTTP_POST, [](AsyncWebServerRequest* request) {                
         AsyncWebServerResponse *response = request->beginResponse((Update.hasError())?500:200, "text/plain", (Update.hasError())?"FAIL":"OK");
                response->addHeader("Connection", "close");
                response->addHeader("Access-Control-Allow-Origin", "*");
                request->send(response);
                restart(); },
       [](AsyncWebServerRequest* request, String filename, size_t index, uint8_t* data, size_t len, bool final) {
          if (!index) {
             int cmd = (filename == "firmware.bin") ? U_FLASH : U_SPIFFS;

             if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) {
                Update.printError(Serial);
                return request->send(400, "text/plain", "OTA could not begin");
             }
          }
          if (len) {
             if (Update.write(data, len) != len) {
                return request->send(400, "text/plain", "OTA could not begin");
             }
          }
          if (final) {
             if (!Update.end(true)) {
                Update.printError(Serial);
                return request->send(400, "text/plain", "Could not end OTA");
             }
          } else {
             return;
          }
       });
   server.serveStatic("/", SPIFFS, "/");

   server.begin();

   Serial.println("HTTP server started");
}

void setup() {
   uint32_t ret;

   // Initialize UART
   Serial.begin(115200);
   while (!Serial);

   // Initialize SPI
   SPI_Init();

   initSPIFFS();
   initWiFi();
   initNTP();

   initDRV8243Configuration();

   // Obter o gateway padrão da rede
   // IPAddress gateway = WiFi.gatewayIP();
   // mqtt_server = gateway.toString();

   client.setServer(mqtt_server.c_str(), mqtt_port);
   client.setCallback(callback);

   initDS3234();
   initSHT35();
   initTdsSensor();
   // initPhSensor();
   initds18b20Sensor();
   initMqtt();
   initServer();
   initRtos();

   Serial.printf("Total heap: %d\n", ESP.getHeapSize());
   Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
   Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
   Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
   Serial.printf("Total Flash: %d\n", ESP.getFlashChipSize());
   Serial.printf("Free Flash: %d\n", ESP.getFreeSketchSpace());
   Serial.printf("Sketch size: %d\n", ESP.getSketchSize());

   // Allow the hardware to sort itself out
   delay(1000);
}

void loop() {
   vTaskDelete(NULL);
}

void vTaskCheckWiFi(void* pvParameters) {
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            WiFi.reconnect();
         }
         xSemaphoreGive(xWifiMutex);
      }

      // Verificar uso da pilha
      // UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      // Serial.print("TaskCheckWiFi stack high water mark: ");
      // Serial.println(stackHighWaterMark);

      vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));
   }
}

void vTaskNTP(void* pvParameters) {
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (ntp.update()) {
            unsigned long epochTime = ntp.getEpochTime();
            RtcDateTime dateTime = convertEpochToDateTime(epochTime);
            Rtc.SetDateTime(dateTime);  // Atualiza o RTC com o tempo do NTP
            Serial.println("Time updated from NTP.");
         } else {
            Serial.println("Failed to update time from NTP.");
         }
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(NTP_DELAY));
   }
}

void vTaskMqttReconnect(void* parameter) {
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (!client.connected()) {
            Serial.println("Reconnecting to Mqtt...");
            client.disconnect();

            if (client.connect((WiFi.getHostname()), "diego", "D1993rS*")) {  // Não confundir o id com o server
               Serial.println(" connected.");
               String topic = String(WiFi.getHostname()) + "/output";
               client.subscribe(topic.c_str());
               client.subscribe("getdevices");
            } else {
               Serial.println("Failed, reconnecting ... ");
               Serial.print("Client State: ");
               Serial.println(client.state());
            }
         }
         client.loop();
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(CHECK_MQTT_DELAY));
   }
}

void vTaskUpdate(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (WiFi.status() == WL_CONNECTED) {
            loadConfigurationCloud(pump->pumperCode, pump->getJsonDataPointer());
            updateConfiguration(pump->getJsonData(), pump->getDriveTimesPointer(), pump->pulseDurationPointer);

            // Salvar os driveTimes atualizados apenas se houver conexão
            saveDriveTimes(*(pump->getDriveTimesPointer()), "/driveTimes.bin");
         } else {
            Serial.println("No internet connection, using stored drive times.");
         }
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(UPDATE_DELAY));
   }
}

void vTaskTurnOnPump(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      RtcDateTime now = Rtc.GetDateTime();        // Pegando o tempo do RTC
      String currentDateTime = getDateTime(now);  // Formatando o tempo obtido

      for (String driveTime : pump->getDriveTimes()) {
         if (currentDateTime == driveTime) {  // Comparando com os horários de acionamento
            pump->startPump();
         }
      }

      vTaskDelay(pdMS_TO_TICKS(TURN_ON_PUMP_DELAY));
   }
}

void vTaskds18b20SensorRead(void* pvParameters) {
   while (1) {
      sensors.requestTemperatures();
      vTaskDelay(pdMS_TO_TICKS(DS18B20_SENSOR_READ_DELAY));
   }
}

void vTaskds18b20DataProcess(void* pvParameters) {
   while (1) {
      float temperatureC = sensors.getTempCByIndex(0);
      // Serial.printf("Temperature: %.2f\n", temperatureC);

      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (WiFi.status() == WL_CONNECTED) {
            String ds18b20Topic = String("sensors/") + String(WiFi.getHostname()) + "/ds18b20/temperature";
            String ds18b20Payload = String(temperatureC);
            if (client.connected()) {
               client.publish(ds18b20Topic.c_str(), ds18b20Payload.c_str());
            }
         } else {
            Serial.println("No internet connection, skipping data processing.");
         }
         xSemaphoreGive(xWifiMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(DS18B20_DATA_PROCESS_DELAY));
   }
}

void vTaskSHT35SensorRead(void* pvParameters) {
   while (1) {
      if (!gSht3x.getTemperatureHumidity(sht35Data)) {
         Serial.println("Não foi possível ler os dados do sensor SHT35");
      }
      vTaskDelay(pdMS_TO_TICKS(SHT35_SENSOR_READ_DELAY));  // Atraso definido para a leitura do sensor
   }
}

void vTaskSHT35DataProcess(void* pvParameters) {
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (WiFi.status() == WL_CONNECTED) {
            String sht35TempTopic = String("sensors/") + String(WiFi.getHostname()) + "/sht35/temperature";
            String sht35HumTopic = String("sensors/") + String(WiFi.getHostname()) + "/sht35/humidity";
            String sht35TempPayload = String(sht35Data.Temperature);
            String sht35HumPayload = String(sht35Data.Humidity);

            if (client.connected()) {
               client.publish(sht35TempTopic.c_str(), sht35TempPayload.c_str());
               client.publish(sht35HumTopic.c_str(), sht35HumPayload.c_str());
            }
         } else {
            Serial.println("No internet connection, skipping data processing.");
         }
         xSemaphoreGive(xWifiMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(SHT35_DATA_PROCESS_DELAY));  // Atraso definido para o processamento dos dados
   }
}

void vTaskPhSensorRead(void* pvParameters) {
   float ph, temp, internalTemp, AVDD;
   while (1) {
      if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
         if (WiFi.status() == WL_CONNECTED) {
            if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {
               AVDD = CN0326_CalculateAVDD();
               internalTemp = CN0326_CalculateInternalTemp();
               temp = CN0326_CalculateTemp();
               ph = CN0326_CalculatePH();
               xSemaphoreGive(xSPIMutex);

               String phTopic = String("sensors/") + String(WiFi.getHostname()) + "/ph";
               String phPayload = String(ph);

               String tempTopic = String("sensors/") + String(WiFi.getHostname()) + "/temperature";
               String tempPayload = String(temp);

               String internalTempTopic = String("sensors/") + String(WiFi.getHostname()) + "/internalTemperature";
               String internalTempPayload = String(internalTemp);

               String avddTopic = String("sensors/") + String(WiFi.getHostname()) + "/avdd";
               String avddPayload = String(AVDD);

               if (client.connected()) {
                  client.publish(phTopic.c_str(), phPayload.c_str());
                  client.publish(tempTopic.c_str(), tempPayload.c_str());
                  client.publish(internalTempTopic.c_str(), internalTempPayload.c_str());
                  client.publish(avddTopic.c_str(), avddPayload.c_str());
               }
            } else {
               Serial.println("Failed to take SPI mutex");
            }
         } else {
            Serial.println("No internet connection, skipping data processing.");
         }
         xSemaphoreGive(xWifiMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(PH_SENSOR_READ_DELAY));
   }
}

void vTaskPhDataProcess(void* pvParameters) {
   while (1) {
      vTaskDelay(pdMS_TO_TICKS(PH_DATA_PROCESS_DELAY));
   }
}

void vTaskTdsSensorRead(void* pvParameters) {
   while (1) {
      if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {
         xSemaphoreGive(xSPIMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(TDS_SENSOR_READ_DELAY));
   }
}

void vTaskTdsDataProcess(void* pvParameters) {
   while (1) {
      vTaskDelay(pdMS_TO_TICKS(TDS_DATA_PROCESS_DELAY));
   }
}