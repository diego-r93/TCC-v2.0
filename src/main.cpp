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
#include "CN0411.h"
#include "Communication.h"
#include "DRV8243Controller.h"
#include "NTPClient.h"
#include "PIDController.h"
#include "SHT3x.h"
#include "SPIFFS.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "hydraulicPumpController.h"
#include "mongoDbAtlas.h"
#include "mqttCredentials.h"
#include "wifiCredentials.h"

/*
Task                         Core  Prio                                    Descrição
--------------------------------------------------------------------------------------------------------------------------
vTaskMqttLoop                 0     5     Loop dedicado ao client.loop() (subscribe, reconnect e callback) a cada 10 ms
vTaskCheckWiFi                0     4     Verifica a conexão WiFi e tenta reconectar caso esteja deconectado
vTaskMqttReconnect            0     3     Verifica a conexão MQTT e tenta reconectar caso esteja deconectado
vTaskNTP                      0     1     Atualiza o horário com base no NTP
vTaskMqttHandler              1     3     Trata as mensagens recebidas do MQTT e executa as ações correspondentes
vTaskUpdate                   1     3     Atualiza as informações através de um POST no MongoDB Atlas
vTaskTurnOnPump               1     4     Liga a bomba quando chegar no seu horário de acionamento
vTaskds18b20SensorRead        1     1     Lê o sensor ds18b20
vTaskSHT35SensorRead          1     1     Lê o sensor SHT35
vTaskPhSensorRead             1     1     Lê o sensor de pH
vTaskECSensorRead             1     1     Lê o sensor de condutividade elétrica (EC)
vTaskPhDataProcess            1     2     Processa os dados do sensor de pH
vTaskECDataProcess            1     2     Processa os dados do sensor de condutividade elétrica (EC)
--------------------------------------------------------------------------------------------------------------------------
*/

// Tasks delays
#define UPDATE_DELAY 10000
#define UPDATE_TIMEOUT 3000
#define TURN_ON_PUMP_DELAY 100

#define DS18B20_SENSOR_READ_DELAY 5000
#define SHT35_SENSOR_READ_DELAY 1000
#define PH_SENSOR_READ_DELAY 5000
#define EC_SENSOR_READ_DELAY 5000

#define PH_DATA_PROCESS_DELAY 5000
#define EC_DATA_PROCESS_DELAY 5000

struct cn0411_init_params cn0411_init_params = {
    CH_GAIN_RES_20M,
    ADC_SINGLE_CONV,
    RTD_RES_1K,
    {GAIN_RES_20,
     GAIN_RES_200,
     GAIN_RES_2K,
     GAIN_RES_20K,
     GAIN_RES_200K,
     GAIN_RES_2M,
     GAIN_RES_20M},
    OFFSET_RES_INIT,
    DAC_OUT_DEFAULT_VAL,
    EXC_DEFAULT_VAL,
    VR20S_DEFAULT_VAL,
    VR200S_DEFAULT_VAL,
    RDRES_DEFAULT_VAL,
    EXC_DEFAULT_VAL,
    CELL_CONST_NORMAL,
    TEMP_DEFAULT_VAL,
    VPP_DEFAULT_VAL,
    VINP_DEFAULT_VAL,
    VINN_DEFAULT_VAL,
    COND_DEFAULT_VAL,
    COMP_COND_DEFAULT_VAL,
    TDS_DEFAULT_VAL,
    {TEMP_COEFF_NACL,
     TDS_NACL}};

struct cn0411_device cn0411_dev;

// Motors configuration
DRV8243Controller drvController;

HydraulicPumpController myPumps[] = {
    {"code01", drvController, 0, 60000},
    {"code02", drvController, 1, 60000},
    {"code03", drvController, 2, 60000},
    {"code04", drvController, 3, 60000},
};

const int numPumps = sizeof(myPumps) / sizeof(myPumps[0]);

// WiFi configuration
#define CHECK_WIFI_DELAY 500  // 500 ms
#define WIFI_TIMEOUT 5000     // 5 segundos

// NTP configuration
#define NTP_DELAY 1000                      // 1 segundo
#define NTP_TIMEOUT 3000                    // 3 segundos
#define NTP_UPDATE_INTERVAL 15 * 60 * 1000  // 15 minutos

WiFiUDP udp;
NTPClient ntp(udp, "a.st1.ntp.br", -3 * 3600, NTP_UPDATE_INTERVAL);

// MQTT configuration
#define MQTT_LOOP_DELAY 10           // 10 ms
#define CHECK_MQTT_DELAY 1000        // 1 segundo
#define MQTT_RECONNECT_TIMEOUT 5000  // 5 segundos

WiFiClient espClient;
PubSubClient client(espClient);

#define MQTT_QUEUE_LENGTH 10
#define MQTT_TOPIC_MAX_LEN 64
#define MQTT_PAYLOAD_MAX_LEN 256

// MQTT message structure
struct MqttMessage {
   char topic[MQTT_TOPIC_MAX_LEN];
   uint8_t payload[MQTT_PAYLOAD_MAX_LEN];
   size_t length;
};

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

QueueHandle_t xMqttQueue = NULL;

// FreeRTOS Task Handles
TaskHandle_t UpdateTaskHandle = NULL;
TaskHandle_t CheckWiFiTaskHandle = NULL;
TaskHandle_t MqttLoopTaskHandle = NULL;
TaskHandle_t MqttReconnectTaskHandle = NULL;
TaskHandle_t TaskMqttHandlerHandle = NULL;
TaskHandle_t NTPTaskHandle = NULL;
TaskHandle_t TurnOnPumpTaskHandle = NULL;

TaskHandle_t ds18b20SensorReadTaskHandle = NULL;
TaskHandle_t SHT35SensorReadTaskHandle = NULL;
TaskHandle_t PhSensorReadTaskHandle = NULL;
TaskHandle_t ECSensorReadTaskHandle = NULL;

TaskHandle_t PhDataProcessTaskHandle = NULL;
TaskHandle_t ECDataProcessTaskHandle = NULL;

// FreeRTOS Task Functions
void vTaskUpdate(void* pvParameters);
void vTaskCheckWiFi(void* pvParametes);
void vTaskMqttLoop(void* pvParametes);
void vTaskMqttReconnect(void* pvParametes);
void vTaskMqttHandler(void* pvParameters);
void vTaskNTP(void* pvParameters);
void vTaskTurnOnPump(void* pvParametes);

void vTaskds18b20SensorRead(void* pvParameters);
void vTaskSHT35SensorRead(void* pvParameters);
void vTaskPhSensorRead(void* pvParameters);
void vTaskECSensorRead(void* pvParameters);

void vTaskPhDataProcess(void* pvParameters);
void vTaskECDataProcess(void* pvParameters);

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

void mqttCallback(char* topic, byte* payload, unsigned int length) {
   MqttMessage msg;
   // copia tópico
   strncpy(msg.topic, topic, MQTT_TOPIC_MAX_LEN - 1);
   msg.topic[MQTT_TOPIC_MAX_LEN - 1] = '\0';
   // ajusta tamanho e copia payload
   msg.length = (length > MQTT_PAYLOAD_MAX_LEN) ? MQTT_PAYLOAD_MAX_LEN : length;
   memcpy(msg.payload, payload, msg.length);

   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   xQueueSendFromISR(xMqttQueue, &msg, &xHigherPriorityTaskWoken);
   // se a queue acordou task de maior prioridade, fuerza um yield
   if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
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
   https.setTimeout(5000);  // ⏳ Evita travar por muito tempo

   bool beginOK = https.begin(client, serverName);

   if (!beginOK) {
      ESP_LOGE("HTTPS", "❌ HTTPS begin() failed");
      return;
   }

   https.addHeader("apiKey", apiKey);
   https.addHeader("Content-Type", "application/json");
   https.addHeader("Accept", "application/json");

   int httpResponseCode = https.POST(json);

   if (httpResponseCode <= 0) {
      ESP_LOGE("HTTPS", "❌ POST failed. Error code: %d", httpResponseCode);
      https.end();  // Libera conexão
      return;
   } else {
      ESP_LOGI("HTTPS", "✅ POST successful. Response code: %d", httpResponseCode);
   }

   String payload = https.getString();
   https.end();

   DeserializationError error = deserializeJson(response, payload);

   if (error) {
      ESP_LOGE("HTTPS", "❌ Failed to parse JSON response.");
      return;
   }

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
      ESP_LOGE("SPIFFS", "❌ Failed to open file for writing");
      return;
   }

   for (const String& time : driveTimes) {
      file.write((uint8_t*)time.c_str(), time.length());
      file.write('\n');  // Adiciona uma quebra de linha para separar os horários
   }

   file.close();
   ESP_LOGI("SPIFFS", "✅ Drive times saved successfully");
}

void loadDriveTimes(std::set<String>& driveTimes, const String& filename) {
   File file = SPIFFS.open(filename, FILE_READ);
   if (!file) {
      ESP_LOGE("SPIFFS", "❌ Failed to open file for reading");
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
   ESP_LOGI("SPIFFS", "✅ Drive times loaded successfully");
}

void initSPIFFS() {
   if (!SPIFFS.begin(true)) {
      ESP_LOGE("SPIFFS", "An error has occurred while mounting SPIFFS");
   }
   ESP_LOGI("SPIFFS", "SPIFFS mounted successfully");
}

bool initWiFi(uint32_t timeout_ms = 15000, uint8_t max_retries = 3) {
   WiFi.mode(WIFI_STA);
   WiFi.persistent(false);  // opcional: não grava em NVS a cada begin
   WiFi.setAutoReconnect(true);

   uint8_t attempt = 0;
   while (attempt < max_retries) {
      attempt++;
      ESP_LOGI("WIFI", "Connecting to WiFi %s (attempt %u/%u)...", ssid, attempt, max_retries);
      WiFi.begin(ssid, password);

      uint32_t start = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
         ESP_LOGI("WIFI", ".");
         delay(500);
      }

      if (WiFi.status() == WL_CONNECTED) {
         ESP_LOGI("WIFI", "WiFi connected.");
         ESP_LOGI("WIFI", "MAC Address: %s", WiFi.macAddress().c_str());
         ESP_LOGI("WIFI", "Local IP: %s", WiFi.localIP().toString().c_str());
         ESP_LOGI("WIFI", "Hostname: %s", WiFi.getHostname());
         ESP_LOGI("WIFI", "Gateway: %s", WiFi.gatewayIP().toString().c_str());
         ESP_LOGI("WIFI", "Subnet Mask: %s", WiFi.subnetMask().toString().c_str());

         // mDNS (só inicia se conectado)
         if (!MDNS.begin(WiFi.getHostname())) {
            ESP_LOGE("WIFI", "Error setting up MDNS responder!");
            return false;
         }
         ESP_LOGI("WIFI", "mDNS started at %s.local", WiFi.getHostname());
         return true;
      }

      // chegou aqui sem conectar no timeout: limpa e tenta de novo
      ESP_LOGW("WIFI", "Connection timed out. Retrying...");
      WiFi.disconnect(true /*wifioff*/);
      delay(500);
   }

   ESP_LOGE("WIFI", "Failed to connect to WiFi after %u attempts", max_retries);
   return false;
}

void initNTP() {
   ntp.begin();
   ntp.forceUpdate();
}

bool initMqtt(uint8_t max_retries = 3) {
   client.setServer(mqtt_server, mqtt_port);
   client.setCallback(mqttCallback);
   client.setBufferSize(2048);
   // client.setSocketTimeout(3);  // Maximum blocking time in seconds
   // client.setKeepAlive(30);     // Interval in seconds for PINGREQ to broker

   if (WiFi.status() != WL_CONNECTED) {
      ESP_LOGE("MQTT", "WiFi not connected");
      return false;
   }

   const char* clientId = WiFi.getHostname();

   for (uint8_t attempt = 1; attempt <= max_retries; ++attempt) {
      ESP_LOGI("MQTT", "Attempting MQTT connection on %s:%d (attempt %u/%u)...", mqtt_server, mqtt_port, attempt, max_retries);

      client.disconnect();

      if (client.connect(clientId, mqtt_user, mqtt_password)) {
         ESP_LOGI("MQTT", "MQTT connected.");
         String outTopic = String(clientId) + "/output";
         client.subscribe(outTopic.c_str());
         client.subscribe("getdevices");
         ESP_LOGI("MQTT", "Subscribed to topics: %s and getdevices", outTopic.c_str());
         return true;
      }

      ESP_LOGE("MQTT", "Failed to connect to MQTT broker. State: %d", client.state());
      delay(500);
   }

   ESP_LOGE("MQTT", "Failed to connect to MQTT broker after %u attempts", max_retries);
   return false;
}

void initds18b20Sensor() {
   sensors.begin();
   ESP_LOGI("DS18B20", "Found %d devices.", sensors.getDeviceCount());
   ESP_LOGI("DS18B20", "DS18B20 sensor initialized");
}

void initSHT35() {
   Wire.begin(1, 2);

   if (!gSht3x.begin()) {
      ESP_LOGE("SHT35", "gSht3x.begin() failed");
   }

   ESP_LOGI("SHT35", "SHT35 sensor initialized");
}

void initPhSensor() {
   AD7793_Init();
   CN0326_Init();
}

void initECSensor() {
   uint32_t ret;

   ret = CN0411_init(&cn0411_dev, cn0411_init_params);

   if (ret == CN0411_FAILURE) {
      ESP_LOGE("ECSensor", "CN0411 Initialization error!");
   }

   ESP_LOGI("ECSensor", "CN0411 Initialization successful!");

   uint8_t pwm_duty = 50;  // Duty cycle de 50%

   // Define tensão DAC
   CN0411_DAC_set_value(&cn0411_dev, DAC_OUT_DEFAULT_VAL);
   if (CN0411_read_vdac(&cn0411_dev) == CN0411_FAILURE) {
      ESP_LOGE("ECSensor", "Falha ao ler a tensão do DAC.");
   } else {
      ESP_LOGI("ECSensor", "Tensão DAC: %.5f V", cn0411_dev.read_dac);
   }

   // Define resistor de ganho
   CN0411_set_gain_res(&cn0411_dev, CH_GAIN_RES_2K);

   // Define frequência PWM e duty cycle
   CN0411_pwm_freq(PWM_FREQ_94, pwm_duty);
}

void initDS3234() {
   SPI3.begin(PIN_SCK3, PIN_MISO3, PIN_MOSI3, PIN_CS_RTC);
   Rtc.Begin();

   RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

   if (!Rtc.IsDateTimeValid()) {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      ESP_LOGE("RTC", "RTC lost confidence in the DateTime!");

      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue
   }

   if (!Rtc.GetIsRunning()) {
      ESP_LOGI("RTC", "RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
   }

   // 2) Tenta NTP na primeira vez
   if (ntp.forceUpdate()) {
      unsigned long epoch = ntp.getEpochTime();
      RtcDateTime dt = convertEpochToDateTime(epoch);
      ESP_LOGI("RTC", "RTC synchronized by NTP at init: %s", getDateTime(dt).c_str());
      Rtc.SetDateTime(dt);
   } else {
      // se NTP falhou e RTC era inválido, usa compile time
      if (!Rtc.IsDateTimeValid()) {
         ESP_LOGI("RTC", "Setting RTC to compile time: %s", getDateTime(compiled).c_str());
         Rtc.SetDateTime(compiled);
      }
   }

   RtcDateTime now = Rtc.GetDateTime();
   if (now < compiled) {
      ESP_LOGI("RTC", "RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
   } else if (now > compiled) {
      ESP_LOGI("RTC", "RTC is newer than compile time. (this is expected)");
   } else if (now == compiled) {
      ESP_LOGI("RTC", "RTC is the same as compile time! (not expected but all is fine)");
   }

   // never assume the Rtc was last configured by you, so
   // just clear them to your needed state
   Rtc.Enable32kHzPin(false);
   Rtc.SetSquareWavePin(DS3234SquareWavePin_ModeNone);

   ESP_LOGI("RTC", "RTC initialized");
}

void initDRV8243Configuration() {
   if (!drvController.begin()) {
      ESP_LOGE("DRV8243", "Failed to initialize DRV8243Controller");
   } else {
      ESP_LOGI("DRV8243", "DRV8243Controller initialized successfully");
   }
}

void initRtos() {
   xTaskCreatePinnedToCore(vTaskCheckWiFi, "taskCheckWiFi", configMINIMAL_STACK_SIZE + 4096, NULL, 4, &CheckWiFiTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskMqttLoop, "taskMqttLoop", configMINIMAL_STACK_SIZE + 2048, NULL, 5, &MqttLoopTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskMqttReconnect, "taskMqttReconnect", configMINIMAL_STACK_SIZE + 2048, NULL, 3, &MqttReconnectTaskHandle, PRO_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskMqttHandler, "taskMqttHandler", configMINIMAL_STACK_SIZE + 4096, NULL, 3, &TaskMqttHandlerHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskNTP, "taskNTP", configMINIMAL_STACK_SIZE + 2048, NULL, 1, &NTPTaskHandle, PRO_CPU_NUM);

   for (int indice = 0; indice < numPumps; indice++) {
      xTaskCreatePinnedToCore(vTaskUpdate, "taskUpdate", configMINIMAL_STACK_SIZE + 8192, &myPumps[indice], 3, &UpdateTaskHandle, APP_CPU_NUM);
      xTaskCreatePinnedToCore(vTaskTurnOnPump, "taskTurnOnPump", configMINIMAL_STACK_SIZE + 2048, &myPumps[indice], 4, &TurnOnPumpTaskHandle, APP_CPU_NUM);
   }

   xTaskCreatePinnedToCore(vTaskds18b20SensorRead, "taskds18b20SensorRead", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &ds18b20SensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskSHT35SensorRead, "taskSHT35SensorRead", configMINIMAL_STACK_SIZE + 4096, NULL, 1, &SHT35SensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskPhSensorRead, "pH Meter Task", 4096, NULL, 1, &PhSensorReadTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskECSensorRead, "EC Meter Task", 4096, NULL, 1, &ECSensorReadTaskHandle, APP_CPU_NUM);

   xTaskCreatePinnedToCore(vTaskPhDataProcess, "pH Data Process Task", 4096, NULL, 2, &PhDataProcessTaskHandle, APP_CPU_NUM);
   xTaskCreatePinnedToCore(vTaskECDataProcess, "EC Data Process Task", 4096, NULL, 2, &ECDataProcessTaskHandle, APP_CPU_NUM);

   ESP_LOGI("RTOS", "All tasks created successfully");
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

   ESP_LOGI("HTTP", "HTTP server started");
}

void setup() {
   Serial.begin(115200);
   // while (!Serial);
   delay(1000);  // Give time for Serial to initialize

   SPI_Init();
   initSPIFFS();

   if (!initWiFi(15000, 3)) {
      ESP_LOGE("SETUP", "Failed to connect to WiFi. Starting without WiFi.");
   }
   initNTP();

   if (!initMqtt()) {
      ESP_LOGE("SETUP", "Failed to connect to MQTT broker. Starting without MQTT.");
   }

   xSPIMutex = xSemaphoreCreateMutex();
   configASSERT(xSPIMutex);

   xWifiMutex = xSemaphoreCreateMutex();
   configASSERT(xWifiMutex);

   xPIDControllerMutex = xSemaphoreCreateMutex();
   configASSERT(xPIDControllerMutex);

   xMqttQueue = xQueueCreate(MQTT_QUEUE_LENGTH, sizeof(MqttMessage));
   configASSERT(xMqttQueue);

   initDS3234();
   initSHT35();
   initds18b20Sensor();

   initDRV8243Configuration();
   initPhSensor();
   initECSensor();

   initServer();
   initRtos();

   ESP_LOGI("MEM", "Total heap: %d\n", ESP.getHeapSize());
   ESP_LOGI("MEM", "Free heap: %d\n", ESP.getFreeHeap());
   ESP_LOGI("MEM", "Total PSRAM: %d\n", ESP.getPsramSize());
   ESP_LOGI("MEM", "Free PSRAM: %d\n", ESP.getFreePsram());
   ESP_LOGI("MEM", "Total Flash: %d\n", ESP.getFlashChipSize());
   ESP_LOGI("MEM", "Free Flash: %d\n", ESP.getFreeSketchSpace());
   ESP_LOGI("MEM", "Sketch size: %d\n", ESP.getSketchSize());

   // Allow the hardware to sort itself out
   delay(1000);
}

void loop() {
   vTaskDelete(NULL);
}

void vTaskCheckWiFi(void* pvParameters) {
   const uint32_t timeout_ms = 15000;  // igual ao initWiFi
   const uint8_t max_retries = 3;      // igual ao initWiFi

   while (1) {
      if (WiFi.status() != WL_CONNECTED) {
         ESP_LOGE("WIFI", "⚠️ WiFi disconnected!");

         uint8_t attempt = 0;
         while (attempt < max_retries && WiFi.status() != WL_CONNECTED) {
            attempt++;
            ESP_LOGI("WIFI", "Connecting to WiFi %s (attempt %u/%u)...", ssid, attempt, max_retries);

            if (xSemaphoreTake(xWifiMutex, pdMS_TO_TICKS(WIFI_TIMEOUT))) {
               WiFi.begin(ssid, password);
               xSemaphoreGive(xWifiMutex);
            } else {
               ESP_LOGW("WIFI", "Timeout trying to acquire xWifiMutex for WiFi.begin(); retrying...");
               vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));
               continue;
            }

            // Espera conectar (SEM mutex)
            uint32_t start = millis();
            while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
               ESP_LOGI("WIFI", ".");
               vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));  // yield sem travar o RTOS
            }

            if (WiFi.status() != WL_CONNECTED) {
               ESP_LOGW("WIFI", "Connection timed out. Retrying...");

               // Desassociar/zerar sessão (curto e com mutex)
               if (xSemaphoreTake(xWifiMutex, pdMS_TO_TICKS(WIFI_TIMEOUT))) {
                  WiFi.disconnect(true /* wifioff */);  // igual ao seu initWiFi
                  xSemaphoreGive(xWifiMutex);
               } else {
                  ESP_LOGW("WIFI", "Timeout ao pegar xWifiMutex para WiFi.disconnect()");
               }

               vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));
            }
         }

         if (WiFi.status() == WL_CONNECTED) {
            ESP_LOGI("WIFI", "WiFi connected.");
            ESP_LOGI("WIFI", "MAC Address: %s", WiFi.macAddress().c_str());
            ESP_LOGI("WIFI", "Local IP: %s", WiFi.localIP().toString().c_str());
            ESP_LOGI("WIFI", "Hostname: %s", WiFi.getHostname());
            ESP_LOGI("WIFI", "Gateway: %s", WiFi.gatewayIP().toString().c_str());
            ESP_LOGI("WIFI", "Subnet Mask: %s", WiFi.subnetMask().toString().c_str());

            // garante mDNS quando reconectar (mesma política do initWiFi)
            if (!MDNS.begin(WiFi.getHostname())) {
               ESP_LOGE("WIFI", "Error setting up MDNS responder!");
            } else {
               ESP_LOGI("WIFI", "mDNS started at %s.local", WiFi.getHostname());
            }
         } else {
            ESP_LOGE("WIFI", "Failed to connect to WiFi after %u attempts. Keeping offline.", max_retries);
         }
      }

      vTaskDelay(pdMS_TO_TICKS(CHECK_WIFI_DELAY));
   }
}

void vTaskNTP(void* pvParameters) {
   while (1) {
      if ((WiFi.status() == WL_CONNECTED) && ntp.update()) {
         if (xSemaphoreTake(xWifiMutex, pdMS_TO_TICKS(NTP_TIMEOUT))) {
            ESP_LOGI("NTP", "🔐 WiFi mutex acquired for NTP update.");
            unsigned long epochTime = ntp.getEpochTime();
            RtcDateTime newTime = convertEpochToDateTime(epochTime);

            // debug: checa RTC antes da escrita
            if (!Rtc.IsDateTimeValid()) {
               ESP_LOGW("NTP", "⚠️ RTC inválido antes de NTP update");
            }
            if (!Rtc.GetIsRunning()) {
               ESP_LOGW("NTP", "⚠️ Oscilador RTC parado antes de update — reiniciando");
               Rtc.SetIsRunning(true);
            }

            // escreve no DS3234
            Rtc.SetDateTime(newTime);
            ESP_LOGI("NTP", "✅ Time updated from NTP: %s", getDateTime(newTime).c_str());
            xSemaphoreGive(xWifiMutex);
            ESP_LOGI("NTP", "🔓 WiFi mutex released after NTP update.");
         } else {
            ESP_LOGW("NTP", "⚠️ Timeout while trying to acquire xWifiMutex in vTaskNTP");
         }
      }

      vTaskDelay(pdMS_TO_TICKS(NTP_DELAY));
   }
}

// -----------------------------------------------------------------------------
// Task 1: Loop MQTT (alta prioridade, sem bloqueio de mutex, a cada 10 ms)
// -----------------------------------------------------------------------------
void vTaskMqttLoop(void* pvParameters) {
   while (true) {
      // Se não está conectado, não processa loop até reconectar
      if (!client.connected()) {
         vTaskDelay(pdMS_TO_TICKS(MQTT_LOOP_DELAY));
         continue;
      }

      // Processa incoming packets, keep-alive, invoca callback
      client.loop();

      // Pequeno delay para não rodar 100% CPU, mas bastante frequente
      vTaskDelay(pdMS_TO_TICKS(MQTT_LOOP_DELAY));
   }
}

// -----------------------------------------------------------------------------
// Task 2: Reconnect MQTT (prioridade um pouco menor, checa a cada 1 s)
// -----------------------------------------------------------------------------
void vTaskMqttReconnect(void* parameter) {
   const char* clientId = WiFi.getHostname();

   while (1) {
      if (client.connected()) {
         vTaskDelay(pdMS_TO_TICKS(CHECK_MQTT_DELAY));
         continue;
      } else {
         ESP_LOGW("MQTT", "⚠️ MQTT client not connected, attempting to reconnect...\n");

         if (WiFi.status() == WL_CONNECTED) {
            ESP_LOGI("MQTT", "✅ WiFi is connected, trying to reconnect MQTT client...\n");

            if (xSemaphoreTake(xWifiMutex, pdMS_TO_TICKS(MQTT_RECONNECT_TIMEOUT))) {
               ESP_LOGI("MQTT", "🔐 WiFi mutex acquired for MQTT Reconnect.\n");
               client.disconnect();         // Clean any stale session
               xSemaphoreGive(xWifiMutex);  // Immediately release the mutex
               ESP_LOGI("MQTT", "🔓 WiFi mutex released after disconnect.\n");

               if (client.connect(clientId, mqtt_user, mqtt_password)) {
                  ESP_LOGI("MQTT", "✅ Reconnected as '%s'. Subscribing...\n", clientId);
                  String outTopic = String(clientId) + "/output";
                  client.subscribe(outTopic.c_str());
                  client.subscribe("getdevices");
                  ESP_LOGI("MQTT", "✅ Subscriptions done.\n");
               } else {
                  ESP_LOGW("MQTT", "⚠️ Reconnect failed (state=%d). Will retry.\n", client.state());
               }
            } else {
               ESP_LOGW("MQTT", "⚠️ Timeout while trying to acquire xWifiMutex in vTaskMqttReconnect\n");
            }
         } else {
            ESP_LOGW("MQTT", "⚠️ WiFi is not connected, skipping MQTT reconnect.\n");
         }
      }

      vTaskDelay(pdMS_TO_TICKS(CHECK_MQTT_DELAY));
   }
}

// -----------------------------------------------------------------------------
// Task de tratamento de mensagens
// -----------------------------------------------------------------------------
void vTaskMqttHandler(void* pvParameters) {
   MqttMessage msg;

   while (true) {
      // bloqueia até receber uma mensagem
      if (xQueueReceive(xMqttQueue, &msg, portMAX_DELAY) != pdTRUE) {
         continue;
      }

      // 1) Imprime chegada da mensagem
      ESP_LOGI("MQTT", "Message arrived on topic: %s. Message: ", msg.topic);
      for (size_t i = 0; i < msg.length; ++i) {
         ESP_LOGI("MQTT", "%c", (char)msg.payload[i]);
      }
      ESP_LOGI("MQTT", "\n");

      // 2) Constrói String para facilitar o JSON
      String topicStr = String(msg.topic);
      String messageTemp;
      for (size_t i = 0; i < msg.length; ++i) {
         messageTemp += (char)msg.payload[i];
      }

      // 3) Trata o tópico "<hostname>/output"
      if (topicStr == String(WiFi.getHostname()) + "/output") {
         ESP_LOGI("MQTT", "Received command to change motor states");

         JsonDocument doc;
         DeserializationError err = deserializeJson(doc, messageTemp);
         if (!err) {
            for (int indice = 0; indice < NUMBER_OUTPUTS; ++indice) {
               String key = String(indice);

               if (doc[key].is<bool>()) {
                  bool state = doc[key];

                  ESP_LOGI("MQTT", "Changing motor %s to %s\n", key.c_str(), state ? "on" : "off");

                  if (state)
                     myPumps[indice].startPump();
                  else
                     myPumps[indice].stopPump();
               }
            }
            ESP_LOGI("MQTT", "Motor states: M1=%d, M2=%d, M3=%d, M4=%d\n",
                     int(myPumps[0].getPumpState()),
                     int(myPumps[1].getPumpState()),
                     int(myPumps[2].getPumpState()),
                     int(myPumps[3].getPumpState()));
         }
      }
      // 4) Trata o tópico "getdevices"
      else if (topicStr == "getdevices") {
         if (messageTemp == "get all") {
            // Monta um JSON de status
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
               port["gpio"] = myPumps[indice].getMotorIndex();
               port["state"] = myPumps[indice].getPumpState();
               port["pulseDuration"] = myPumps[indice].getPulseDuration();

               // Adicionar os driveTimes ao JSON
               JsonArray driveTimesArray = port["driveTimes"].to<JsonArray>();
               for (const String& time : *(myPumps[indice].getDriveTimesPointer())) {
                  driveTimesArray.add(time);
               }
            }

            String jsonTopic = String("devices/") + String(WiFi.getHostname());
            String jsonStr;
            serializeJson(jsonPayload, jsonStr);
            // client.publish(jsonTopic.c_str(), jsonStr.c_str());
            client.publish("devices", jsonStr.c_str());
         }
      }
      // (demais tópicos, se houverem…)
   }
}

void vTaskUpdate(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      if (WiFi.status() == WL_CONNECTED) {
         if (xSemaphoreTake(xWifiMutex, pdMS_TO_TICKS(UPDATE_TIMEOUT))) {
            ESP_LOGI("WIFI", "🔐 WiFi mutex acquired for update.");
            loadConfigurationCloud(pump->getCode(), pump->getJsonDataPointer());
            updateConfiguration(pump->getJsonData(), pump->getDriveTimesPointer(), pump->pulseDurationPointer);

            xSemaphoreGive(xWifiMutex);
            ESP_LOGI("WIFI", "🔓 WiFi mutex released after update.");

            saveDriveTimes(*(pump->getDriveTimesPointer()), "/driveTimes.bin");
         } else {
            ESP_LOGE("WIFI", "⚠️ Timeout while trying to acquire xWifiMutex in vTaskUpdate.");
         }
      } else {
         ESP_LOGE("WIFI", "⚠️ No internet connection, using stored drive times.");
      }

      vTaskDelay(pdMS_TO_TICKS(UPDATE_DELAY));
   }
}

void vTaskTurnOnPump(void* pvParameters) {
   HydraulicPumpController* pump = (HydraulicPumpController*)pvParameters;

   while (1) {
      RtcDateTime now = Rtc.GetDateTime();

      char timeBuffer[9];  // "HH:MM:SS" + null terminator
      snprintf(timeBuffer, sizeof(timeBuffer), "%02u:%02u:%02u", now.Hour(), now.Minute(), now.Second());
      String formattedTime(timeBuffer);

      for (String driveTime : pump->getDriveTimes()) {
         if (formattedTime == driveTime && !pump->getPumpState()) {
            ESP_LOGI("PUMP", "🔔 Time to turn on pump %s at %s\n", pump->getCode(), driveTime.c_str());
            pump->startPump();
         }
      }
      vTaskDelay(pdMS_TO_TICKS(TURN_ON_PUMP_DELAY));
   }
}

void vTaskds18b20SensorRead(void* pvParameters) {
   float temperatureC;

   while (1) {
      sensors.requestTemperatures();

      temperatureC = sensors.getTempCByIndex(0);

      if (temperatureC == DEVICE_DISCONNECTED_C) {
         ESP_LOGE("DS18B20", "Failed to read temperature from DS18B20 sensor, retrying...");
         vTaskDelay(pdMS_TO_TICKS(DS18B20_SENSOR_READ_DELAY));
         continue;
      }

      if (WiFi.status() == WL_CONNECTED && client.connected()) {
         String ds18b20Topic = String("sensors/") + String(WiFi.getHostname()) + "/ds18b20/temperature";
         String ds18b20Payload = String(temperatureC);

         if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
            if (client.connected()) {
               client.publish(ds18b20Topic.c_str(), ds18b20Payload.c_str());
               xSemaphoreGive(xWifiMutex);
            } else {
               ESP_LOGE("MQTT", "MQTT client not connected, cannot publish DS18B20 data.");
            }
         }
      } else {
         ESP_LOGE("WIFI", "No internet connection, skipping data processing.");
      }

      vTaskDelay(pdMS_TO_TICKS(DS18B20_SENSOR_READ_DELAY));
   }
}

void vTaskSHT35SensorRead(void* pvParameters) {
   while (1) {
      if (!gSht3x.getTemperatureHumidity(sht35Data)) {
         ESP_LOGE("SHT35", "Failed to read SHT35 data, retrying...");
         vTaskDelay(pdMS_TO_TICKS(SHT35_SENSOR_READ_DELAY));
         continue;
      }

      if (WiFi.status() == WL_CONNECTED && client.connected()) {
         String sht35TempTopic = String("sensors/") + String(WiFi.getHostname()) + "/sht35/temperature";
         String sht35HumTopic = String("sensors/") + String(WiFi.getHostname()) + "/sht35/humidity";
         String sht35TempPayload = String(sht35Data.Temperature);
         String sht35HumPayload = String(sht35Data.Humidity);

         if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
            if (client.connected()) {
               client.publish(sht35TempTopic.c_str(), sht35TempPayload.c_str());
               client.publish(sht35HumTopic.c_str(), sht35HumPayload.c_str());
               xSemaphoreGive(xWifiMutex);
            } else {
               ESP_LOGE("MQTT", "MQTT client not connected, cannot publish SHT35 data.");
            }
         }
      } else {
         ESP_LOGE("WIFI", "No internet connection, skipping data processing.");
      }

      vTaskDelay(pdMS_TO_TICKS(SHT35_SENSOR_READ_DELAY));
   }
}

void vTaskPhSensorRead(void* pvParameters) {
   float ph, temp, internalTemp, AVDD;
   while (1) {
      AVDD = CN0326_CalculateAVDD();
      internalTemp = CN0326_CalculateInternalTemp();
      ph = CN0326_CalculatePH();
      temp = CN0326_CalculateTemp();

      if (WiFi.status() == WL_CONNECTED) {
         String phTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0326/ph";
         String phPayload = String(ph);

         String tempTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0326/temperature";
         String tempPayload = String(temp);

         String internalTempTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0326/internalTemperature";
         String internalTempPayload = String(internalTemp);

         String avddTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0326/avdd";
         String avddPayload = String(AVDD);

         if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
            if (client.connected()) {
               client.publish(phTopic.c_str(), phPayload.c_str());
               client.publish(tempTopic.c_str(), tempPayload.c_str());
               client.publish(internalTempTopic.c_str(), internalTempPayload.c_str());
               client.publish(avddTopic.c_str(), avddPayload.c_str());
               xSemaphoreGive(xWifiMutex);
            } else {
               ESP_LOGE("MQTT", "MQTT client not connected, cannot publish pH data.");
            }
         }
      } else {
         ESP_LOGE("WIFI", "No internet connection, skipping data processing.");
      }

      vTaskDelay(pdMS_TO_TICKS(PH_SENSOR_READ_DELAY));
   }
}

void vTaskPhDataProcess(void* pvParameters) {
   while (1) {
      vTaskDelay(pdMS_TO_TICKS(PH_DATA_PROCESS_DELAY));
   }
}

void vTaskECSensorRead(void* pvParameters) {
   while (true) {
      // PWM principal com 50% duty
      digitalWrite(PWM_MAIN_PIN, HIGH);
      digitalWrite(PWM_POS_PIN, HIGH);
      digitalWrite(PWM_NEG_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(5));  // metade do período (aprox. 5.3ms)

      digitalWrite(PWM_MAIN_PIN, LOW);
      digitalWrite(PWM_POS_PIN, LOW);
      digitalWrite(PWM_NEG_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(5));  // metade do período (aprox. 5.3ms)

      if (xSemaphoreTake(xSPIMutex, portMAX_DELAY)) {
         CN0411_read_temp(&cn0411_dev);
         CN0411_compute_cond(&cn0411_dev);
         CN0411_compensate_cond(&cn0411_dev);
         CN0411_compute_tds(&cn0411_dev);
         xSemaphoreGive(xSPIMutex);
      }

      String tempTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/temperature";
      String tempPayload = String(cn0411_dev.temp);

      String vppTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/vpp";
      String vppPayload = String(cn0411_dev.vpp);

      String rdResTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/rdres";
      String rdResPayload = String(cn0411_dev.rdres);

      String condTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/cond";
      String condPayload = String(1000000 * cn0411_dev.cond);

      String compensatedCondTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/compensatedCond";
      String compensatedCondPayload = String(1000000 * cn0411_dev.comp_cond);

      String tdsTopic = String("sensors/") + String(WiFi.getHostname()) + "/cn0411/tds";
      String tdsPayload = String(1000000 * cn0411_dev.tds);

      if (WiFi.status() == WL_CONNECTED) {
         if (xSemaphoreTake(xWifiMutex, portMAX_DELAY)) {
            if (client.connected()) {
               client.publish(tempTopic.c_str(), tempPayload.c_str());
               client.publish(vppTopic.c_str(), vppPayload.c_str());
               client.publish(rdResTopic.c_str(), rdResPayload.c_str());
               client.publish(condTopic.c_str(), condPayload.c_str());
               client.publish(compensatedCondTopic.c_str(), compensatedCondPayload.c_str());
               client.publish(tdsTopic.c_str(), tdsPayload.c_str());
               xSemaphoreGive(xWifiMutex);
            } else {
               ESP_LOGE("MQTT", "MQTT client not connected, cannot publish EC data.");
            }
         }
      } else {
         ESP_LOGE("WIFI", "No internet connection, skipping data processing.");
      }

      vTaskDelay(pdMS_TO_TICKS(EC_SENSOR_READ_DELAY));
   }
}

void vTaskECDataProcess(void* pvParameters) {
   while (1) {
      vTaskDelay(pdMS_TO_TICKS(EC_DATA_PROCESS_DELAY));
   }
}