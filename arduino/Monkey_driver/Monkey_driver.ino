// SYSTEM INCLUDE:
#include <vector>

// LIBRARY INCLUDE:
#include <Arduino.h>
#include <ArduinoLog.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#define ARDUINOJSON_ENABLE_STD_STRING 1
#include <ArduinoJson.h>
//#include <FreeRTOS.h>

//GLOBAL VARIABLES:
Adafruit_PWMServoDriver pca9685;
const int OSC_FREQ = 27000000;
const int SERVO_FREQ_HZ = 300;
const int TIME_STEP_MICROS=100000;

QueueHandle_t g_TaskQueue;
const int MAX_QUEUED_TASKS = 10;

enum SERVOS : uint8_t {
  SERVO_0 = 0,
  SERVO_1,
  SERVO_2,
  SERVO_3,
  SERVO_4,
  SERVO_5,
  SERVO_6,
  SERVO_7,

  NUM_SERVOS
};

struct GAIT_STRUCT {
  String gateName;
  int numServoSignals;
  std::vector<std::vector<int>> servoSignals;
};

const char* G_SSID = "Flatties";
const char* G_PASSWORD = "LekkerByDieSee1";
const uint16_t uPORT = 80;
WebServer server(uPORT);
StaticJsonDocument<100> jsonDocument;

void executeGait(const GAIT_STRUCT gait) {
  Log.noticeln("Executing new gait-type: %s - number of signals=%d", gait.gateName, gait.numServoSignals);
  for(int sig = 0; sig < gait.numServoSignals; sig++) {
    for(uint8_t servo = 0; servo < SERVOS::NUM_SERVOS; servo++) {
      pca9685.setPWM(servo, 0, gait.servoSignals[servo][sig]);
    }

    int end_time = esp_timer_get_time() + TIME_STEP_MICROS;
    for(;;) {
       if(esp_timer_get_time()>=end_time) {
        break;
       }
    }
  }
}

void configureLogging() {
  Serial.begin(115200);
  while(!Serial && !Serial.available()){}
  randomSeed(analogRead(0));
  Log.begin(LOG_LEVEL_NOTICE, &Serial);
  Log.noticeln("Logging configured");
}

void configurePca9685() {
  Log.noticeln("Configuring PCA9685");
  pca9685 = Adafruit_PWMServoDriver();
  pca9685.begin();
  pca9685.setOscillatorFrequency(OSC_FREQ);
  pca9685.setPWMFreq(SERVO_FREQ_HZ);
  Log.noticeln("PCA Configured");
}

void configureCorePipe() {
  Log.noticeln("Configuring Core Pipe");
  g_TaskQueue = xQueueCreate(MAX_QUEUED_TASKS, sizeof(GAIT_STRUCT*));
  if (g_TaskQueue == NULL)
  {
    Log.errorln("g_TaskQueue is NULL");
  }
  Log.noticeln("Core Pipe Configured");
}

// {"gait-name":"my-2d-with-step","num-signals":20,"pulse-lengths":[[0,1,2,3],[4,5,6,7]]}
void handleGait() {
  Log.noticeln("Handling new gait");
  jsonDocument.clear();
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  GAIT_STRUCT* gait = new GAIT_STRUCT();
  String nm = jsonDocument["gait-name"];
  gait->gateName = nm;
  gait->numServoSignals = jsonDocument["num-signals"];

  Log.noticeln("Retrieving gait pulse-lengths for %s", gait->gateName);
  // Parse the gaits
  for(int servo = 0; servo < static_cast<uint8_t>(SERVOS::NUM_SERVOS); servo++) {
    for(int signal = 0; signal < gait->numServoSignals; signal++) {
      gait->servoSignals[servo].push_back(jsonDocument["pulse-lengths"][servo][signal]);
    }
  }

  Log.noticeln("Adding gait to processing queue");
  xQueueSend(g_TaskQueue, gait, (TickType_t)0);
  server.send(200, "application/json", "{}");
}

/** https://microcontrollerslab.com/esp32-rest-api-web-server-get-post-postman/ */
void configureHttpServer() {
  Log.noticeln("Configuring HTTP Server");
  WiFi.begin(G_SSID, G_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Log.noticeln("Connecting...");
    delay(1000);
  }
  IPAddress my_ip = WiFi.localIP();
  Log.noticeln("IP Adres: %d.%d.%d.%d", my_ip[0],my_ip[1],my_ip[2],my_ip[3]);
  server.on("/put-gait", HTTP_POST, handleGait);    
  server.begin();  
}

void coreTaskServoDriver(void* pvParameters) {
  Log.noticeln("Servo driver running on core %d", xPortGetCoreID());
  while(true) {

    // READ OFF THE QUEUE
    for(int i = 0;i < MAX_QUEUED_TASKS; i++) {
        GAIT_STRUCT* pGait = nullptr;
        xQueueReceive(g_TaskQueue, &pGait, portMAX_DELAY);

        // EXECUTE
        executeGait(*pGait);
        delete pGait;
    }

    vTaskDelete( NULL );
  }
}



void configureCoreTasks(){
  static uint8_t parameters;
  xTaskCreatePinnedToCore(
                          coreTaskServoDriver, /* Funtion for the task to call */
                          "servo-driver", /* Name of the task */
                          1 * 1024, /* Stack size in words, 128KiB -> 32768 4 byte words */
                          &parameters, /* Task input parameters */
                          0, /* Priority of task */
                          NULL, /* Task Handle */
                          0 /* Core where this task should run */
    );
}

void setup() {
  // put your setup code here, to run once:
  configureLogging();
  configurePca9685();
  configureCorePipe();
  configureHttpServer();
  configureCoreTasks();
}

void loop() {
  server.handleClient();
}
