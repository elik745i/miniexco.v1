/*
  MINIEXCO Project 2025
  ESP32S3 V.690 Changes:
  -Integrated receival of telemetry data from DFROBOT BNO055 module mounted on esp32-cam module over UDP
  Library tweaks to get it working: - README.txt

  ESP32-CAM Dual Server: Streaming (port 81) + Control (port 80)
  ESP32-CAM V.047 Changes:
  - integrated DFROBOT BNO055 module, do not forget to change sensor_t to adafruit_sensor_t in libs: 
  C:\Users\...\Documents\Arduino\libraries\Adafruit_Unified_Sensor
  and
  C:\Users\...\Documents\Arduino\libraries\Adafruit_BNO055
  both in cpp and h files!
*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>  //change in lib  src/ESPAsyncWebServer.h stroke 1102 to tcp_state state() {
#include <AsyncTCP.h> // by dvarrel
#include <WiFi.h>
#include <Preferences.h> // NEW!
#include <ESPmDNS.h>  // <- mDNS library for ESP32
#include <ElegantOTA.h> // change in lib src/ElegantOTA.h stroke 27 to #define ELEGANTOTA_USE_ASYNC_WEBSERVER 1 also tweak cpp and h files like this:
#include <iostream>
#include <sstream>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include "driver/ledc.h"

#include "HTML.h"
#include "SettingsModal.h"
#include "WiFiPages.h"
#include "favicon_16x16.h"
#include "commonStyle.h"
#include "commonScript.h"
#include "modalScript.h"
#include "controlScript.h"
#include "telemetryScript.h"
#include "drawScript.h"

//String ssid = "OTC_GUEST";
//String password = "??@@343Leg1596@";

#define MAIN_FW_VERSION "v0.698"
#define S3_ID "MINIEXCO_S3_002"
#define S3_CAM_ID "MINIEXCO_002"

// PWM channel mapping summary
#define CH_RIGHT_MOTOR_IN1 0
#define CH_RIGHT_MOTOR_IN2 1
#define CH_LEFT_MOTOR_IN1  2
#define CH_LEFT_MOTOR_IN2  3
#define CH_ARM_MOTOR_IN1   4
#define CH_ARM_MOTOR_IN2   5
#define CH_BUCKET_SERVO    6
#define CH_AUX_SERVO       7

#define PWM_RES_BITS 14
#define PWM_PERIOD_US 20000

unsigned long lastFpsReceived = millis();
unsigned long lastImuReceived = millis();
unsigned long lastIpResend = 0;


bool leftSignalActive = false;
bool rightSignalActive = false;
bool beaconOn = false;
bool emergencyOn = false;
bool shouldReboot = false;
bool otaValid = false;

bool displayUDPserial = false;  //Dislay UDP data in serial monitor

String currentCamFirmware = "Updating...";  // Default; will be updated if received over UDP

bool camAcknowledged = false;

int wifiRetryCount = 5;  // Default fallback value

// Non-blocking auto-detach tracking
unsigned long auxDetachTime = 0;
bool auxAttached = false;

// Non-blocking auto-detach tracking
unsigned long bucketDetachTime = 0;
bool bucketAttached = false;

unsigned long lastHandshakeResend = 0;
const unsigned long handshakeResendInterval = 5000; // every 5 seconds

struct MotorState {
  int currentSpeed;
  int targetSpeed;
  int dirPin;
  int otherPin;
  int dirChannel;
  int otherChannel;
  unsigned long lastUpdateTime;
  bool dirForward;
};

MotorState motorStates[3]; // RIGHT, LEFT, ARM
const int rampStep = 15;   // speed increment
const int rampDelay = 5;   // ms between each ramp step


WiFiUDP udp;
const char* broadcastIP = "255.255.255.255";
const unsigned int udpPort = 4210;
char udpBuffer[128];

bool wifiConnecting = false;
unsigned long wifiConnectStartTime = 0;
String wifiSSID = "";
String wifiPassword = "";

int lastBucketValue = 140;  // Set to your safe init value
int lastAuxValue = 150;


// defines
#define BLevel 9
#define CSense 10
#define bucketServoPin  11
#define auxServoPin 12
#define lightPin1 1
#define lightPin2 2
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0

#define CAM_RESET_PIN 3  // now used to reset ESP32-CAM
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define ARM_MOTOR 2

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

//Use a 10k + 4.7k divider — safe and very common. Let me know if yours is different.
unsigned long lastTelemetrySend = 0;
const float R1 = 10000.0; // 10k Ohm (top resistor)
const float R2 = 4700.0;  // 4.7k Ohm (bottom resistor)
const float MAX_BATTERY_VOLTAGE = 8.4; // 2S Li-ion full charge
const float MIN_BATTERY_VOLTAGE = 6.0; // 2S safe cutoff


//Camera Detection variable
bool esp32camOnline = false;
String cameraIP = "";  // Discovered IP of the ESP32-CAM


bool darkMode = false;
bool horizontalScreen = false;
bool holdBucket = false;
bool holdAux = false;



// global constants
//extern const char* htmlWiFiSetupPage PROGMEM;
//extern const char* htmlHomePage PROGMEM;
const char* ap_ssid = "MiniExco_Setup";

// global variables
bool removeArmMomentum = false;
bool light = false;
Preferences preferences; // NEW
Preferences camPrefs;
Preferences keymapPrefs;  // NEW for keyboard mappings
Preferences imuPrefs;

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins = {
  {4, 13},  // RIGHT_MOTOR Pins
  {5, 6},  // LEFT_MOTOR Pins
  {8, 7}   // ARM_MOTOR pins
};


AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

// 👇 Helper: Stop server + WebSocket cleanly before Wi-Fi switch
void stopWebServerAndWS() {
    wsCarInput.closeAll();  // Close all websocket clients
    server.end();           // Fully stop the HTTP server
    Serial.println("✅ WebServer + WebSocket stopped.");
}

// 👇 Helper: Restart server + WebSocket after Wi-Fi connects
void startWebServerAndWS() {
    // Re-attach websocket handler after restart
    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);
    server.begin();  // Restart HTTP server
    Serial.println("✅ WebServer restarted.");
    // Restart ElegantOTA binding too
    ElegantOTA.begin(&server);
    Serial.println("✅ WebSocket + OTA re-attached.");
}


void rotateMotor(int motorNumber, int motorDirection) {
  MotorState& m = motorStates[motorNumber];
  m.lastUpdateTime = millis();  // reset timing for ramp

  if (motorDirection == FORWARD) {
    m.targetSpeed = 255;
    m.currentSpeed = 0;  // reset ramp
    m.dirForward = true;

  } else if (motorDirection == BACKWARD) {
    m.targetSpeed = 255;
    m.currentSpeed = 0;
    m.dirForward = false;

  } else {
    if (removeArmMomentum && motorNumber == ARM_MOTOR) {
      ledcWrite(m.dirChannel, 255);
      ledcWrite(m.otherChannel, 0);
      delay(10);
    }

    ledcWrite(m.dirChannel, 0);
    ledcWrite(m.otherChannel, 0);
    m.currentSpeed = 0;
    m.targetSpeed = 0;
  }
}

void rotateMotorS(int motorNumber, int motorDirection, int speed = 255) {
  MotorState &m = motorStates[motorNumber];
  m.lastUpdateTime = millis();
  m.targetSpeed = constrain(speed, 0, 255);
  m.currentSpeed = m.targetSpeed;

  if (motorDirection == FORWARD) {
    m.dirForward = true;
    digitalWrite(m.otherPin, LOW);
    ledcWrite(m.dirChannel, m.currentSpeed);
  } else if (motorDirection == BACKWARD) {
    m.dirForward = false;
    digitalWrite(m.dirPin, LOW);
    ledcWrite(m.otherChannel, m.currentSpeed);
  } else {
    ledcWrite(m.dirChannel, 0);
    ledcWrite(m.otherChannel, 0);
    m.targetSpeed = 0;
  }
  if (motorDirection == STOP || speed == 0) {
    ledcWrite(m.dirChannel, 0);
    ledcWrite(m.otherChannel, 0);
    m.targetSpeed = 0;
    return;
  }
}


void moveCarS(char command, int speed = 255) {
  switch (toupper(command)) {
    case 'F':
      rotateMotorS(RIGHT_MOTOR, FORWARD, speed);
      rotateMotorS(LEFT_MOTOR, FORWARD, speed);
      break;
    case 'B':
      rotateMotorS(RIGHT_MOTOR, BACKWARD, speed);
      rotateMotorS(LEFT_MOTOR, BACKWARD, speed);
      break;
    case 'L':
      rotateMotorS(RIGHT_MOTOR, FORWARD, speed);
      rotateMotorS(LEFT_MOTOR, BACKWARD, speed);
      break;
    case 'R':
      rotateMotorS(RIGHT_MOTOR, BACKWARD, speed);
      rotateMotorS(LEFT_MOTOR, FORWARD, speed);
      break;
    case 'A':
      rotateMotorS(ARM_MOTOR, FORWARD, speed);
      break;
    case 'D':
      rotateMotorS(ARM_MOTOR, BACKWARD, speed);
      break;
    case 'S':
    default:
      rotateMotorS(RIGHT_MOTOR, STOP);
      rotateMotorS(LEFT_MOTOR, STOP);
      rotateMotorS(ARM_MOTOR, STOP);
      break;
  }
}

void moveCar(int inputValue)
{
  Serial.printf("Got value as %d\n", inputValue);
  if (!horizontalScreen) {
    switch (inputValue) {
      case UP:
      case DOWN:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        break;
      case LEFT:
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        break;
      case RIGHT:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);
        break;
      case STOP:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD);
        break;
      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD);
        removeArmMomentum = true;
        break;
      default:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
    }
  } else {
    switch (inputValue) {
      case UP:
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        break;
      case DOWN:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);
        break;
      case LEFT:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        break;
      case RIGHT:
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);
        break;
      case STOP:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD);
        break;
      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD);
        removeArmMomentum = true;
        break;
      default:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
    }
  }
}


void writeServo(ledc_channel_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse_us = map(angle, 0, 180, 500, 2500);
  uint32_t duty = ((uint64_t)pulse_us << PWM_RES_BITS) / PWM_PERIOD_US;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void bucketTilt(int bucketServoValue) {
  if (!bucketAttached) {
    // Use low-speed timer explicitly
    ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_14_BIT,
      .timer_num = LEDC_TIMER_2,  // 🔄 CHANGED from 0 → 2
      .freq_hz = 50,
      .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
      .gpio_num = bucketServoPin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)CH_BUCKET_SERVO,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_2,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&ch);
    bucketAttached = true;
    Serial.println("🪣 Bucket PWM configured (low-speed)");
  }

  writeServo((ledc_channel_t)CH_BUCKET_SERVO, bucketServoValue);

  lastBucketValue = bucketServoValue;
  preferences.putInt("bucketAngle", lastBucketValue);
  if (!holdBucket) bucketDetachTime = millis() + 300;
}


void auxControl(int auxServoValue) {
  if (!auxAttached) {
    ledc_timer_config_t timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_14_BIT,
      .timer_num = LEDC_TIMER_3,  // 🔄 CHANGED from 1 → 3
      .freq_hz = 50,
      .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
      .gpio_num = auxServoPin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)CH_AUX_SERVO,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_3,  // use a separate timer from bucket
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&ch);
    auxAttached = true;
    Serial.println("🔧 AUX PWM configured (low-speed)");
  }

  writeServo((ledc_channel_t)CH_AUX_SERVO, auxServoValue);

  lastAuxValue = auxServoValue;
  preferences.putInt("auxAngle", lastAuxValue);
  if (!holdAux) auxDetachTime = millis() + 300;
}



void lightControl() {
  if (!light) {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
    light = true;
    Serial.println("Lights ON");
  } else {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
    light = false;
    Serial.println("Lights OFF");
  }
}

void handleRoot(AsyncWebServerRequest *request)
{
  String page = String(FPSTR(htmlHomePage));  // Load main HTML from PROGMEM
  if (cameraIP == "") cameraIP = "0.0.0.0";
  page.replace("%CAMERAIP%", cameraIP);

  // Replace version placeholders
  page.replace("%MAINFW%", MAIN_FW_VERSION);
  page.replace("%CAMFW%", currentCamFirmware);

  // ➕ Append Camera Modal
  page.replace("%MODAL%", FPSTR(htmlSettingsModal));  // ✅ Replace modal placeholder
  page.replace("%COMMON_STYLE%", FPSTR(commonStyle));
  page.replace("%COMMON_SCRIPT%", FPSTR(commonScript));

  request->send(200, "text/html", page);
}


// NEW - Wi-Fi Setup HTML page

void handleWiFiSetup(AsyncWebServerRequest *request) {
  String page = htmlWiFiSetupPage; // Load template
  String options = "";

  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    options += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + "dBm)</option>";
  }

  page.replace("%OPTIONS%", options);
  request->send(200, "text/html", page);
}


// NEW - Save Wi-Fi credentials
//extern const char* htmlWiFiSuccessPage PROGMEM; // add at the top if not added

void handleSaveWiFi(AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
        String ssid = request->getParam("ssid", true)->value();
        String password = request->getParam("password", true)->value();
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        preferences.putString(("wifi_" + ssid).c_str(), password);  // ✅ SAVE PER-SSID TOO

        String existingList = preferences.getString("networks", "");
        if (existingList.indexOf(ssid) == -1) {
            existingList += (existingList.length() > 0 ? "," : "") + ssid;
            preferences.putString("networks", existingList);
        }

        request->send_P(200, "text/html", htmlWiFiSuccessPage);
        
        delay(3000);
        ESP.restart();  // <-- ONLY needed here because you need to **reboot** after saving new credentials
    } else {
        request->send(400, "text/html", "<html><body><h1>Missing SSID or Password</h1></body></html>");
    }
}


void handleListWiFi(AsyncWebServerRequest *request) {
  int n = WiFi.scanComplete();

  if (n == -2) {
    // No scan yet, start it
    WiFi.scanNetworks(true);
    request->send(202, "application/json", "[]");  // Accept request, but scanning now
    return;
  }

  if (n == -1) {
    // Scan ongoing
    request->send(202, "application/json", "[]");  // Accept request, still scanning
    return;
  }

  // Scan complete, return results
  String json = "[";
  for (int i = 0; i < n; ++i) {
    if (i) json += ",";
    json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
  }
  json += "]";

  WiFi.scanDelete();  // Free memory
  request->send(200, "application/json", json);
}


void handleNotFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "File Not Found");
}

void controlMotorByDirection(const std::string& dir, int speed) {
  speed = constrain(speed, 0, 255);

  if (dir == "Forward") {
    rotateMotor(RIGHT_MOTOR, FORWARD);
    rotateMotor(LEFT_MOTOR, FORWARD);
    motorStates[RIGHT_MOTOR].targetSpeed = speed;
    motorStates[LEFT_MOTOR].targetSpeed = speed;

  } else if (dir == "Backward") {
    rotateMotor(RIGHT_MOTOR, BACKWARD);
    rotateMotor(LEFT_MOTOR, BACKWARD);
    motorStates[RIGHT_MOTOR].targetSpeed = speed;
    motorStates[LEFT_MOTOR].targetSpeed = speed;

  } else if (dir == "Left") {
    rotateMotor(RIGHT_MOTOR, FORWARD);
    rotateMotor(LEFT_MOTOR, BACKWARD);
    motorStates[RIGHT_MOTOR].targetSpeed = speed;
    motorStates[LEFT_MOTOR].targetSpeed = speed;

  } else if (dir == "Right") {
    rotateMotor(RIGHT_MOTOR, BACKWARD);
    rotateMotor(LEFT_MOTOR, FORWARD);
    motorStates[RIGHT_MOTOR].targetSpeed = speed;
    motorStates[LEFT_MOTOR].targetSpeed = speed;

  } else if (dir == "ArmUp") {
    rotateMotor(ARM_MOTOR, FORWARD);
    motorStates[ARM_MOTOR].targetSpeed = speed;

  } else if (dir == "ArmDown") {
    rotateMotor(ARM_MOTOR, BACKWARD);
    motorStates[ARM_MOTOR].targetSpeed = speed;
  }

  motorStates[LEFT_MOTOR].lastUpdateTime = millis();
  motorStates[RIGHT_MOTOR].lastUpdateTime = millis();
}



void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      client->text("HoldBucket," + String(holdBucket ? 1 : 0));
      client->text("HoldAux," + String(holdAux ? 1 : 0));
      client->text("Switch," + String(horizontalScreen ? 1 : 0)); // Optional: for HorizontalScreen checkbox too
      client->text("DarkMode," + String(darkMode ? 1 : 0));
      client->text("SliderInit,Forward,0");
      client->text("SliderInit,Backward,0");
      client->text("SliderInit,Left,0");
      client->text("SliderInit,Right,0");
      client->text("SliderInit,ArmUp,0");
      client->text("SliderInit,ArmDown,0");

      client->text("AUX," + String(lastAuxValue));
      client->text("Bucket," + String(lastBucketValue));
      client->text("Beacon," + String(beaconOn ? 1 : 0));
      client->text("Emergency," + String(emergencyOn ? 1 : 0));



      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      moveCar(STOP);
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        std::string myData = "";
        myData.assign((char *)data, len);
        std::istringstream ss(myData);
        std::string key, value1, value2;
        std::getline(ss, key, ',');
        std::getline(ss, value1, ',');
        std::getline(ss, value2, ',');

        // NEW: Turn Signal Handling
        if (key == "Slider") {
          String side = String(value1.c_str());
          int val = atoi(value2.c_str());

          if (side == "Left") {
            if (val > 5 && !leftSignalActive) {
              udp.beginPacket(cameraIP.c_str(), udpPort);
              udp.print("FLASH_LEFT");
              udp.endPacket();
              leftSignalActive = true;
              wsCarInput.textAll("TURN_LEFT,1");
            } else if (val <= 5 && leftSignalActive) {
              leftSignalActive = false;
              wsCarInput.textAll("TURN_LEFT,0");
            }
          }

          if (side == "Right") {
            if (val > 5 && !rightSignalActive) {
              udp.beginPacket(cameraIP.c_str(), udpPort);
              udp.print("FLASH_RIGHT");
              udp.endPacket();
              rightSignalActive = true;
              wsCarInput.textAll("TURN_RIGHT,1");
            } else if (val <= 5 && rightSignalActive) {
              rightSignalActive = false;
              wsCarInput.textAll("TURN_RIGHT,0");
            }
          }
        }

        // NEW: Beacon Toggle Button
        if (key == "Beacon") {
          beaconOn = !beaconOn;
          udp.beginPacket(cameraIP.c_str(), udpPort);
          udp.print(beaconOn ? "BEACON" : "BEACON_OFF");
          udp.endPacket();
          Serial.println(beaconOn ? "🟡 Beacon ON" : "⚫ Beacon OFF");
        }

        // NEW: Emergency Toggle Button
        if (key == "Emergency") {
          emergencyOn = !emergencyOn;
          udp.beginPacket(cameraIP.c_str(), udpPort);
          udp.print(emergencyOn ? "EMERGENCY" : "EMERGENCY_OFF");
          udp.endPacket();
          Serial.println(emergencyOn ? "🟡 Emergency ON" : "⚪ Emergency OFF");
        }

        if (key == "Motor") {
          controlMotorByDirection(value1, atoi(value2.c_str()));
        }

        Serial.printf("Key [%s] Value1[%s] Value2[%s]\n", key.c_str(), value1.c_str(), value2.c_str());
        int valueInt = atoi(value1.c_str());

        if (key == "MoveCar") moveCar(valueInt);
        else if (key == "AUX") auxControl(valueInt);
        else if (key == "Bucket") bucketTilt(valueInt);
        else if (key == "Light") {
          lightControl();

          // 🔁 Forward to ESP32-CAM via UDP
          if (cameraIP != "") {
            udp.beginPacket(cameraIP.c_str(), udpPort);
            udp.print(light ? "LEDON" : "LEDOFF");
            udp.endPacket();
          }

          // ✅ Fix: Send back to frontend over WebSocket
          //client->text("Light," + String(light ? 1 : 0));
          //all clients to update LED state
          wsCarInput.textAll("Light," + String(light ? 1 : 0));
        }

        else if (key == "Switch") {
          horizontalScreen = !horizontalScreen;
          preferences.putBool("Switch", horizontalScreen);
        }
        else if (key == "HoldBucket") {
          holdBucket = (valueInt != 0);
          preferences.putBool("HoldBucket", holdBucket);
        }
        else if (key == "HoldAux") {
          holdAux = (valueInt != 0);
          preferences.putBool("HoldAux", holdAux);
        }
        else if (key == "DarkMode") {
          darkMode = (valueInt != 0);
          preferences.putBool("darkMode", darkMode);
        }

      }
      break;
    default:
      break;
  }
}

void setUpPinModes() {
  const int pwmFreq = 1000;
  const int pwmRes = 8;

  // RIGHT motor
  pinMode(4, OUTPUT); pinMode(13, OUTPUT);
  ledcSetup(CH_RIGHT_MOTOR_IN1, pwmFreq, pwmRes); // HIGH_SPEED mode by default
  ledcSetup(CH_RIGHT_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(4, CH_RIGHT_MOTOR_IN1);
  ledcAttachPin(13, CH_RIGHT_MOTOR_IN2);
  motorStates[RIGHT_MOTOR] = {0, 0, 4, 13, CH_RIGHT_MOTOR_IN1, CH_RIGHT_MOTOR_IN2, millis(), true};

  // LEFT motor
  pinMode(5, OUTPUT); pinMode(6, OUTPUT);
  ledcSetup(CH_LEFT_MOTOR_IN1, pwmFreq, pwmRes);
  ledcSetup(CH_LEFT_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(5, CH_LEFT_MOTOR_IN1);
  ledcAttachPin(6, CH_LEFT_MOTOR_IN2);
  motorStates[LEFT_MOTOR] = {0, 0, 5, 6, CH_LEFT_MOTOR_IN1, CH_LEFT_MOTOR_IN2, millis(), true};

  // ARM motor
  pinMode(7, OUTPUT); pinMode(8, OUTPUT);
  ledcSetup(CH_ARM_MOTOR_IN1, pwmFreq, pwmRes);
  ledcSetup(CH_ARM_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(7, CH_ARM_MOTOR_IN1);
  ledcAttachPin(8, CH_ARM_MOTOR_IN2);
  motorStates[ARM_MOTOR] = {0, 0, 7, 8, CH_ARM_MOTOR_IN1, CH_ARM_MOTOR_IN2, millis(), true};

  // Force stop on all channels
  for (int i = 0; i < 3; i++) {
    ledcWrite(motorStates[i].dirChannel, 0);
    ledcWrite(motorStates[i].otherChannel, 0);
    digitalWrite(motorStates[i].dirPin, LOW);
    digitalWrite(motorStates[i].otherPin, LOW);
  }
}


bool connectToWiFiWithRetries(const String& ssid, const String& password, int retries) {
  WiFi.begin(ssid.c_str(), password.c_str());

  for (int attempt = 1; attempt <= retries; attempt++) {
    Serial.printf("🔁 Attempt %d to connect to WiFi SSID: %s\n", attempt, ssid.c_str());
    unsigned long startAttemptTime = millis();

    while (millis() - startAttemptTime < 2000) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✅ Connected to WiFi!");
        return true;
      }
      delay(200);
    }

    WiFi.disconnect();  // Reset for retry
    delay(100);
    WiFi.begin(ssid.c_str(), password.c_str());
  }

  Serial.println("❌ Failed to connect after retries.");
  return false;
}


bool startNonBlockingWiFiConnection()
  {
    Serial.println("🔍 Scanning for nearby Wi-Fi networks...");
    int n = WiFi.scanNetworks();
    bool found = false;

    // 1️⃣ Check if the last saved SSID is present
    if (wifiSSID != "") {
        for (int i = 0; i < n; i++) {
            String nearbySSID = WiFi.SSID(i);
            if (nearbySSID == wifiSSID) {
                Serial.println("✅ Last saved Wi-Fi is nearby: " + wifiSSID);
                found = true;
                break;
            }
        }
    }

    // 2️⃣ If not found, look through all saved networks
    if (!found) {
        Serial.println("⚠️ Last saved Wi-Fi not found. Checking other saved networks...");

        String savedList = preferences.getString("networks", "");
        int lastIndex = 0, nextIndex;
        while ((nextIndex = savedList.indexOf(',', lastIndex)) != -1) {
            String ssid = savedList.substring(lastIndex, nextIndex);
            ssid.trim();
            lastIndex = nextIndex + 1;

            for (int i = 0; i < n; i++) {
                if (ssid == WiFi.SSID(i)) {
                    Serial.println("✅ Found another saved SSID nearby: " + ssid);
                    wifiSSID = ssid;
                    wifiPassword = preferences.getString(("wifi_" + ssid).c_str(), "");
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        // Handle the last item in the list (if any)
        if (!found && lastIndex < savedList.length()) {
            String ssid = savedList.substring(lastIndex);
            ssid.trim();
            for (int i = 0; i < n; i++) {
                if (ssid == WiFi.SSID(i)) {
                    Serial.println("✅ Found another saved SSID nearby: " + ssid);
                    wifiSSID = ssid;
                    wifiPassword = preferences.getString(("wifi_" + ssid).c_str(), "");
                    found = true;
                    break;
                }
            }
        }
    }

    // 3️⃣ If nothing is found, abort and stay in AP mode
    if (!found || wifiSSID == "") {
        Serial.println("❌ No saved networks are nearby. Staying in AP mode.");
        return false;
    }

      unsigned long waitStart = millis();
      //bool camAcknowledged = false;

      while (millis() - waitStart < 10000) {  // Wait max 10 sec
          int packetSize = udp.parsePacket();
          if (packetSize) {
              int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
              if (len > 0) {
                  udpBuffer[len] = 0;
                  String incomingMsg = String(udpBuffer);
                  //Serial.println("📨 Received UDP: " + incomingMsg);
                  if (incomingMsg.startsWith("CAMID:" + String(S3_CAM_ID)) && incomingMsg.indexOf("OK:YES") != -1) {
                      camAcknowledged = true;
                      Serial.println("✅ CAM acknowledgment confirmed! Stopping resend loop.");
                      break;
                  }
              }
          }
      }

      if (!camAcknowledged) {
          Serial.println("⚠️ CAM did not acknowledge during AP, switching to STA and retrying listen...");

          WiFi.softAPdisconnect(true);
          delay(500);  // 🔧 Give time for stack to settle
          WiFi.mode(WIFI_OFF);  // 🔄 Reset WiFi stack
          delay(100);           // Short pause
          WiFi.mode(WIFI_STA);  // Set STA mode cleanly

          if (!connectToWiFiWithRetries(wifiSSID, wifiPassword, wifiRetryCount)) {
              Serial.println("❌ All retries failed. Staying in AP mode.");
              WiFi.mode(WIFI_AP);
              WiFi.softAP(ap_ssid);
              udp.begin(udpPort);
              return false;
          }



          unsigned long postConnectStart = millis();
          while (millis() - postConnectStart < 5000) {  // 5-second post-connect listen
              int packetSize = udp.parsePacket();
              if (packetSize) {
                  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
                  if (len > 0) {
                      udpBuffer[len] = 0;
                      String incomingMsg = String(udpBuffer);
                      //Serial.println("📨 Post-STA Received UDP: " + incomingMsg);
                      if (incomingMsg.startsWith("CAMID:" + String(S3_CAM_ID)) && incomingMsg.indexOf("OK:YES") != -1) {
                          camAcknowledged = true;
                          Serial.println("✅ CAM acknowledgment confirmed! Stopping resend loop.");
                          break;
                      }
                  }
              }
          }

          if (!camAcknowledged) {
              Serial.println("❌ CAM still did not acknowledge, staying in AP mode.");
              WiFi.disconnect();
              WiFi.mode(WIFI_AP);
              WiFi.softAP(ap_ssid);
              udp.begin(udpPort);  // Make sure binding after AP comes up
              return false;
          }
      }


      Serial.println("✅ CAM acknowledged, switching S3 to STA mode...");

      // FULL cleanup before switching modes
      WiFi.disconnect(true, true);  // Disconnect + Erase flash config
      WiFi.softAPdisconnect(true);
      delay(200);
      WiFi.mode(WIFI_OFF);
      delay(500);  // Let stack fully reset

      Serial.println("✅ Wi-Fi stack fully reset, switching to STA mode...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());



      wifiConnecting = true;
      wifiConnectStartTime = millis();
      return true;
  }



void resetEsp32Cam() {
  Serial.println("🔄 Resetting ESP32-CAM via GPIO3...");
  pinMode(CAM_RESET_PIN, OUTPUT);
  digitalWrite(CAM_RESET_PIN, LOW);   // Hold in reset
  delay(200);                          // Hold time (adjust if needed)
  digitalWrite(CAM_RESET_PIN, HIGH);  // Release reset
  delay(200);                          // Wait after release
}

void broadcastInitialStates() {
  if (!esp32camOnline || cameraIP == "") return;

  if (beaconOn) {
    udp.beginPacket(cameraIP.c_str(), udpPort);
    udp.print("BEACON");
    udp.endPacket();
    Serial.println("📤 Beacon state resent: ON");
  }

  if (emergencyOn) {
    udp.beginPacket(cameraIP.c_str(), udpPort);
    udp.print("EMERGENCY");
    udp.endPacket();
    Serial.println("📤 Emergency state resent: ON");
  }

  // Hold modes
  String holdMsg = String("HOLD,BUCKET,") + (holdBucket ? "1" : "0") + ",AUX," + (holdAux ? "1" : "0");
  udp.beginPacket(cameraIP.c_str(), udpPort);
  udp.print(holdMsg);
  udp.endPacket();
  Serial.println("📤 Hold mode states sent");

  // Orientation
  udp.beginPacket(cameraIP.c_str(), udpPort);
  udp.print(horizontalScreen ? "ORIENTATION,H" : "ORIENTATION,V");
  udp.endPacket();
  Serial.println("📤 Orientation state sent");
}

void setup()
{
  Serial.begin(115200);
  setUpPinModes();
  Serial.println("Ready. Send F/B/L/R/A/D <speed> or S to stop.");

  pinMode(lightPin1, OUTPUT);
  digitalWrite(lightPin1, LOW);  // or HIGH depending on logic

  // DO NOT redefine motorStates if already done in setUpPinModes()
  for (int i = 0; i < 3; i++) {
    ledcWrite(motorStates[i].dirChannel, 0);
    ledcWrite(motorStates[i].otherChannel, 0);
  }


  analogReadResolution(12); // 12-bit ADC (0-4095)
  analogSetAttenuation(ADC_11db); // Full range: 0-3.6V (good for voltage dividers)

  preferences.begin("wifiCreds", false);
  keymapPrefs.begin("keymap", false);  // read/write

  darkMode = preferences.getBool("darkMode", false);
  horizontalScreen = preferences.getBool("Switch", false);
  holdBucket = preferences.getBool("HoldBucket", false);
  holdAux = preferences.getBool("HoldAux", false);


  wifiRetryCount = preferences.getInt("wifiRetryCount", 5);  // Default to 5 retries if not set


  wifiSSID = preferences.getString("ssid", "");
  wifiPassword = preferences.getString("password", "");

  if (wifiSSID != "") {
      Serial.println("🟡 Found saved Wi-Fi, preparing handshake with CAM...");
      // Start AP temporarily just for handshake
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid);
      udp.begin(udpPort);  // Make sure binding after AP comes up
      Serial.print("Started temporary AP for CAM handshake: ");
      Serial.println(ap_ssid);
      // ✅ Now reset CAM AFTER AP is up and UDP is bound
      resetEsp32Cam();
      pinMode(CAM_RESET_PIN, INPUT);  // Float the line to avoid driving CAM reset pin

      // Proceed to handshake and push credentials
      startNonBlockingWiFiConnection();
  } else {
      Serial.println("🟠 No saved Wi-Fi, staying in AP mode for setup.");
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid);
      udp.begin(udpPort);  // Make sure binding after AP comes up
      Serial.print("Started AP: ");
      Serial.println(ap_ssid);
  }


  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());




  // WebServer Routes
  server.on("/", HTTP_GET, handleRoot);
  // Serve favicon.ico to browser
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "image/x-icon", favicon_16x16, sizeof(favicon_16x16));
  });
  server.on("/setup", HTTP_GET, handleWiFiSetup);
  server.on("/savewifi", HTTP_POST, handleSaveWiFi);
  server.on("/listwifi", HTTP_GET, handleListWiFi);
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });
  server.on("/get_camera_ip", HTTP_GET, [](AsyncWebServerRequest *request) {
    String savedIP = camPrefs.getString("lastIP", "");
    request->send(200, "application/json", "{\"ip\":\"" + savedIP + "\"}");
  });
  server.on("/modalScript.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "application/javascript", modalScript);
  });
  server.on("/controlScript.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "application/javascript", controlScript);
  });

  server.on("/telemetryScript.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "application/javascript", telemetryScript);
  });

  server.on("/calibrate_imu", HTTP_POST, [](AsyncWebServerRequest *request) {
      Preferences imuPrefs;
      imuPrefs.begin("imu", true);
      bool stored = imuPrefs.getBool("stored", false);
      int sys = imuPrefs.getInt("sys", -1);
      int gyro = imuPrefs.getInt("gyro", -1);
      int accel = imuPrefs.getInt("accel", -1);
      int mag = imuPrefs.getInt("mag", -1);
      imuPrefs.end();

      if (stored && sys >= 0 && gyro >= 0 && accel >= 0 && mag >= 0) {
          // ✅ Already calibrated — return stored data to frontend
          String json = "{\"status\":\"stored\",\"sys\":" + String(sys) +
                        ",\"gyro\":" + String(gyro) +
                        ",\"accel\":" + String(accel) +
                        ",\"mag\":" + String(mag) + "}";
          request->send(200, "application/json", json);
          Serial.println("📤 Returned stored calibration data to frontend.");
      } else {
          // ❗ Not stored — request from CAM
          String requestCalib = "S3ID:" + String(S3_ID) + ",REQ_CALIB:YES";
          udp.beginPacket(cameraIP.c_str(), udpPort);
          udp.write((const uint8_t*)requestCalib.c_str(), requestCalib.length());
          udp.endPacket();
          request->send(200, "application/json", "{\"status\":\"requested\"}");
          Serial.println("📤 Requested calibration from CAM.");
      }
  });

  server.on("/drawScript.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "application/javascript", drawScript);
  });


  server.on("/get_keymap", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<512> doc;
    doc["forward"] = keymapPrefs.getString("forward", "w");
    doc["backward"] = keymapPrefs.getString("backward", "s");
    doc["left"] = keymapPrefs.getString("left", "a");
    doc["right"] = keymapPrefs.getString("right", "d");
    doc["stop"] = keymapPrefs.getString("stop", " ");

    doc["armUp"] = keymapPrefs.getString("armUp", "u");
    doc["armDown"] = keymapPrefs.getString("armDown", "j");

    doc["bucketUp"] = keymapPrefs.getString("bucketUp", "u");
    doc["bucketDown"] = keymapPrefs.getString("bucketDown", "j");
    doc["auxUp"] = keymapPrefs.getString("auxUp", "i");
    doc["auxDown"] = keymapPrefs.getString("auxDown", "k");
    doc["led"] = keymapPrefs.getString("led", "l");
    doc["beacon"] = keymapPrefs.getString("beacon", "b");
    doc["emergency"] = keymapPrefs.getString("emergency", "e");

    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  server.on("/set_keymap", HTTP_POST, [](AsyncWebServerRequest *request){},
  NULL,
  [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, data, len);

    if (error) {
      request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    for (JsonPair kv : doc.as<JsonObject>()) {
      keymapPrefs.putString(kv.key().c_str(), kv.value().as<String>());
    }

    request->send(200, "application/json", "{\"status\":\"ok\"}");
  });



  server.on("/ota/upload", HTTP_POST, [](AsyncWebServerRequest *request){},
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {

      if (index == 0) {
        // Basic firmware header check
        if (len < 4 || data[0] != 0xE9) {
          request->send(400, "text/plain", "Invalid firmware format. Aborted.");
          otaValid = false;
          return;
        }

        otaValid = true;

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          request->send(500, "text/plain", "OTA begin failed.");
          return;
        }
      }

      if (otaValid) {
        if (Update.write(data, len) != len) {
          request->send(500, "text/plain", "OTA write failed.");
          return;
        }

        if (final) {
          if (Update.end(true)) {
            request->send(200, "text/plain", "✅ Update complete. Rebooting...");
            shouldReboot = true;
          } else {
            request->send(500, "text/plain", "OTA end failed.");
          }
        }
      }
    });



  server.on("/list_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
      String savedList = preferences.getString("networks", "");
      String json = "[";
      int lastIndex = 0, nextIndex;
      while ((nextIndex = savedList.indexOf(',', lastIndex)) != -1) {
          String ssid = savedList.substring(lastIndex, nextIndex);
          ssid.trim();
          String wifiKey = "wifi_" + ssid;
          String pass = preferences.getString(wifiKey.c_str(), "");
          int retry = preferences.getInt(("retry_" + ssid).c_str(), 5);  // ← NEW
          if (json.length() > 1) json += ",";
          json += "{\"ssid\":\"" + ssid + "\", \"password\":\"" + pass + "\", \"retry\":" + retry + "}";
          lastIndex = nextIndex + 1;
      }
      if (lastIndex < savedList.length()) {
          String ssid = savedList.substring(lastIndex);
          ssid.trim();
          String wifiKey = "wifi_" + ssid;
          String pass = preferences.getString(wifiKey.c_str(), "");
          int retry = preferences.getInt(("retry_" + ssid).c_str(), 5);  // ← NEW
          if (json.length() > 1) json += ",";
          json += "{\"ssid\":\"" + ssid + "\", \"password\":\"" + pass + "\", \"retry\":" + retry + "}";
      }
      json += "]";
      request->send(200, "application/json", json);
  });

  server.on("/update_wifi_password", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("ssid") && request->hasParam("password")) {
          String ssid = request->getParam("ssid")->value();
          String password = request->getParam("password")->value();
          String wifiKey = "wifi_" + ssid;
          preferences.putString(wifiKey.c_str(), password);
          request->send(200, "text/plain", "Password updated");
      } else {
          request->send(400, "text/plain", "Missing parameters");
      }
  });

  server.on("/connect_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("ssid")) {
          String ssid = request->getParam("ssid")->value();
          String wifiKey = "wifi_" + ssid;
          String pass = preferences.getString(wifiKey.c_str(), "");
          if (pass != "") {
              preferences.putString("ssid", ssid);
              preferences.putString("password", pass);

              // ADD UDP SEND HERE:
              String msg = "S3ID:" + String(S3_ID) + ",SSID:" + ssid + ",PASS:" + pass;
              udp.beginPacket(broadcastIP, udpPort);
              udp.write((const uint8_t*)msg.c_str(), msg.length());
              udp.endPacket();
              Serial.println("📤 Sent UDP: " + msg);

              Serial.println("🔄 Switching to saved Wi-Fi: " + ssid);

              // ✅ New: stop server + WS before switching network
              stopWebServerAndWS();
              delay(300);  // Give it time to fully stop

              connectToWiFiWithRetries(ssid, pass, wifiRetryCount);

              wifiSSID = ssid;
              wifiPassword = pass;
              wifiConnecting = true;
              wifiConnectStartTime = millis();

              request->send(200, "text/plain", "Switching Wi-Fi, please wait...");

          } else {
              request->send(404, "text/plain", "SSID not found");
          }
      } else {
          request->send(400, "text/plain", "Missing SSID parameter");
      }
  });

  server.on("/resync_cam", HTTP_GET, [](AsyncWebServerRequest *request) {
      String readyMsg = "S3ID:" + String(S3_ID) + ",READY:YES";
      udp.beginPacket(broadcastIP, udpPort);
      udp.write((const uint8_t*)readyMsg.c_str(), readyMsg.length());
      udp.endPacket();
      Serial.println("📣 Manual resync: Sent READY message to CAM");

      request->send(200, "text/plain", "Resync message sent to camera.");
  });

  server.on("/update_retry_count", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (request->hasParam("ssid") && request->hasParam("count")) {
          String ssid = request->getParam("ssid")->value();
          int count = request->getParam("count")->value().toInt();
          if (count >= 1 && count <= 10) {
              preferences.putInt(("retry_" + ssid).c_str(), count);
              request->send(200, "text/plain", "Retry count updated for " + ssid);
          } else {
              request->send(400, "text/plain", "Retry count must be between 1 and 10");
          }
      } else {
          request->send(400, "text/plain", "Missing ssid or count parameter");
      }
  });



  server.onNotFound(handleNotFound);

  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);
  server.begin();
  Serial.println("HTTP server started");

  //OTA Update
  ElegantOTA.begin(&server);  // Works with AsyncWebServer
  Serial.println("OTA ready: http://<device_ip>/update");
  ElegantOTA.onEnd([](bool success) {
      Serial.println("ElegantOTA finished, restarting...");
      keymapPrefs.end();  // Optional
      preferences.end();  // Optional
      delay(500);
      ESP.restart();
  });

 
  //Camera Detection
  if (esp32camOnline) {
    Serial.println("Camera stream assumed available at: http://" + cameraIP + ":81/stream");
  }

  udp.begin(udpPort);
  Serial.println("📡 Listening for UDP camera broadcasts...");

  camPrefs.begin("camera", false);
  cameraIP = camPrefs.getString("lastIP", "");
    if (cameraIP != "") {
        Serial.println("ℹ️ Previously saved camera IP (from last session): " + cameraIP);
    } else {
        Serial.println("ℹ️ No previously saved camera IP found in Preferences.");
    }

  if (esp32camOnline && cameraIP != "") {
    broadcastInitialStates();  // 🔁 Sync CAM with current toggled states
  }


}

void loop()
{

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      char cmd = toupper(input.charAt(0));
      int val = (input.length() > 2) ? input.substring(2).toInt() : 255;
      moveCarS(cmd, constrain(val, 0, 255));
      Serial.printf("✅ Command: %c %d\n", cmd, val);
    }
  }

  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
    if (len > 0) {
      udpBuffer[len] = 0;
      String msg = String(udpBuffer);
      if (displayUDPserial){
       Serial.println("📨 Received UDP: " + msg);
      }

  if (msg.startsWith("FPS,")) {
      lastFpsReceived = millis();  // 👈 Track last FPS time
      wsCarInput.textAll(msg);
      if (displayUDPserial){
      Serial.println("📤 Forwarded FPS to WebSocket: " + msg);
      }
  }


  if (msg.startsWith("IMU,")) {
    lastImuReceived = millis();  // 👈 Track last IMU time
    int numCommas = 0;
    for (int i = 0; i < msg.length(); i++) {
      if (msg[i] == ',') numCommas++;
    }

    if (numCommas >= 7) {
      wsCarInput.textAll(msg);  // Raw CSV IMU already in correct format
      if (displayUDPserial) {
        Serial.println("📤 Forwarded IMU to frontend: " + msg);
      }
    } else {
      Serial.println("⚠️ IMU format invalid, expected 8 fields: " + msg);
    }
  }

  if (msg.startsWith("CALIB_DATA:")) {
    int sys, gyro, accel, mag;
    if (sscanf(msg.c_str(), "CALIB_DATA:%d,%d,%d,%d", &sys, &gyro, &accel, &mag) == 4) {
      String wsMsg = "CALIB:" + String(sys) + "," + String(gyro) + "," + String(accel) + "," + String(mag);
      wsCarInput.textAll(wsMsg);
      if (displayUDPserial) Serial.println("📤 Forwarded CALIB to frontend: " + wsMsg);

      // 🧠 Save to preferences
      Preferences imuPrefs;
      imuPrefs.begin("imu", false);
      imuPrefs.putInt("sys", sys);
      imuPrefs.putInt("gyro", gyro);
      imuPrefs.putInt("accel", accel);
      imuPrefs.putInt("mag", mag);
      imuPrefs.putBool("stored", true);
      imuPrefs.end();
      Serial.println("💾 Calibration saved to Preferences.");
    } else {
      Serial.println("⚠️ Invalid CALIB_DATA format: " + msg);
    }
  }


  if (msg.startsWith("GET_CALIB:YES")) {
      Preferences imuPrefs;
      imuPrefs.begin("imu", true);
      int sys = imuPrefs.getInt("sys", -1);
      int gyro = imuPrefs.getInt("gyro", -1);
      int accel = imuPrefs.getInt("accel", -1);
      int mag = imuPrefs.getInt("mag", -1);
      bool stored = imuPrefs.getBool("stored", false);
      imuPrefs.end();

      if (stored && sys >= 0) {
        String reply = "CALIB_SAVED," + String(sys) + "," + String(gyro) + "," + String(accel) + "," + String(mag);
        udp.beginPacket(cameraIP.c_str(), udpPort);
        udp.write((const uint8_t*)reply.c_str(), reply.length());
        udp.endPacket();
        Serial.println("📤 Sent saved calibration to CAM: " + reply);
      } else {
        Serial.println("ℹ️ No calibration stored yet.");
      }
  }



  if (msg.startsWith("CAMID:" + String(S3_CAM_ID))) {

      if (msg.indexOf("REQ_CRED:YES") != -1) {
          Serial.println("📣 CAM requested WiFi credentials, sending now...");
          String creds = "S3ID:" + String(S3_ID) + ",SSID:" + wifiSSID + ",PASS:" + wifiPassword;
          udp.beginPacket(broadcastIP, udpPort);
          udp.write((const uint8_t*)creds.c_str(), creds.length());
          udp.endPacket();
          if (displayUDPserial){
            Serial.println("📤 Sent credentials to CAM (on request).");
          }
      }


        // ✅ New: immediately acknowledge if you receive OK:YES (even if no IP/FW)
      if (msg.indexOf("OK:YES") != -1 && !camAcknowledged) {
          camAcknowledged = true;
          Serial.println("✅ CAM acknowledgment confirmed (from loop)!");

          // 🆕 Check if we are still in AP mode, and switch to STA now
          if (WiFi.getMode() == WIFI_AP) {
              Serial.println("🔄 Switching to STA mode now that CAM acknowledged...");

              // FULL cleanup before switching modes
              WiFi.disconnect(true, true);  // Disconnect + Erase flash config
              WiFi.softAPdisconnect(true);
              delay(200);
              WiFi.mode(WIFI_OFF);
              delay(500);  // Let stack fully reset

              Serial.println("✅ Wi-Fi stack fully reset, switching to STA mode...");
              WiFi.mode(WIFI_STA);
              connectToWiFiWithRetries(wifiSSID, wifiPassword, wifiRetryCount);

              wifiConnecting = true;
              wifiConnectStartTime = millis();

          }
      }

        int ipIndex = msg.indexOf("IP:");
        int fwIndex = msg.indexOf("FW:");
        //if (!wifiConnecting) {
            if (ipIndex != -1) {
                String newIP = (fwIndex != -1) ? msg.substring(ipIndex + 3, fwIndex - 1) : msg.substring(ipIndex + 3);
                newIP.trim();
                if (cameraIP != newIP) {
                    cameraIP = newIP;
                    esp32camOnline = true;
                    camPrefs.putString("lastIP", cameraIP);
                    Serial.println("Camera found via UDP at " + cameraIP);
                    wsCarInput.textAll("CAMIP," + cameraIP);
                }
            }
            if (fwIndex != -1) {
                String newFW = msg.substring(fwIndex + 3);
                newFW.trim();
                if (newFW != currentCamFirmware) {
                    currentCamFirmware = newFW;
                    Serial.println("Camera firmware version updated: " + currentCamFirmware);
                }
            }
        //}
    }

  }


  if (wifiConnecting) {
      if (WiFi.status() == WL_CONNECTED) {
          Serial.println("✅ WiFi connected!");

          // ✅ New: restart server + WS safely
          startWebServerAndWS();

          // 🆕 Ask CAM to send its IP after we connect
          String requestIP = "S3ID:" + String(S3_ID) + ",REQ_IP:YES";
          udp.beginPacket(broadcastIP, udpPort);
          udp.write((const uint8_t*)requestIP.c_str(), requestIP.length());
          udp.endPacket();
          Serial.println("📣 Requested CAM to announce IP.");


          // 🆕 Resend credentials to CAM after reconnect
          String confirmMsg = "S3ID:" + String(S3_ID) + ",SSID:" + wifiSSID + ",PASS:" + wifiPassword;
          udp.beginPacket(broadcastIP, udpPort);
          udp.write((const uint8_t*)confirmMsg.c_str(), confirmMsg.length());
          udp.endPacket();
          Serial.println("📤 Re-sent credentials to CAM after S3 connect");

          // Notify CAM we're ready
          String readyMsg = "S3ID:" + String(S3_ID) + ",READY:YES";

          for (int attempt = 0; attempt < 3; attempt++) {
              udp.beginPacket(broadcastIP, udpPort);
              udp.write((const uint8_t*)readyMsg.c_str(), readyMsg.length());
              udp.endPacket();
              Serial.printf("📤 Attempt %d: Sent READY message to CAM\n", attempt + 1);
              delay(500);  // short pause between attempts
          }

          Serial.println("📣 Sent READY message to CAM");


          wifiConnecting = false;  // stop checking

          Serial.print("Connected IP: ");
          Serial.println(WiFi.localIP());

          String existingList = preferences.getString("networks", "");
          if (existingList.indexOf(wifiSSID) == -1) {
              existingList += (existingList.length() > 0 ? "," : "") + wifiSSID;
              preferences.putString("networks", existingList);
          }
          String wifiKey = "wifi_" + wifiSSID;
          preferences.putString(wifiKey.c_str(), wifiPassword);

          if (cameraIP != "") {
              esp32camOnline = true;
              Serial.println("ESP32-CAM found at: " + cameraIP);
              camPrefs.putString("lastIP", cameraIP);
              broadcastInitialStates();  // 🔁 Sync CAM with current toggled states
          } else {
              Serial.println("ESP32-CAM NOT found.");
          }

      } else if (millis() - wifiConnectStartTime > 10000) {
          Serial.println("❌ WiFi connection timeout. Starting fallback AP...");
          wifiConnecting = false;  // stop checking

          WiFi.disconnect();
          WiFi.mode(WIFI_AP);
          WiFi.softAP(ap_ssid);
          udp.begin(udpPort);  // Make sure binding after AP comes up
          Serial.print("Started AP: ");
          Serial.println(ap_ssid);
          Serial.print("AP IP: ");
          Serial.println(WiFi.softAPIP());
      }
   }

    wsCarInput.cleanupClients();
  }

  static unsigned long lastFpsPoll = 0;
  if (millis() - lastFpsPoll > 1000 && esp32camOnline && cameraIP != "") {
    lastFpsPoll = millis();

    String fpsRequestMsg = "S3ID:" + String(S3_ID) + ",REQ_FPS:YES";
    udp.beginPacket(cameraIP.c_str(), udpPort);
    udp.write((const uint8_t*)fpsRequestMsg.c_str(), fpsRequestMsg.length());
    udp.endPacket();
  }

  static unsigned long lastImuPoll = 0;
  if (millis() - lastImuPoll > 500 && esp32camOnline && cameraIP != "") {
    lastImuPoll = millis();

    String imuRequestMsg = "S3ID:" + String(S3_ID) + ",REQ_IMU:YES";
    udp.beginPacket(cameraIP.c_str(), udpPort);
    udp.write((const uint8_t*)imuRequestMsg.c_str(), imuRequestMsg.length());
    udp.endPacket();
  }

  // 🛠️ If no telemetry received for 5s, re-send IP to CAM
  if (esp32camOnline && cameraIP != "") {
    unsigned long now = millis();
    bool imuSilent = now - lastImuReceived > 5000;
    bool fpsSilent = now - lastFpsReceived > 5000;

    if ((imuSilent || fpsSilent) && now - lastIpResend > 5000) {
      lastIpResend = now;

      String requestIP = "S3ID:" + String(S3_ID) + ",REQ_IP:YES";
      udp.beginPacket(broadcastIP, udpPort);  // 🔄 Broadcast so CAM can update S3 IP
      udp.write((const uint8_t*)requestIP.c_str(), requestIP.length());
      udp.endPacket();

      Serial.println("📣 No telemetry received — re-sent IP request to CAM");
    }
  }

    // 👇 Telemetry
  unsigned long now = millis();
  if (now - lastTelemetrySend > 1000) {
    lastTelemetrySend = now;

    int adcRaw = analogRead(BLevel);         // Battery
    int adcRawCharging = analogRead(CSense); // Charger

    // Battery divider: 22k (R1) + 10k (R2)
    const float battR1 = 22000.0;
    const float battR2 = 10000.0;

    // Charger divider: 10k (R1) + 6.8k (R2)
    const float chgR1 = 10000.0;
    const float chgR2 = 6800.0;

    // Battery voltage
    float correctionFactorB = 8.233 / 8.03;  // Measure Battery terminal voltage and serial reading ≈ 1.025
    float batteryVoltage = (adcRaw * 3.3 / 4095.0) * ((battR1 + battR2) / battR2) * correctionFactorB;

    // Charger voltage
    float correctionFactorC = 5.0 / 5.0;  // Adjust this if needed
    float Vadc = adcRawCharging * 3.3 / 4095.0;
    float chargerVoltage = Vadc * ((chgR1 + chgR2) / chgR2) * correctionFactorC;

    // Determine charger status with 3 states
    const char* chargingStatus = nullptr;

    if (chargerVoltage > 4) {
        chargingStatus = "YES";
    } else if (chargerVoltage < 2.5) {
        chargingStatus = "NO";
    } else {
        chargingStatus = "FAULT";  // Voltage in abnormal mid-range
    }

    //Serial.printf("🔋 Battery: %.2f V | 🔌 Charger: %.2f V (Raw: %d) | Charging: %s\n",
    //              batteryVoltage, chargerVoltage, adcRawCharging,
    //              chargingStatus);


    int batteryPercent = constrain((int)(((batteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0), 0, 100);

    int rssi = WiFi.RSSI();
    int wifiQuality = constrain(2 * (rssi + 100), 0, 100);

    //wsCarInput.textAll("FPS,15");  // Optional — you may want to calculate this in JS as discussed
    wsCarInput.textAll("BATT," + String(batteryPercent) + "," + String(batteryVoltage, 2) + "," + String(wifiQuality));
    wsCarInput.textAll("CHARGE," + String(chargingStatus));

    unsigned long uptimeSecs = millis() / 1000;
    float chipTemp = temperatureRead();
    wsCarInput.textAll("STATS," + String(uptimeSecs) + "," + String(chipTemp, 1));

    //Serial.printf("🔋 %.2fV (%d%%) | 📶 %d%% | 🌡 %.1f°C\n", batteryVoltage, batteryPercent, wifiQuality, chipTemp);
  }

  // In your existing loop(), before the final delay(1);
  if (!holdAux && auxAttached && millis() > auxDetachTime) {
    ledcDetachPin(auxServoPin);
    auxAttached = false;
  }


  if (!holdBucket && bucketAttached && millis() > bucketDetachTime) {
    ledcDetachPin(bucketServoPin);
    bucketAttached = false;
  }


  // ✅ Non-blocking PWM motor ramping
  for (int i = 0; i < 3; i++) {
    MotorState &m = motorStates[i];
    if (millis() - m.lastUpdateTime >= rampDelay && m.currentSpeed != m.targetSpeed) {
      m.lastUpdateTime = millis();
      if (m.currentSpeed < m.targetSpeed) {
        m.currentSpeed += rampStep;
        if (m.currentSpeed > m.targetSpeed) m.currentSpeed = m.targetSpeed;
      } else {
        m.currentSpeed -= rampStep;
        if (m.currentSpeed < m.targetSpeed) m.currentSpeed = m.targetSpeed;
      }

      if (m.dirForward) {
        digitalWrite(m.otherPin, LOW);
        ledcWrite(m.dirChannel, m.currentSpeed);
      } else {
        digitalWrite(m.dirPin, LOW);
        ledcWrite(m.otherChannel, m.currentSpeed);
      }
    }
  }

  if (shouldReboot) {
      delay(1000);
      ESP.restart();
  }
  delay(1);
}