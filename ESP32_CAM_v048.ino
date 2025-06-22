/*
  MINIEXCO Project 2025
  ESP32-CAM Dual Server: Streaming (port 81) + Control (port 80)
  V.047 - integrated DFROBOT BNO055 module, do not forget to change sensor_t to adafruit_sensor_t in libs: 
  C:\Users\...\Documents\Arduino\libraries\Adafruit_Unified_Sensor
  and
  C:\Users\...\Documents\Arduino\libraries\Adafruit_BNO055
  both in cpp and h files!
*/
#define CONFIG_ESP32_BROWNOUT_DET 0


#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"
// Disable brownout detector properly in new ESP32 cores
#include "esp_system.h"
#include <Preferences.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_Sensor.h>      // change sensor_t to adafruit_sensor_t in libs
#include <Adafruit_BNO055.h>      // change sensor_t to adafruit_sensor_t in libs


// Left/Right blink state
bool flashLeftActive = false;
bool flashRightActive = false;
bool flashBlinkState = false;
unsigned long flashLeftStart = 0;
unsigned long flashRightStart = 0;
const unsigned long turnSignalTimeout = 3000;  // 3 seconds auto timeout
bool emergencyActive = false;

unsigned long lastFlashToggle = 0;
const unsigned long flashBlinkInterval = 300;

#define WS_PIN 12
#define NUM_LEDS 10
Adafruit_NeoPixel wsLeds(NUM_LEDS, WS_PIN, NEO_GRB + NEO_KHZ800);
uint32_t savedColors[NUM_LEDS];  // Backup for restoring LED states

//DFROBOT BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // 0x28 = default I2C address
unsigned long lastIMUSend = 0;

// Define I2C pins
#define OLED_SDA 15
#define OLED_SCL 14

IPAddress s3IPAddress;  // Add global to store s3 IP

// OLED display resolution
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Initialize SSD1306 OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


#define ESP32_CAM_VERSION "v0.48"
#define S3_AP_SSID "MiniExco_Setup"
#define ROVER_ID "MINIEXCO_S3_002"
#define CAM_ID "MINIEXCO_002"

bool hasPsram = false;  // Global declaration

//Frame Counter
unsigned long lastFPSReport = 0;
int frameCounter = 0;

static unsigned long idleEyeStart = 0;
static bool idleEyeActive = false;

String s3IPStr = "";  // Displayed S3 IP


bool reconnecting = false;
unsigned long reconnectStart = 0;
String reconnectSSID = "";
String reconnectPass = "";


WiFiUDP udp;
const char* broadcastIP = "255.255.255.255";
const unsigned int udpPort = 4210;

Preferences prefs;

//String ssid = "OTC_GUEST";
//String password = "??@@343Leg1596@";

String ssid = "";
String password = "";
static char ssidChar[64] = {0};
static char passChar[64] = {0};

const char* otapassword = "Samir2012A";

#define PART_BOUNDARY "123456789000000000000987654321"
#define LED_FLASH_PIN 4

#define CAMERA_MODEL_AI_THINKER
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

httpd_handle_t stream_httpd = NULL;
httpd_handle_t control_httpd = NULL;
bool flashOn = false;


// Set up PWM (LEDC) for LED brightness control
#define LEDC_CHANNEL 2
#define LEDC_FREQUENCY 5000
#define LEDC_RESOLUTION 8

// GLOBAL VARIABLE
int lastBrightness = 255;  // default full brightness


// STREAM HANDLER (port 81)
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  char *part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" PART_BOUNDARY);
  if(res != ESP_OK) return res;

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    if(fb->format != PIXFORMAT_JPEG){
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if(!jpeg_converted){
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
        break;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    size_t hlen = snprintf((char *)part_buf, 64, "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if(res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if(res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--" PART_BOUNDARY "\r\n", strlen("\r\n--" PART_BOUNDARY "\r\n"));

    esp_camera_fb_return(fb);

    frameCounter++;

    if(res != ESP_OK) break;
  }
  return res;
}

static esp_err_t id_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  Preferences prefs;
  prefs.begin("camcfg", true);
  String camID = prefs.getString("camid", CAM_ID);
  prefs.end();

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, camID.c_str(), camID.length());
  return ESP_OK;
}

void putIntIfChanged(Preferences& prefs, const char* key, int value) {
  if (prefs.getInt(key, value + 1) != value) prefs.putInt(key, value);
}

void putBoolIfChanged(Preferences& prefs, const char* key, bool value) {
  if (prefs.getBool(key, !value) != value) prefs.putBool(key, value);
}

void putStringIfChanged(Preferences& prefs, const char* key, const String& value) {
  if (prefs.getString(key, "%%@@") != value) prefs.putString(key, value);
}


// CAMERA SETTINGS HANDLER (port 80) with Preferences support
static esp_err_t control_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    httpd_resp_send(req, "Sensor error", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  Preferences prefs;
  prefs.begin("camcfg", false);

  char query[128];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char value[16];

    if (httpd_query_key_value(query, "quality", value, sizeof(value)) == ESP_OK) {
      int quality = atoi(value);
      if (quality >= 10 && quality <= 63) {
        s->set_quality(s, quality);
        putIntIfChanged(prefs, "quality", quality);
      }
    }

    if (httpd_query_key_value(query, "res", value, sizeof(value)) == ESP_OK) {
      int res = atoi(value);
      if (res >= FRAMESIZE_96X96 && res <= FRAMESIZE_UXGA) {
        s->set_framesize(s, (framesize_t)res);
        putIntIfChanged(prefs, "res", res);
      }
    }

    if (httpd_query_key_value(query, "rot", value, sizeof(value)) == ESP_OK) {
      int rot = atoi(value);
      if (rot >= 0 && rot <= 3) {
        s->set_hmirror(s, rot & 1);
        s->set_vflip(s, rot & 2 ? 1 : 0);
        putIntIfChanged(prefs, "rot", rot);
      }
    }

    if (httpd_query_key_value(query, "sat", value, sizeof(value)) == ESP_OK) {
      int sat = atoi(value);
      s->set_saturation(s, sat);
      putIntIfChanged(prefs, "sat", sat);
    }

    if (httpd_query_key_value(query, "gray", value, sizeof(value)) == ESP_OK) {
      int gray = atoi(value);
      s->set_special_effect(s, gray ? 2 : 0);
      putBoolIfChanged(prefs, "gray", gray);
    }

    if (httpd_query_key_value(query, "bright", value, sizeof(value)) == ESP_OK) {
      int bright = atoi(value);
      s->set_brightness(s, bright);
      putIntIfChanged(prefs, "bright", bright);
    }

    if (httpd_query_key_value(query, "contrast", value, sizeof(value)) == ESP_OK) {
      int contrast = atoi(value);
      s->set_contrast(s, contrast);
      putIntIfChanged(prefs, "contrast", contrast);
    }

    if (httpd_query_key_value(query, "fps", value, sizeof(value)) == ESP_OK) {
      int fps = atoi(value);
      if (fps >= 5 && fps <= 60) {
        putIntIfChanged(prefs, "fps", fps);
      }
    }

    if (httpd_query_key_value(query, "camid", value, sizeof(value)) == ESP_OK) {
      putStringIfChanged(prefs, "camid", value);
    }

    if (httpd_query_key_value(query, "sharp", value, sizeof(value)) == ESP_OK) {
      int sharp = atoi(value);
      s->set_sharpness(s, sharp);
      putIntIfChanged(prefs, "sharp", sharp);
    }

    if (httpd_query_key_value(query, "denoise", value, sizeof(value)) == ESP_OK) {
      int denoise = atoi(value);
      s->set_denoise(s, denoise);
      putIntIfChanged(prefs, "denoise", denoise);
    }

    if (httpd_query_key_value(query, "gamma", value, sizeof(value)) == ESP_OK) {
      int gamma = atoi(value);
      putIntIfChanged(prefs, "gamma", gamma);
    }

    if (httpd_query_key_value(query, "compression", value, sizeof(value)) == ESP_OK) {
      int compression = atoi(value);
      putIntIfChanged(prefs, "compression", compression);
    }
  }

  prefs.end();
  httpd_resp_send(req, "Settings updated", HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}


// LED HANDLER (port 80)
static esp_err_t led_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char param[32] = {0};
  if (httpd_req_get_url_query_str(req, param, sizeof(param)) == ESP_OK) {
    char value[8];

    if (httpd_query_key_value(param, "brightness", value, sizeof(value)) == ESP_OK) {
        int brightness = atoi(value);
        brightness = constrain(brightness, 0, 255);
        lastBrightness = brightness;
        ledcWrite(LEDC_CHANNEL, brightness);
        flashOn = (brightness > 0);

        // 🔴 Add these lines
        Preferences prefs;
        prefs.begin("camcfg", false);
        prefs.putInt("led_brightness", brightness);
        prefs.end();

        httpd_resp_send(req, "LED Brightness Updated", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    if (httpd_query_key_value(param, "state", value, sizeof(value)) == ESP_OK) {
        flashOn = atoi(value);
        ledcWrite(LEDC_CHANNEL, flashOn ? lastBrightness : 0);
        
        String reply = String("LEDSTATE:") + (flashOn ? "1" : "0");
        httpd_resp_send(req, reply.c_str(), reply.length());
        return ESP_OK;
    }


  }

  httpd_resp_send(req, "Missing parameter", HTTPD_RESP_USE_STRLEN);
  return ESP_FAIL;
}


static esp_err_t get_settings_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  Preferences prefs;
  prefs.begin("camcfg", true); // Read-only
  String response = "{";
  response += "\"res\":" + String(prefs.getInt("res", hasPsram ? FRAMESIZE_UXGA : FRAMESIZE_SVGA)) + ",";
  response += "\"fps\":" + String(prefs.getInt("fps", 15)) + ",";
  response += "\"rot\":" + String(prefs.getInt("rot", 0)) + ",";
  response += "\"sat\":" + String(prefs.getInt("sat", 0)) + ",";
  response += "\"gray\":" + String(prefs.getBool("gray", false) ? 1 : 0);
  response += ",\"led\":" + String(prefs.getInt("led_brightness", 0));
  response += ",\"bright\":" + String(prefs.getInt("bright", 0));
  response += ",\"contrast\":" + String(prefs.getInt("contrast", 0));
  response += ",\"sharp\":" + String(prefs.getInt("sharp", 2));
  response += ",\"denoise\":" + String(prefs.getInt("denoise", 1));
  response += ",\"gamma\":" + String(prefs.getInt("gamma", 0));
  response += ",\"compression\":" + String(prefs.getInt("compression", 12));
  response += ",\"quality\":" + String(prefs.getInt("quality", 10));  // Default is 10
  response += "}";

  String camID = prefs.getString("camid", CAM_ID);
  prefs.end();

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, response.c_str(), response.length());
  return ESP_OK;
}



void startCameraServer() {
  // Main Control Server - Port 80
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;  // Set control port manually
  config.max_open_sockets = 2;
  config.stack_size = 4096;

  if (httpd_start(&control_httpd, &config) == ESP_OK) {
    static httpd_uri_t id_uri = {
      .uri = "/id",
      .method = HTTP_GET,
      .handler = id_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(control_httpd, &id_uri);


    static httpd_uri_t root_uri = {
      .uri       = "/",
      .method    = HTTP_GET,
      .handler   = [](httpd_req_t *req) {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, "ESP32-CAM Controller Running", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      },
      .user_ctx  = NULL
    };
    static httpd_uri_t led_uri = {
      .uri = "/led",
      .method = HTTP_GET,
      .handler = led_handler,
      .user_ctx = NULL
    };
    static httpd_uri_t control_uri = {
      .uri = "/control",
      .method = HTTP_GET,
      .handler = control_handler,
      .user_ctx = NULL
    };

    static httpd_uri_t get_settings_uri = {
      .uri = "/getsettings",
      .method = HTTP_GET,
      .handler = get_settings_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(control_httpd, &get_settings_uri);


    httpd_register_uri_handler(control_httpd, &root_uri);
    httpd_register_uri_handler(control_httpd, &led_uri);
    httpd_register_uri_handler(control_httpd, &control_uri);
  }

  // Stream Server - Port 81
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = 32769;  // Must be different from 32768!
  stream_config.max_open_sockets = 1;
  stream_config.stack_size = 8192;

  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    static httpd_uri_t stream_uri = {
      .uri       = "/stream",
      .method    = HTTP_GET,
      .handler   = stream_handler,
      .user_ctx  = NULL
    };
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void drawWakingEyeSmooth() {
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;

  for (float r = 0; r <= 16; r += 0.5) {
    display.clearDisplay();

    // Outer eyeball shape
    display.drawCircle(centerX, centerY, (int)r, SSD1306_WHITE);

    // Optional: add an "iris" to simulate eye opening
    if (r > 4) {
      display.fillCircle(centerX, centerY, (int)(r / 4), SSD1306_WHITE);
    }

    display.display();
    delay(30);  // Lower delay = smoother, but more CPU load
  }

  delay(300); // Brief pause after animation
}


void blinkEyeSmooth() {
  int eyeRadius = 8;
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;

  for (int i = 0; i < 2; i++) {
    // Closing eyelid
    for (int h = 0; h <= eyeRadius * 2; h++) {
      display.clearDisplay();
      display.drawCircle(centerX, centerY, eyeRadius, SSD1306_WHITE);
      display.fillRect(centerX - eyeRadius, centerY - eyeRadius, eyeRadius * 2, h, SSD1306_BLACK);
      display.display();
      delay(20);
    }

    // Pause with eye closed
    delay(100);

    // Opening eyelid
    for (int h = eyeRadius * 2; h >= 0; h--) {
      display.clearDisplay();
      display.drawCircle(centerX, centerY, eyeRadius, SSD1306_WHITE);
      display.fillRect(centerX - eyeRadius, centerY - eyeRadius, eyeRadius * 2, h, SSD1306_BLACK);
      display.display();
      delay(20);
    }

    delay(100);
  }
}

void drawProgressBar(int percent) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println("Handshake...");

  int barWidth = SCREEN_WIDTH - 20;
  int filled = (barWidth * percent) / 100;

  display.drawRect(10, 30, barWidth, 10, SSD1306_WHITE);        // Outline
  display.fillRect(10, 30, filled, 10, SSD1306_WHITE);          // Fill
  display.display();
}


void showTickMark() {
  display.clearDisplay();
  display.drawCircle(64, 32, 16, SSD1306_WHITE);  // circle around tick
  display.drawLine(58, 32, 62, 36, SSD1306_WHITE); // tick part 1
  display.drawLine(62, 36, 72, 24, SSD1306_WHITE); // tick part 2
  display.display();
  delay(800);
}

void showIPAddress(String ipStr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(ipStr, 0, 0, &x1, &y1, &w, &h);
  int x_centered = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x_centered, 0);
  display.println(ipStr);
  display.setCursor(0, 24);
  display.println("CAM Online!");
  display.println("Streaming ready");
  display.display();
}

void animateWaitingEyeLoop() {
  static unsigned long lastFrameTime = 0;
  static int step = 0;

  // Only update every 200ms
  if (millis() - lastFrameTime > 200) {
    lastFrameTime = millis();
    display.clearDisplay();

    int centerX = SCREEN_WIDTH / 2;
    int centerY = SCREEN_HEIGHT / 2;

    if (step % 2 == 0) {
      display.drawCircle(centerX, centerY, 8, SSD1306_WHITE); // open eye
    } else {
      display.fillRect(centerX - 8, centerY - 2, 16, 4, SSD1306_WHITE); // closed eye
    }

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(20, 52);
    display.println("Waiting for S3...");

    display.display();
    step++;
  }
}

void drawStylizedEye(int centerX, int centerY, int offsetX, int offsetY) {
  // DO NOT clear the display here — it erases IP text from animateIdleEye()

  // 1️⃣ Draw almond-shaped eye outline (sclera)
  display.drawRoundRect(centerX - 30, centerY - 15, 60, 30, 15, SSD1306_WHITE);

  // 2️⃣ Calculate iris center with offset
  int irisX = centerX + offsetX;
  int irisY = centerY + offsetY;

  // 3️⃣ Draw large white iris + black pupil
  display.fillCircle(irisX, irisY, 10, SSD1306_WHITE);
  display.fillCircle(irisX, irisY, 6, SSD1306_BLACK); // Pupil

  // 4️⃣ Draw glints
  display.fillCircle(irisX - 2, irisY - 2, 2, SSD1306_WHITE);
  display.fillCircle(irisX + 2, irisY + 1, 1, SSD1306_WHITE);

  display.display();
}




void animateIdleEye() {
  static int frame = 0;
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    // Show IP above eye
    display.clearDisplay();
    if (s3IPStr != "") {
      int16_t x1, y1;
      uint16_t w, h;
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.getTextBounds(s3IPStr, 0, 0, &x1, &y1, &w, &h);
      int x_centered = (SCREEN_WIDTH - w) / 2;
      display.setCursor(x_centered, 0);
      display.println(s3IPStr);
    }

    // Animate eye motion in smooth loop
    int irisOffsetX = sin(frame * 0.08) * 10;
    int irisOffsetY = cos(frame * 0.1) * 6;
    drawStylizedEye(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2 + 10, irisOffsetX, irisOffsetY);

    frame++;
  }
}

void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    // Skip 0 and 4 if a turn signal is active
    if ((i == 0 || i == 5) && flashLeftActive) continue;
    if ((i == 4 || i == 9) && flashRightActive) continue;
    savedColors[i] = wsLeds.Color(r, g, b);
    wsLeds.setPixelColor(i, savedColors[i]);
  }
  wsLeds.show();
}



// Flash state
struct FlashState {
  bool active = false;
  int ledIndex = 0;
  uint8_t r = 0, g = 0, b = 0;
  unsigned long startTime = 0;
  int duration = 150;
};

FlashState flashState;

void startFlashLed(int index, uint8_t r, uint8_t g, uint8_t b, int duration = 150) {
  if (index < 0 || index >= NUM_LEDS) return;
  flashState.active = true;
  flashState.ledIndex = index;
  flashState.r = r;
  flashState.g = g;
  flashState.b = b;
  flashState.startTime = millis();
  flashState.duration = duration;

  wsLeds.setPixelColor(index, wsLeds.Color(r, g, b));
  wsLeds.show();
}

void handleFlashLed() {
  if (flashState.active && millis() - flashState.startTime > flashState.duration) {
    wsLeds.setPixelColor(flashState.ledIndex, 0);  // Turn off
    wsLeds.show();
    flashState.active = false;
  }
}


bool beaconActive = false;
bool beaconState = false;
unsigned long lastBeaconToggle = 0;
const unsigned long beaconInterval = 300;

void handleBeacon() {
  if (!beaconActive) return;

  if (millis() - lastBeaconToggle > beaconInterval) {
    lastBeaconToggle = millis();
    beaconState = !beaconState;
    for (int i = 0; i < NUM_LEDS; i++) {
      savedColors[i] = wsLeds.Color(beaconState ? 255 : 0, 0, beaconState ? 0 : 255);
      wsLeds.setPixelColor(i, savedColors[i]);
    }
    wsLeds.show();
  }
}

void handleTurnSignals() {
  unsigned long now = millis();

  if (emergencyActive) {
    if (now - lastFlashToggle > flashBlinkInterval) {
      lastFlashToggle = now;
      flashBlinkState = !flashBlinkState;
      wsLeds.setPixelColor(0, flashBlinkState ? wsLeds.Color(255, 255, 0) : 0);
      wsLeds.setPixelColor(4, flashBlinkState ? wsLeds.Color(255, 255, 0) : 0);
      wsLeds.setPixelColor(5, flashBlinkState ? wsLeds.Color(255, 255, 0) : 0);
      wsLeds.setPixelColor(9, flashBlinkState ? wsLeds.Color(255, 255, 0) : 0);
      wsLeds.show();
    }
    return;
  }

  if (!flashLeftActive && !flashRightActive) return;

  if (now - lastFlashToggle > flashBlinkInterval) {
    lastFlashToggle = now;
    flashBlinkState = !flashBlinkState;

    if (flashLeftActive) {
      if (flashBlinkState){
        wsLeds.setPixelColor(0, wsLeds.Color(255, 255, 0));  // yellow
        wsLeds.setPixelColor(5, wsLeds.Color(255, 255, 0));  // yellow
      }else{
        wsLeds.setPixelColor(0, savedColors[0]);  // restore original
        wsLeds.setPixelColor(5, savedColors[5]);  // restore original
      }
    }

    if (flashRightActive) {
      if (flashBlinkState){
        wsLeds.setPixelColor(4, wsLeds.Color(255, 255, 0));
        wsLeds.setPixelColor(9, wsLeds.Color(255, 255, 0));
      }else{
        wsLeds.setPixelColor(4, savedColors[4]);
        wsLeds.setPixelColor(9, savedColors[9]);
      }
    }

    wsLeds.show();
  }

  // Auto-timeout
  if (flashLeftActive && now - flashLeftStart > turnSignalTimeout) {
    flashLeftActive = false;
    wsLeds.setPixelColor(0, savedColors[0]);  // restore original
    wsLeds.setPixelColor(5, savedColors[0]);  // restore original
    wsLeds.show();
  }
  if (flashRightActive && now - flashRightStart > turnSignalTimeout) {
    flashRightActive = false;
    wsLeds.setPixelColor(4, savedColors[4]);
    wsLeds.setPixelColor(9, savedColors[9]);
    wsLeds.show();
  }
}




void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  Wire.begin(OLED_SDA, OLED_SCL);  // Initialize I2C with custom pins

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is the typical I2C addr
      Serial.println(F("SSD1306 allocation failed"));
      for(;;);
  }

  drawWakingEyeSmooth();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SXGA;           // or FRAMESIZE_SXGA (1280x1024) if UXGA is too heavy
  config.jpeg_quality = 10;                     // lower = better quality
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;


  hasPsram = psramFound();
  if (hasPsram) {
    config.frame_size = FRAMESIZE_SXGA;   // Native for OV5640
    config.jpeg_quality = 8;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;


  prefs.begin("camcfg", false);
  if (!prefs.isKey("camid")) {
    prefs.putString("camid", CAM_ID);
  }
  prefs.end();

  prefs.begin("camcfg", true);
  int savedRes = prefs.getInt("res", hasPsram ? FRAMESIZE_UXGA : FRAMESIZE_SVGA);
  if (prefs.isKey("res")) {
    config.frame_size = (framesize_t)savedRes;
  }
  int savedBright = prefs.getInt("bright", 0);
  int savedContrast = prefs.getInt("contrast", 0);
  int savedSat = prefs.getInt("sat", 0);
  int savedGray = prefs.getBool("gray", false);
  int savedRot = prefs.getInt("rot", 0); // 0 = none, 1 = hmirror, 2 = vflip, 3 = both
  int savedQuality = prefs.getInt("quality", 10);
  prefs.end();  // ✅ close after reading


  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera Init Failed");
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, savedBright);
    s->set_contrast(s, savedContrast);
    s->set_saturation(s, savedSat);
    s->set_special_effect(s, savedGray ? 2 : 0);
    s->set_hmirror(s, savedRot & 1);
    s->set_vflip(s, savedRot & 2 ? 1 : 0);

    // Extra tuning for OV5640
    //s->set_quality(s, 10);             // JPEG quality (lower is better quality)
    s->set_sharpness(s, 2);            // 0–3
    s->set_denoise(s, 1);              // Enable denoise
    s->set_exposure_ctrl(s, 1);        // Enable AEC
    s->set_aec_value(s, 300);          // Brightness target
    s->set_gain_ctrl(s, 1);            // Enable AGC
    s->set_gainceiling(s, (gainceiling_t)6);  // Max gain
    s->set_whitebal(s, 1);             // Enable AWB
    s->set_awb_gain(s, 1);             // More AWB gain
    s->set_wb_mode(s, 0);              // Auto white balance mode
    s->set_colorbar(s, 0);             // Disable test pattern
    s->set_quality(s, savedQuality);
  }



  ledcSetup(LEDC_CHANNEL, LEDC_FREQUENCY, LEDC_RESOLUTION);
  ledcAttachPin(LED_FLASH_PIN, LEDC_CHANNEL);
  ledcWrite(LEDC_CHANNEL, 0);  // Start with LED off

  while (true) {
    Serial.println("Connecting to S3 AP for handshake...");
    WiFi.begin(S3_AP_SSID, "");
    udp.begin(udpPort);  // 🛠 START UDP LISTENING IMMEDIATELY

    unsigned long connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connectStart < 5000) {
      animateWaitingEyeLoop();  // <-- Smooth blinking eye animation
      delay(10);  // Give CPU breathing room
    }


    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ Connected to S3 AP!");
    } else {
      Serial.println("\n❌ Failed to connect to S3 AP, retrying...");
      WiFi.disconnect(true);
      delay(200);
      WiFi.mode(WIFI_OFF);  // 💥 FULL RESET
      delay(300);
      WiFi.mode(WIFI_STA);  // 🔄 Re-enable STA mode fresh
      delay(100);

      continue;
    }

    Serial.println("Waiting for S3 WiFi credentials...");

    // 🔥 NEW: Actively request credentials from S3
    String requestMsg = "CAMID:" + String(CAM_ID) + ",REQ_CRED:YES";
    udp.beginPacket(broadcastIP, udpPort);
    udp.write((const uint8_t*)requestMsg.c_str(), requestMsg.length());
    udp.endPacket();
    Serial.println("📣 Requested WiFi credentials from S3.");

    bool receivedCredentials = false;
    unsigned long startTime = millis();

      unsigned long elapsed = 0;
      unsigned long totalWait = 10000;
      while ((elapsed = millis() - startTime) < totalWait) {
        int progress = (elapsed * 100) / totalWait;
        drawProgressBar(progress);

      int packetSize = udp.parsePacket();
      if (packetSize) {
        char incomingPacket[128];
        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) {
          incomingPacket[len] = 0;
          String msg = String(incomingPacket);
          Serial.println("📨 Received UDP: " + msg);
          // Always store the sender IP as S3
          s3IPAddress = udp.remoteIP();
          s3IPStr = s3IPAddress.toString();
          Serial.print("👀 Detected S3 IP: ");
          Serial.println(s3IPAddress);

          if (msg.startsWith("S3ID:" + String(ROVER_ID))) {
            int ssidIndex = msg.indexOf("SSID:");
            int passIndex = msg.indexOf("PASS:");
            if (ssidIndex != -1 && passIndex != -1) {
              String newSSID = msg.substring(ssidIndex + 5, msg.indexOf(",", ssidIndex));
              newSSID.trim();
              String newPass = msg.substring(passIndex + 5);
              newPass.trim();
              newSSID.toCharArray(ssidChar, sizeof(ssidChar));
              newPass.toCharArray(passChar, sizeof(passChar));
              Serial.printf("✅ Updated WiFi credentials from S3: SSID='%s', PASS='%s'\n", ssidChar, passChar);
              receivedCredentials = true;
              break;
            }
          }
        }
      }
    }

    if (receivedCredentials) {
        // ✅ Send acknowledgment back to S3 before switching networks
        String okMsg = "CAMID:" + String(CAM_ID) + ",OK:YES";
        udp.beginPacket(broadcastIP, udpPort);
        udp.write((const uint8_t*)okMsg.c_str(), okMsg.length());
        udp.endPacket();
        Serial.println("📣 Sent OK to S3:" + String(okMsg));

        break;  // exit loop and proceed to Wi-Fi connection
    }

    else {
      Serial.println("❌ No credentials received, retrying handshake...");
      WiFi.disconnect(true);
      delay(2000);
    }
  }

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);    // Full reset
  delay(300);
  WiFi.mode(WIFI_STA);    // Re-enable cleanly
  delay(100);
  WiFi.begin(ssidChar, passChar);
  Serial.printf("🔄 Attempting to connect to STA with SSID='%s' PASS='%s'\n", ssidChar, passChar);

  udp.begin(udpPort);     // ✅ Now safe to rebind


  // WAIT for WiFi to connect
  unsigned long connectTimeout = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - connectTimeout < 30000) {
      delay(500);
      Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ STA connected!");
      if (MDNS.begin("esp32cam")) Serial.println("mDNS started");
      Serial.println(WiFi.localIP());

  // ✅ Update OLED: big IP on top, rest below
  display.clearDisplay();

    blinkEyeSmooth();  
    display.clearDisplay();       
    showTickMark();  

  display.clearDisplay();
  // 1️⃣ IP Address - BIG & Centered (YELLOW zone)
  String ipStr = s3IPAddress.toString();
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(1);  // Or 3 if you want bigger, but 2 is safer
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(ipStr, 0, 0, &x1, &y1, &w, &h);
  int x_centered = (SCREEN_WIDTH - w) / 2;
  int y_position = 0;  // Top of the screen (yellow zone)
  display.setCursor(x_centered, y_position);
  display.println(ipStr);

    // Start eye animation after IP is shown
    unsigned long idleStart = millis();
    while (millis() - idleStart < 5000) {  // show idle animation for 5 seconds
      animateIdleEye();
      delay(10);
    }


  }
  else {
      Serial.println("\n❌ STA failed to connect within timeout.");
      // OPTIONAL: show failure on OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.println("WiFi failed!");
      display.println("Retrying...");
      display.display();
  }


  startCameraServer();


  ArduinoOTA.onStart([]() {
    esp_camera_deinit();
    if (stream_httpd) httpd_stop(stream_httpd);
    if (control_httpd) httpd_stop(control_httpd);
    Serial.println("Start updating...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nUpdate complete!");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.setTimeout(10000);
  ArduinoOTA.setPassword(otapassword);
  ArduinoOTA.begin();
  Serial.println("✅ ArduinoOTA ready, listening on port 3232");
  Serial.println("ℹ️ Waiting for STA connection... Will announce once connected.");

  String msg = String("CAMID:") + CAM_ID + ",IP:" + WiFi.localIP().toString() + ",FW:" + ESP32_CAM_VERSION;
  udp.beginPacket(broadcastIP, udpPort);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();
  Serial.println("📣 Announced camera on network.");

  Serial.printf("Restored settings: res=%d, bright=%d, contrast=%d, sat=%d, gray=%d, rot=%d\n",
                savedRes, savedBright, savedContrast, savedSat, savedGray, savedRot);

  // ✅ Update OLED: big IP on top, rest below
  display.clearDisplay();

  // 1️⃣ IP Address - BIG & Centered (YELLOW zone)
  String ipStr = s3IPAddress.toString();
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(1);  // Or 3 if you want bigger, but 2 is safer
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(ipStr, 0, 0, &x1, &y1, &w, &h);
  int x_centered = (SCREEN_WIDTH - w) / 2;
  int y_position = 0;  // Top of the screen (yellow zone)
  display.setCursor(x_centered, y_position);
  display.println(ipStr);

    // Start eye animation after IP is shown
    unsigned long idleStart = millis();
    while (millis() - idleStart < 5000) {  // show idle animation for 5 seconds
      animateIdleEye();
      delay(10);
    }

  for (int i = 0; i < NUM_LEDS; i++) {
    savedColors[i] = 0;  // Default all to off
  }

  wsLeds.begin();
  wsLeds.show();  // Initialize all to off

//DFROBOT BNO055
if (!bno.begin()) {
  Serial.println("❌ BNO055 not detected. Check I2C wiring or address!");
} else {
  Serial.println("✅ BNO055 initialized.");
  bno.setExtCrystalUse(true);
}

// Inside setup(), after UDP begin and bno begin:
String ask = "GET_CALIB:YES";
udp.beginPacket(s3IPAddress, udpPort);
udp.write((const uint8_t*)ask.c_str(), ask.length());
udp.endPacket();
Serial.println("📨 Requested stored calibration from ESP32-S3.");


}



void loop() {
    // Check for incoming UDP packets
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char incomingPacket[128];
        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) {
            incomingPacket[len] = 0;
            String msg = String(incomingPacket);
            Serial.println("📨 Received UDP: " + msg);

            // Always store the sender IP as S3
            s3IPAddress = udp.remoteIP();
            Serial.print("👀 Detected S3 IP: ");
            Serial.println(s3IPAddress);

            // ✅ OLED: Update to show new S3 IP each time we detect it
            s3IPStr = s3IPAddress.toString();

            // Optional: log it
            Serial.println("🖥️ S3 IP updated to OLED: " + s3IPStr);

            // Handle LED control commands from S3
            if (msg == "LEDON") {
              beaconActive = false;
              flashLeftActive = false;
              flashRightActive = false;
              setAllLeds(255, 255, 255);  // White
            }else if (msg == "LEDOFF") {
              beaconActive = false;
              flashLeftActive = false;
              flashRightActive = false;
              setAllLeds(0, 0, 0);  // Off
            } else if (msg == "FLASH_LEFT") {
              flashLeftActive = true;
              flashLeftStart = millis();  // ⏱️ Start timeout
              flashRightActive = false;   // 🚫 Turn off right to avoid conflict
            } else if (msg == "FLASH_RIGHT") {
              flashRightActive = true;
              flashRightStart = millis(); // ⏱️ Start timeout
              flashLeftActive = false;    // 🚫 Turn off left to avoid conflict
            } else if (msg == "BEACON") {
              beaconActive = true;  // Will be handled by handleBeacon()
            }
            else if (msg == "BEACON_OFF") {
              beaconActive = false;
              setAllLeds(0, 0, 0);  // Turn off LEDs when stopping beacon
            } else if (msg == "FLASH_LEFT_OFF") {
              flashLeftActive = false;
              wsLeds.setPixelColor(0, 0);  // Turn off left LED
              wsLeds.setPixelColor(5, 0);  // Turn off left LED
              wsLeds.show();
            } else if (msg == "FLASH_RIGHT_OFF") {
              flashRightActive = false;
              wsLeds.setPixelColor(4, 0);  // Turn off right LED
              wsLeds.setPixelColor(9, 0);  // Turn off right LED
              wsLeds.show();
            } else if (msg == "EMERGENCY") {
              emergencyActive = true;
              flashLeftActive = false;
              flashRightActive = false;
            } else if (msg == "EMERGENCY_OFF") {
              emergencyActive = false;
              wsLeds.setPixelColor(0, 0);
              wsLeds.setPixelColor(4, 0);
              wsLeds.setPixelColor(5, 0);
              wsLeds.setPixelColor(9, 0);
              wsLeds.show();
            }


            // ✅ Start idle eye for 5s after S3 IP update
            if (s3IPStr != "" && !idleEyeActive) {
              idleEyeActive = true;
              idleEyeStart = millis();
            }

            if (idleEyeActive) {
              animateIdleEye();
              if (millis() - idleEyeStart > 5000) {
                idleEyeActive = false;
              }
            }


            if (msg.startsWith("S3ID:" + String(ROVER_ID))) {
                int ssidIndex = msg.indexOf("SSID:");
                int passIndex = msg.indexOf("PASS:");

                // 🆕 Handle IP request
                if (msg.indexOf("REQ_IP:YES") != -1) {
                    String announceMsg = "CAMID:" + String(CAM_ID) + ",IP:" + WiFi.localIP().toString() + ",FW:" + ESP32_CAM_VERSION;
                    udp.beginPacket(broadcastIP, udpPort);
                    udp.write((const uint8_t*)announceMsg.c_str(), announceMsg.length());
                    udp.endPacket();
                    Serial.println("📣 Responded to S3 IP request: " + announceMsg);
                }

                else if (msg.indexOf("REQ_FPS:YES") != -1) {
                    int currentFPS = frameCounter;
                    frameCounter = 0;

                    String fpsReply = "FPS," + String(currentFPS);
                    udp.beginPacket(s3IPAddress, udpPort);
                    udp.write((const uint8_t*)fpsReply.c_str(), fpsReply.length());
                    udp.endPacket();

                    Serial.println("📤 Responded with FPS: " + fpsReply);
                }

                else if (msg.indexOf("REQ_CALIB:YES") != -1) {
                    uint8_t sys, gyro, accel, mag;
                    bno.getCalibration(&sys, &gyro, &accel, &mag);

                    String reply = "CALIB," + String(sys) + "," + String(gyro) + "," + String(accel) + "," + String(mag);
                    udp.beginPacket(s3IPAddress, udpPort);
                    udp.write((const uint8_t*)reply.c_str(), reply.length());
                    udp.endPacket();

                    Serial.println("📤 Responded with calibration data: " + reply);
                }

                else if (msg.indexOf("START_CALIB:YES") != -1) {
                    // Normally you must move the sensor to calibrate it.
                    // We simulate it or assume it's calibrated for now
                    Serial.println("⚙️ Starting IMU calibration...");

                    // Optionally you can wait/move here or flash LED

                    delay(5000);  // pretend calibration is happening

                    uint8_t sys, gyro, accel, mag;
                    bno.getCalibration(&sys, &gyro, &accel, &mag);

                    String calibReply = "CALIB," + String(sys) + "," + String(gyro) + "," + String(accel) + "," + String(mag);
                    udp.beginPacket(s3IPAddress, udpPort);
                    udp.write((const uint8_t*)calibReply.c_str(), calibReply.length());
                    udp.endPacket();

                    Serial.println("📤 Sent calibrated values: " + calibReply);
                }

                else if (msg.indexOf("REQ_IMU:YES") != -1) {
                    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
                    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
                    int temp = bno.getTemp();

                    String imuMsg = "IMU," + String(euler.x(), 1) + "," +
                                            String(euler.y(), 1) + "," +
                                            String(euler.z(), 1) + "," +
                                            String(mag.x(), 1) + "," +
                                            String(mag.y(), 1) + "," +
                                            String(mag.z(), 1) + "," +
                                            String(temp);


                    udp.beginPacket(s3IPAddress, udpPort);
                    udp.write((const uint8_t*)imuMsg.c_str(), imuMsg.length());
                    udp.endPacket();

                    Serial.println("📤 Responded with IMU: " + imuMsg);
                }


                else if (ssidIndex != -1 && passIndex != -1) {
                    String newSSID = msg.substring(ssidIndex + 5, msg.indexOf(",", ssidIndex));
                    newSSID.trim();
                    String newPass = msg.substring(passIndex + 5);
                    newPass.trim();

                    // ✅ Send OK acknowledgment back
                    String okMsg = "CAMID:" + String(CAM_ID) + ",OK:YES";
                    udp.beginPacket(broadcastIP, udpPort);
                    udp.write((const uint8_t*)okMsg.c_str(), okMsg.length());
                    udp.endPacket();
                    Serial.println("📣 Sent OK to S3:" + String(okMsg));

                    // ✅ Check if already on this network
                    if (WiFi.SSID() != newSSID) {
                        Serial.println("🔄 Starting reconnect to new WiFi...");
                        WiFi.disconnect(true);
                        delay(100);
                        WiFi.mode(WIFI_OFF);
                        delay(300);
                        WiFi.mode(WIFI_STA);
                        delay(100);
                        reconnecting = true;
                        reconnectStart = millis();
                        reconnectSSID = newSSID;
                        reconnectPass = newPass;
                        reconnectSSID.toCharArray(ssidChar, sizeof(ssidChar));
                        reconnectPass.toCharArray(passChar, sizeof(passChar));
                        Serial.printf("🔄 Reconnecting to SSID='%s' PASS='%s'\n", ssidChar, passChar);
                        WiFi.begin(ssidChar, passChar);

                    } else {
                        Serial.println("⚠️ Already connected to requested SSID, skipping reconnect.");
                    }
                }
            }
        }
    }

    // ✅ Non-blocking reconnect handling
    if (reconnecting) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("✅ Non-blocking reconnect successful!");
            reconnecting = false;

            // Re-announce to S3
            String msg = String("CAMID:") + CAM_ID + ",IP:" + WiFi.localIP().toString() + ",FW:" + ESP32_CAM_VERSION;
            udp.beginPacket(broadcastIP, udpPort);
            udp.write((const uint8_t*)msg.c_str(), msg.length());
            udp.endPacket();
            Serial.println("📣 Re-announced camera on network.");

        } else if (millis() - reconnectStart > 10000) {
            Serial.println("❌ Non-blocking reconnect timed out.");
            reconnecting = false;
        }
    }

    // Keep OTA running
    ArduinoOTA.handle();
    handleFlashLed();
    handleBeacon();
    handleTurnSignals();

    delay(1);
}

