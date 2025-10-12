/***************************************************************
 *    MiniExco Robot Firmware
 *    Hardware: ESP32-S3-SPK v1.0 (8MB PSRAM / 16MB flash)
 *    - Dual microphones, DAC to mono speaker
 *    - microSD card slot, OV2640 camera, WS2812B LEDs
 *    - Designed for autonomous robotics & media playback
 *
 *    ⚠️  CRITICAL HARDWARE REQUIREMENT: HEATSINK **AND ACTIVE COOLING**!
 *    ------------------------------------------------------------------
 *    This project runs the ESP32-S3 at extremely high load (Wi-Fi, camera, SD, audio).
 *    >>> YOU MUST INSTALL BOTH:
 *        1. A METAL HEATSINK on the ESP32-S3 chip (with thermal adhesive or pad)
 *        2. A 25mm x 25mm (or similar) LOW-PROFILE 5V COOLING FAN for active airflow
 *    ------------------------------------------------------------------
 *    Failure to provide **both** passive (heatsink) and active (fan) cooling will result in:
 *      - Overheating, Wi-Fi/network failures, random resets, severe instability, or PERMANENT DAMAGE.
 *      - There is NO built-in hardware or automatic thermal protection in ESP32-S3!
 *    ------------------------------------------------------------------
 *    [Edit this notice only if you confirm a heatsink AND a cooling fan are properly installed!]
 *
 *    Circuit diagrams, webserver files, and wiring drawings:
 *      [add link here]
 *    Video tutorials:
 *      [add link here]
 *
 *    Revision History:
         v2.0.70: Tweak Controls tab
                 Telemetry recording logics tweaks

        v2.0.69: Controls assignment for both keyboard and joypad
                 Modal window tab logics tweaks
                 İncomplete file uploads rebooting and corrupting existing files fix

        v2.0.68: Added camera stream start/stop button in interface
                 Implement async Json
                 SDLock to prevent collisions
                 Optimized code for webserverdisconnects
 *      v2.0.67: Optimize PSRAM usage; move variables/functions to PSRAM to free Heap on ESP32-S3.
 *      v2.0.66: OLED fixes; system sound queue; mic streaming; voltage filtering; MQTT Home Assistant discovery.
 *      v2.0.64: Heap optimizations; playlist loading optimized; cleaned up sketch; custom partition; frontend version in MQTT.
 *      v2.0.63: Split frontend CSS for index/settings; fixed stream hangs; added firmware version to MQTT.
 *      v2.0.62: BLE joystick PWM; MQTT for Home Assistant; serial debug flag; more fixes.
 *      v2.0.61: Home Assistant support.
 *      v2.0.60: System WAV sound playback (boot, Wi-Fi, etc.); frontend/backend system sound volume.
 *      v2.0.59: Media/video playback (fullscreen/windowed, controls); auto reindex after recording.
 *      v2.0.58: Media library in settings tab.
 *      v2.0.57: Video/photo capture to SD; camera model in settings; apply cam settings on boot; file manager path fixes.
 *      v2.0.56: Telemetry logging to CSV; overlay battery voltage charts.
 *
 *    TODO List:
 *      - Path following algorithms
 *      - Fix file manager refresh after upload
 *      - Fix large file uploads
 *      - OLED battery icon artifacts
 *
 ***************************************************************/

/* Bluepad32 documentation: https...
*/

// ============================================================================
// Feature switches (override with -D in build flags if you like)
// ----------------------------------------------------------------------------
// USE_BLUEPAD32
//   0 = exclude Bluepad32 gamepad support (saves flash/RAM, faster compile)
//   1 = include Bluepad32 (needs Bluepad32 library + BT enabled)
//   Tip: set to 0 unless you actually use a BT gamepad.
//
// DEBUG_SERIAL
//   0 = silence most DBG_PRINT/DBG_PRINTF calls
//   1 = enable verbose serial logging (requires Serial.begin in setup())
//   Tip: keep 1 while developing; switch to 0 for production.
//
// USE_PSRAM
//   0 = use internal heap only for large buffers (frame buffers, media queues)
//   1 = prefer PSRAM for large buffers (much more space; slightly slower access)
//   Notes:
//     - Board must actually have PSRAM. We’ll detect at runtime and fall back.
//     - Good for camera frames, audio buffers, MJPEG chunking, JSON docs.
// ============================================================================

#define USE_BLUEPAD32  0   // 0/1
#define DEBUG_SERIAL   1
#define USE_PSRAM      1

// --- Core / Arduino ---
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <time.h>

// --- NVS / Preferences ---
#include <Preferences.h>

// --- Filesystems / Storage ---
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <sys/stat.h>
#include <utime.h>
#include <mbedtls/sha256.h>  //upload file verification

// Optional fast FS stats (used only if present)
#if __has_include(<sys/statvfs.h>)
  #include <sys/statvfs.h>
#endif

// --- Async web stack ---
#include <AsyncTCP.h>                 // required by ESPAsyncWebServer on ESP32
#include <ESPAsyncWebServer.h>        // (patched per your note)
#include <ArduinoJson.h>
#include <AsyncJson.h>

// --- Networking helpers ---
#include <ESPmDNS.h>
#include <ElegantOTA.h>               // (patched per your note)
#include "esp_wifi.h"
#include "esp_sntp.h"

// --- MQTT ---
#include <PubSubClient.h>

// --- Camera / Display / Sensors ---
#include "esp_camera.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// --- Audio / I2S / LEDC ---
#include <Audio.h>                    // ESP32-audioI2S
#include "driver/i2s.h"
#include "driver/ledc.h"

// --- System / FreeRTOS / PSRAM ---
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <esp32-hal-psram.h>
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <semphr.h>

// --- Optional / project helpers ---
#if USE_PSRAM
  #include "PsramJsonDocHelper.h"
#endif

#if USE_BLUEPAD32
  #include <Bluepad32.h>
#endif

// --- STL / utilities ---
#include <map>
#include <string>
#include <vector>
//#include <iostream>
//#include <sstream>




#define FIRMWARE_VERSION "v2.0.70"

#define S3_ID "MINIEXCO_S3_V1_01"


//---------------------Preferences----------------------------------------------------------------------------------------------------------

Preferences preferences, prefs, wifiPrefs, camPrefs, keymapPrefs, joymapPrefs, imuPrefs, uiPrefs, oledPrefs, camEnablePrefs;

//------------------------------------NPT Tıme----------------------------------------------------------------

int myTimezone = +3; // For US Eastern (UTC-5)
long gmtOffset_sec = myTimezone * 3600;
bool timeIsValid = false;
unsigned long lastTimeCheck = 0;


//-------------------------------------Globals for MQTT---------------------------------------------------------------------
volatile bool mqttNeedsReconnect = false;
unsigned long mqttTelemetryInterval = 5000; // ms, default 1 seconds
unsigned long lastMqttTelemetry = 0;

bool mqttDiscoveryPublished = false;

bool mqttConnected = false;
String mqttLastError = "";

struct MQTTConfig {
  bool enable;
  String host;
  int port;
  String user;
  String pass;
  String topic_prefix;
};

MQTTConfig mqttCfg;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

unsigned long lastMqttAnnounce = 0;

String getMqttPrefix() {
  loadMqttConfig(); // Ensure latest config
  String p = mqttCfg.topic_prefix;
  while (p.endsWith("/")) p.remove(p.length()-1);
  if (p.length()) p += "/";
  return p;
}



//-----------------------------------Globals for system sounds to play on event----------------------------------------------
unsigned long lastLowBatteryBeep = 0;
bool lowBatteryAlertActive = false;
static bool wasCharging = false;              //for charging
static bool chargingCompletePlayed = false;   // For chargeComplete.wav
static bool endChargingPlayed = false;        // For endCharging.wav
static bool wasFullyCharged = false;          // For 100% detection
bool sirenPlaying = false;

volatile bool isSystemSoundPlaying = false;
String lastPlayedFile = "";
unsigned long lastPlayedTime = 0;
unsigned long soundRepeatDelay = 1000; // ms, adjust as needed

#define MAX_SYSTEM_SOUND_QUEUE 3
String systemSoundQueue[MAX_SYSTEM_SOUND_QUEUE];
uint8_t queueHead = 0, queueTail = 0;

// ---------------------------------- Globals for recording ---------------------------------------------------------
volatile bool  videoRecording  = false;
volatile bool  videoTaskActive = false;
TaskHandle_t   videoTaskHandle = nullptr;

//--------------------------------Telemetry Logging Global Variables------------------------------------------------

unsigned long telemetrySampleInterval = 1000; // e.g. 1000ms = 1 second
const size_t TELEMETRY_FILE_MAX_SIZE = 10 * 1024 * 1024; // 10 MB each telemetry data file size
String currentTelemetryFile = "/telemetry/telemetry_01.csv";

const char* TELEMETRY_CSV_HEADER =
  "datetime,timestamp,voltage,temp,charger,imu_euler_x,imu_euler_y,imu_euler_z,"
  "imu_mag_x,imu_mag_y,imu_mag_z,imu_temp,fps";

struct TelemetrySample {
  unsigned long timestamp = 0;
  int batteryPercent = 0;
  float voltage = 0;
  float charger = 0;
  int wifi = 0;
  float temp = 0;
  int fps = 0;
  float imu_euler_x = 0;
  float imu_euler_y = 0;
  float imu_euler_z = 0;
  float imu_mag_x = 0;
  float imu_mag_y = 0;
  float imu_mag_z = 0;
  float imu_temp = 0;
  // Add other fields if needed!
};


TelemetrySample currentSample;


//--------------------------------Bluepad32 Globals-----------------------------------------------------------------------

#if USE_BLUEPAD32
  #define STICK_DEADZONE 100
  #define DPAD_UP     0x01
  #define DPAD_DOWN   0x02
  #define DPAD_LEFT   0x04
  #define DPAD_RIGHT  0x08

  ControllerPtr myControllers[BP32_MAX_GAMEPADS];
#endif


//--------------------------------Audio Globals-------------------------------------------------------------------

Audio audio;

// ---- Folders to scan for music----
// --- Supported audio file extensions ---
const char* supportedExtensions[] = {
  ".mp3", ".wav", ".aac", ".flac", ".ogg", ".mod", ".mid", ".midi", ".opus"
};
const int numSupportedExtensions = sizeof(supportedExtensions)/sizeof(supportedExtensions[0]);

// Helper to check file extension
bool hasSupportedExtension(const String& name) {
  for (int i = 0; i < numSupportedExtensions; ++i) {
    if (name.endsWith(supportedExtensions[i])) return true;
  }
  return false;
}

// --- Folders to scan for audio files ---
std::vector<const char*> mediaFolders = {
  "/media/mp3",
};


bool loopMode = false;
bool shuffleMode = false;
int lastPausedTime = 0;         // in seconds
bool isPaused = false;
bool playbackStarted = false;

//------------------------------------Reindex globals-----------------------------------------------------------

int reindexTotal = 0;
bool reindexCounting = false; // true while counting files
bool reindexReadyToIndex = false; // true when ready to start indexing

volatile bool pendingReindex = false;
String reindexPath = "/";
File reindexDir;
File reindexIdx;
int reindexCount = 0;

//-------------------------------------File Upload----------------------------------------------------------------
namespace sdweb {             // keeps the symbol local to this file
  struct UploadCtx {
    String uploadPath;        // final path
    String tmpPath;           // temp path
    File   uploadFile;        // open temp handle
    String error;             // non-empty => failed
    size_t bytesWritten = 0;
    size_t bytesSinceYield = 0;     // for periodic flush/yield

    // Optional integrity check
    bool verifySha = false;
    mbedtls_sha256_context sha;
    String expectedShaHex;          // 64-hex (lowercase)
    uint8_t digest[32];             // computed
  };

}
using sdweb::UploadCtx;
//----------------------------------------Serial Debug----------------------------------------------------------------------------
/* As pins 19, 43 and 44 is used to control DRV8833 when debut is enabled, you will see serial but will be unable to control motors */

#ifdef DEBUG_SERIAL
  #define DBG_PRINT(...)    do { if (DEBUG_SERIAL) Serial.print(__VA_ARGS__); } while(0)
  #define DBG_PRINTLN(...)  do { if (DEBUG_SERIAL) Serial.println(__VA_ARGS__); } while(0)
  #define DBG_PRINTF(...)   do { if (DEBUG_SERIAL) Serial.printf(__VA_ARGS__); } while(0)

  #define PRINT_HEAP(step) \
    DBG_PRINTF("[HEAP][%s] Free: %u  MinFree: %u  MaxAlloc: %u\n", step, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
#endif

//--------------------------------------------------------------------------------------------------------------------------------


volatile bool micStreamActive = false;
volatile bool micCleanupPending = false;

Adafruit_SSD1306 display(128, 64, &Wire, -1);


// --- I2S config for MSM261S mic ---

#define MY_I2S_PORT I2S_NUM_1   // instead of I2S_NUM_0

#define I2S_MIC_WS    40
#define I2S_MIC_SD    38
#define I2S_MIC_SCK   39

// --- I2S config for NS4168 speaker ---
#define I2S_SPK_SD    9
#define I2S_SPK_BCLK  10
#define I2S_SPK_LRCK  45
#define I2S_SPK_PA    46  // Optional: amp enable

#define I2S_SAMPLE_RATE     16000
#define I2S_SAMPLE_BITS     I2S_BITS_PER_SAMPLE_16BIT
#define I2S_READ_LEN        1024


// ===== MIC STREAM STATE =====
static int16_t* micBuf = nullptr;
static size_t   micBufSamples = I2S_READ_LEN;                   // samples per read
static size_t   micBufBytes   = I2S_READ_LEN * sizeof(int16_t); // bytes per read
static uint32_t micLastSendMs = 0;


// Track current I2S state
enum I2SMode { I2S_NONE, I2S_MIC, I2S_SPEAKER };
I2SMode currentI2SMode = I2S_NONE;

// Channel numbers for ledcWrite/ledcSetup
#define CH_RIGHT_MOTOR_IN1 0
#define CH_RIGHT_MOTOR_IN2 1
#define CH_LEFT_MOTOR_IN1  2
#define CH_LEFT_MOTOR_IN2  3
#define CH_ARM_MOTOR_IN1   4
#define CH_ARM_MOTOR_IN2   5

// PWM channel mapping summary
#define PIN_RIGHT_MOTOR_IN1 16
#define PIN_RIGHT_MOTOR_IN2 17
#define PIN_LEFT_MOTOR_IN1  18

#if DEBUG_SERIAL
  #define PIN_LEFT_MOTOR_IN2  -1
  #define PIN_ARM_MOTOR_IN1   -1
  #define PIN_ARM_MOTOR_IN2   -1
#else
  #define PIN_LEFT_MOTOR_IN2  19
  #define PIN_ARM_MOTOR_IN1   43
  #define PIN_ARM_MOTOR_IN2   44
#endif


#define CH_BUCKET_SERVO    6
#define CH_AUX_SERVO       7

//-----------------------------------------------------------------------CAMERA-------------------

bool cameraEnabled = false;
bool cameraInitialized = false;

#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    33    // ✅ From MCLK
#define SIOD_GPIO_NUM    37    // ✅ From schematic SDA
#define SIOC_GPIO_NUM    36    // ✅ From schematic SCK

#define Y9_GPIO_NUM      47
#define Y8_GPIO_NUM      48
#define Y7_GPIO_NUM      42
#define Y6_GPIO_NUM      8
#define Y5_GPIO_NUM      6
#define Y4_GPIO_NUM      4
#define Y3_GPIO_NUM      5
#define Y2_GPIO_NUM      7

#define VSYNC_GPIO_NUM   35
#define HREF_GPIO_NUM    34
#define PCLK_GPIO_NUM    41

#define PWM_RES_BITS 14
#define PWM_PERIOD_US 20000


// -------------------------------------------------------------------------SD_|CARD Pin defines-----------------------------

#define SD_CS   2
#define SD_SCK  11
#define SD_MOSI 3
#define SD_MISO 12

// ---- SD mutex (recursive) ----
static SemaphoreHandle_t sdMutex = nullptr;

static inline void initSdMutex() {
  if (!sdMutex) sdMutex = xSemaphoreCreateRecursiveMutex();
}

struct SdLock {
  SdLock()  { xSemaphoreTakeRecursive(sdMutex, portMAX_DELAY); }
  ~SdLock() { xSemaphoreGiveRecursive(sdMutex); }
};

//----------I2C-------------------------------------------

#define BNO_SDA   15      // Use your actual I2C SDA pin for ESP32-S3!
#define BNO_SCL   14      // Use your actual I2C SCL pin for ESP32-S3!
Adafruit_BNO055 bno055 = Adafruit_BNO055(55, 0x28);  // 0x28 is default addr


volatile bool isSdUploadInProgress = false;
volatile bool isStreaming = true;
volatile int frameCount = 0;  // For FPS counting, used in loop()

//-----------------------Neopixel Globals------------------------------

#define LED_PIN 21
#define NEO_COUNT 12
Adafruit_NeoPixel pixels(NEO_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool beaconOn = false;
bool emergencyOn = false;
bool leftSignalActive = false;
bool rightSignalActive = false;

uint8_t beaconPhase = 0;
bool blinkState = false;  // For emergency and turn signals
unsigned long lastAnimUpdate = 0;
const unsigned long beaconInterval = 180;
const unsigned long blinkInterval = 400;

unsigned long lastPoliceFlash = 0;
int policePhase = 0; // 0 = red, 1 = blue
//----------------------------------------------------------------------



bool shouldReboot = false;
bool otaValid = false;

int wifiRetryCount = 5;  // Default fallback value

// Non-blocking auto-detach tracking
unsigned long auxDetachTime = 0;
bool auxAttached = false;

// Non-blocking auto-detach tracking
unsigned long bucketDetachTime = 0;
bool bucketAttached = false;

// Struct for a path point
struct PathPoint {
  float x, y;
};


static String pendingUploadPath;


int pathIndex = 0;                    // Current target point
bool pathFollowingActive = false;     // Following path state
float robotX = 0, robotY = 0;         // (Optional) Robot's estimated position


//--------------WiFi Globals------------------------------------------

// Wi-Fi Connection State Machine Variables
enum WifiState {
  WIFI_STA_CONNECTED,
  WIFI_STA_CONNECTING,
  WIFI_STA_RETRYING,
  WIFI_AP_MODE
};
WifiState wifiState = WIFI_STA_CONNECTING; // or WIFI_AP_MODE

String wifiSSID = "";
String wifiPassword = "";

bool wifiConnecting = false;
unsigned long wifiConnectStartTime = 0;

unsigned long wifiLastCheck = 0;
unsigned long wifiRetryStepTime = 0;
unsigned long wifiRetryStart = 0;
unsigned long wifiApScanTime = 0;
uint8_t wifiAutoRetryCount = 0;
uint8_t wifiRetrySubState = 0;


//--------------------------------------------------------------------

int lastBucketValue = 140;  // Set to your safe init value
int lastAuxValue = 150;


// -------------OLED Globals---------------------------------
#define BLevel 13
#define CSense 1

#define SCREEN_WIDTH 128   // or whatever your actual OLED width is
#define SCREEN_HEIGHT 64   // or whatever your OLED height is

#define TOTAL_OLED_FRAMES 30

// For OLED or UI feedback
unsigned long wifiOledLastMsg = 0;
String wifiOledLastStep = "";

int batteryPercentDisplay = 0; 

bool isCharging = false;

int wifiSignalStrength = 0;

String lastWsClientIP = "";
unsigned long lastWsConnectTime = 0;

//-----------------------------------------------------------

#define bucketServoPin  0
#define auxServoPin 20

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
#define ARM_MOTOR 2

#define DIR_STOP     0
#define DIR_FORWARD  1
#define DIR_BACKWARD -1


//Use a 10k + 4.7k divider — safe and very common. Let me know if yours is different.
unsigned long lastTelemetrySend = 0;
const float R1 = 10000.0; // 10k Ohm (top resistor)
const float R2 = 4700.0;  // 4.7k Ohm (bottom resistor)
const float MAX_BATTERY_VOLTAGE = 8.4; // 2S Li-ion full charge
const float MIN_BATTERY_VOLTAGE = 6.0; // 2S safe cutoff

//UI Settings variables------------

bool darkMode    = false;
bool horScreen   = false;
bool holdBucket  = false;
bool holdAux     = false;
bool tlmEnabled  = false;
bool sSndEnabled = false;
int  sSndVolume  = 15;


// global constants
const char* ap_ssid = "MiniExco_Setup";

bool light = false;


AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");


// --- PSRAM allocator for ESP32 Arduino ---
// This must be defined ONCE, above all vector declarations that use it.
#if USE_PSRAM
template <class T>
struct psram_allocator : public std::allocator<T> {
    template<class U> struct rebind { typedef psram_allocator<U> other; };
    T* allocate(std::size_t n) {
        void* p = ps_malloc(n * sizeof(T));
        if (!p) throw std::bad_alloc();
        return static_cast<T*>(p);
    }
    void deallocate(T* p, std::size_t) noexcept { free(p); }
};
#endif

// -----------------------------------------------------
// PSRAM-based or regular vector declarations for globals
// Place this block after your struct/class definitions (TelemetrySample, PathPoint)
// and after psram_allocator is defined
// -----------------------------------------------------

#if USE_PSRAM
// Use psram_allocator for all large or expandable vectors to store them in external PSRAM.
// This saves valuable internal RAM for stack/heap.
// You can add any additional vectors here to move them to PSRAM.

std::vector<String, psram_allocator<String>> playlist;           // Stores filenames or track names for media
std::vector<int, psram_allocator<int>> folderIndex;              // Index mapping for folders
std::vector<PathPoint, psram_allocator<PathPoint>> pathPoints;   // List of path points received from frontend UI
std::vector<TelemetrySample, psram_allocator<TelemetrySample>> telemetryBuffer; // Buffer for telemetry logging

#else
// If not using PSRAM, fall back to normal std::vector on internal RAM.

std::vector<String> playlist;
std::vector<int> folderIndex;
std::vector<PathPoint> pathPoints;
std::vector<TelemetrySample> telemetryBuffer;

#endif

int currentTrack = 0;
int currentVolume = 15;    // Default volume (0–21)
int currentFolder = 0;     // For nextfolder feature

String currentlyPlayingFile = "";
unsigned long lastMediaProgressSend = 0;

//----------------------------------CONTROLS------------------------------------------------------------------------------------------------

// ---- Forward declarations to satisfy Arduino's prototype generation ----
enum Action : uint8_t;  // makes 'Action' known to auto-generated prototypes

static Action actionFromName(const String& name);
static Action actionForKeyToken(String keyToken);
static Action actionForJoyButton(String btnName);
static void   dispatchAction(Action a, bool pressed);


// ---- Actions we support everywhere ----
enum Action : uint8_t {
  ACT_NONE = 0,
  ACT_FORWARD, ACT_BACKWARD, ACT_LEFT, ACT_RIGHT, ACT_STOP,
  ACT_ARM_UP, ACT_ARM_DOWN, ACT_BUCKET_UP, ACT_BUCKET_DOWN, 
  ACT_AUX_UP, ACT_AUX_DOWN, ACT_LIGHT_TOGGLE, ACT_BEACON_TOGGLE, 
  ACT_EMERGENCY_TOGGLE, ACT_HORN
};


// ---- Keyboard defaults (what the UI shows by default too) ----
static const struct { const char* action; const char* def; } KEYMAP_DEFAULTS[] = {
  {"forward",       "w"},
  {"backward",      "s"},
  {"left",          "a"},
  {"right",         "d"},
  {"stop",          " "},   // space
  {"arm_up",        " "},        
  {"arm_down",      " "},
  {"bucket_up",     "e"},
  {"bucket_down",   "q"},
  {"aux_up",        "r"},
  {"aux_down",      "f"},
  {"light_toggle",  "l"},
  {"beacon", "b"},
  {"emergency","x"},
  {"horn",          "h"},
};
static const size_t KEYMAP_N = sizeof(KEYMAP_DEFAULTS)/sizeof(KEYMAP_DEFAULTS[0]);

// ---- Joypad defaults (Bluepad32-style names) ----
// Keep simple & digital to start; you can extend with axes later.
static const struct { const char* action; const char* defBtn; } JOYMAP_DEFAULTS[] = {
  {"forward",       "DPAD_UP"},
  {"backward",      "DPAD_DOWN"},
  {"left",          "DPAD_LEFT"},
  {"right",         "DPAD_RIGHT"},
  {"stop",          "BTN_BACK"},       // “select/back” as stop
  {"arm_up",        "L_STICK_UP"},     // ⟵ changed
  {"arm_down",      "L_STICK_DOWN"},   // ⟵ changed 
  {"bucket_up",     "R1"},
  {"bucket_down",   "L1"},
  {"aux_up",        "R2_CLICK"},       // treat R2/L2 as “click” if your lib offers it
  {"aux_down",      "L2_CLICK"},
  {"light_toggle",  "X"},
  {"beacon", "Y"},
  {"emergency","B"},
  {"horn",          "A"},
};
static const size_t JOYMAP_N = sizeof(JOYMAP_DEFAULTS)/sizeof(JOYMAP_DEFAULTS[0]);

// Normalize UI tokens to stored form
static String normKeyToken(String s) {
  // handle literal single space BEFORE trimming
  if (s.length() == 1 && s[0] == ' ') return String(" ");

  s.trim();
  if (s.equalsIgnoreCase("Space")) return String(" ");
  if (s.startsWith("Arrow")) { s.toLowerCase(); return s; }  // ArrowUp -> arrowup
  s.toLowerCase();                                           // letters/digits -> lowercase
  return s;
}

// Accept both UI camelCase and firmware snake_case names
struct ActionAlias { const char* fw; const char* alias; };
static const ActionAlias ACTION_ALIASES[] = {
  {"forward","forward"}, {"backward","backward"}, {"left","left"}, {"right","right"},
  {"stop","stop"},
  {"arm_up","armUp"}, {"arm_down","armDown"},
  {"bucket_up","bucketUp"}, {"bucket_down","bucketDown"},
  {"aux_up","auxUp"}, {"aux_down","auxDown"},
  {"light_toggle","led"}, {"beacon","beacon"},
  {"emergency","emergency"},               // UI "emergency"
  {"emergency","emergency_toggle"},        // legacy alias accepted too
  {"horn","horn"},
};
static const size_t ACTION_ALIASES_N = sizeof(ACTION_ALIASES)/sizeof(ACTION_ALIASES[0]);

//------------------------------------------------------------------------------------------------FUNCTIONS----------------------------------------------------------------------


//--------------------------------------------------------------------Robot Cam Settings----------------------------------------------------------------

void startCameraServer();

bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;
  config.frame_size   = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count     = 2;
      config.grab_mode    = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size   = FRAMESIZE_SVGA;
      config.fb_location  = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
  #if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
  #endif
  }

  esp_err_t err = esp_camera_init(&config);

  DBG_PRINTF("[HEAP][After camera init] Free: %u, MinFree: %u, MaxAlloc: %u\n",
             ESP.getFreeHeap(),
             heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT),
             heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  DBG_PRINTF("[PSRAM][After camera init] Free: %u, MinFree: %u, MaxAlloc: %u\n",
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM),
             heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

  if (err != ESP_OK) {
    DBG_PRINTF("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s && s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG && s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
  return true;
}


void applySavedCamSettings() {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;

    // === Universal settings ===
    int res        = camPrefs.getInt("res", FRAMESIZE_VGA);
    int quality    = camPrefs.getInt("quality", 10);
    int contrast   = camPrefs.getInt("contrast", 0);
    int brightness = camPrefs.getInt("brightness", 0);
    int saturation = camPrefs.getInt("saturation", 0);
    int gray       = camPrefs.getInt("gray", 0);
    int hmirror    = camPrefs.getInt("hmirror", 0);
    int vflip      = camPrefs.getInt("vflip", 0);
    int special_effect = camPrefs.getInt("special_effect", 0);
    int wb_mode    = camPrefs.getInt("wb_mode", 0);
    int awb        = camPrefs.getInt("awb", 1);
    int agc        = camPrefs.getInt("agc", 1);
    int agc_gain   = camPrefs.getInt("agc_gain", 0);
    int aec        = camPrefs.getInt("aec", 1);
    int aec_value  = camPrefs.getInt("aec_value", 300);
    int aec2       = camPrefs.getInt("aec2", 0);
    int dcw        = camPrefs.getInt("dcw", 1);
    int bpc        = camPrefs.getInt("bpc", 0);
    int wpc        = camPrefs.getInt("wpc", 1);
    int raw_gma    = camPrefs.getInt("raw_gma", 1);
    int lenc       = camPrefs.getInt("lenc", 1);
    int gainceiling= camPrefs.getInt("gainceiling", 0);
    int colorbar   = camPrefs.getInt("colorbar", 0);

    s->set_framesize(s, (framesize_t)res);
    s->set_quality(s, quality);
    s->set_contrast(s, contrast);
    s->set_brightness(s, brightness);
    s->set_saturation(s, saturation);
    s->set_special_effect(s, special_effect);
    s->set_hmirror(s, hmirror);
    s->set_vflip(s, vflip);
    s->set_whitebal(s, awb);
    s->set_wb_mode(s, wb_mode);
    s->set_gain_ctrl(s, agc);
    s->set_agc_gain(s, agc_gain);
    s->set_exposure_ctrl(s, aec);
    s->set_aec_value(s, aec_value);
    s->set_aec2(s, aec2);
    s->set_dcw(s, dcw);
    s->set_bpc(s, bpc);
    s->set_wpc(s, wpc);
    s->set_raw_gma(s, raw_gma);
    s->set_lenc(s, lenc);
    s->set_gainceiling(s, (gainceiling_t)gainceiling);
    s->set_colorbar(s, colorbar);

    // === OV2640-specific ===
    if (s->id.PID == OV2640_PID) {
        int sharpness  = camPrefs.getInt("sharpness", 2);
        int denoise    = camPrefs.getInt("denoise", 0);
        int compression= camPrefs.getInt("compression", 12);

        if (s->set_sharpness) s->set_sharpness(s, sharpness);
        if (s->set_denoise)   s->set_denoise(s, denoise);
        if (s->set_quality)   s->set_quality(s, compression); // Can be optional
    }

    // === OV5640-specific ===
    if (s->id.PID == OV5640_PID) {
        int sharpness  = camPrefs.getInt("sharpness", 33);    // 0..255
        int denoise    = camPrefs.getInt("denoise", 0);       // 0,1
        int brightness = camPrefs.getInt("brightness", 0);    // -4..4
        int saturation = camPrefs.getInt("saturation", 0);    // -4..4
        int contrast   = camPrefs.getInt("contrast", 0);      // -4..4
        // int hue        = camPrefs.getInt("hue", 0);        // NOT USED

        if (s->set_sharpness)   s->set_sharpness(s, sharpness);
        if (s->set_denoise)     s->set_denoise(s, denoise);
        if (s->set_brightness)  s->set_brightness(s, brightness);
        if (s->set_saturation)  s->set_saturation(s, saturation);
        if (s->set_contrast)    s->set_contrast(s, contrast);
        // No set_hue for OV5640 or any camera
    }


    // === Add other models as needed ===
    // For example: OV3660, etc.
    // if (s->id.PID == OV3660_PID) { ... }

    // -- Optionally, print sensor info for debug --
    DBG_PRINTF("Camera PID: 0x%04X\n", s->id.PID);
}

// --- Enable/disable functions ---
bool enableCamera() {
  saveCameraPrefs(true);
  if (!cameraInitialized) {
    if (!startCamera()) {
      DBG_PRINTLN("enableCamera(): startCamera failed");
      return false;
    }
    cameraInitialized = true;
    startCameraServer();       // assumes idempotent
    applySavedCamSettings();
    DBG_PRINTLN("Camera enabled.");
  }
  return true;
}

bool disableCamera() {
  if (cameraInitialized) {
    esp_camera_deinit();
    cameraInitialized = false;
    DBG_PRINTLN("Camera deinitialized.");
  }
  saveCameraPrefs(false);
  return true;
}

void loadCameraPrefs() {
    cameraEnabled = camEnablePrefs.getBool("enabled", true); // or false if you want default OFF
}

void saveCameraPrefs(bool enabled) {
    camEnablePrefs.putBool("enabled", enabled);
}



//-------------------------------Telemetry Logging Functions----------------------------------------------------------------

// Quick SD-present check that doesn't open extra files
static inline bool sdPresent() {
  SdLock lock;
  return SD.cardType() != CARD_NONE;
}

// Single place to flip the flag + persist + cleanup
static void setTelemetryEnabled(bool on) {
  tlmEnabled = on;
  uiPrefs.putBool("RecordTelemetry", tlmEnabled);
  if (!on) {
    telemetryBuffer.clear();       // drop any queued samples
    currentTelemetryFile = "";     // forget target file so we re-pick when re-enabled
  }
}


void resetCurrentSample() {
  currentSample.timestamp    = 0;
  currentSample.voltage      = NAN;
  currentSample.temp         = NAN;
  currentSample.charger      = NAN;
  currentSample.imu_euler_x  = NAN;
  currentSample.imu_euler_y  = NAN;
  currentSample.imu_euler_z  = NAN;
  currentSample.imu_mag_x    = NAN;
  currentSample.imu_mag_y    = NAN;
  currentSample.imu_mag_z    = NAN;
  currentSample.imu_temp     = NAN;
  currentSample.fps          = NAN;
}

void flushTelemetryBufferToSD_Auto() {
  static unsigned long lastFlushTime = 0;
  if (!tlmEnabled) return;  // Only log when enabled
  if (!sdPresent()) return;   // <- don't even try to open anything if card is ejected

  unsigned long now = millis();
  if (now - lastFlushTime < telemetrySampleInterval) return;
  lastFlushTime = now;

  // --- Set timestamp for this sample ---
  if (timeIsValid) {
    time_t nowT;
    time(&nowT);
    currentSample.timestamp = nowT; // UNIX time
  } else {
    currentSample.timestamp = millis() / 1000;
  }

  // --- Buffer currentSample for writing ---
  telemetryBuffer.push_back(currentSample);

  // === SD card is accessed from here on ===
  {
    SdLock lock;  // 🔒 Mutex lock for SD access

    // Ensure /telemetry folder exists
    if (!SD.exists("/telemetry")) {
      SD.mkdir("/telemetry");
    }

    // ---- Find the current file number ----
    int idx = 1;
    while (true) {
      String fname = "/telemetry/telemetry_";
      fname += (idx < 10 ? "0" : "");
      fname += String(idx);
      fname += ".csv";

      if (!SD.exists(fname)) {
        // Create new file with header
        File f = SD.open(fname, FILE_WRITE);
        if (f) {
          f.println(TELEMETRY_CSV_HEADER);
          f.close();
        }
        currentTelemetryFile = fname;
        break;
      } else {
        File f = SD.open(fname, FILE_READ);
        if (f && f.size() < TELEMETRY_FILE_MAX_SIZE) {
          currentTelemetryFile = fname;
          f.close();
          break;
        }
        if (f) f.close();
        idx++;
      }
    }

    // ---- If appending would exceed size, rotate to next file ----
    File f = SD.open(currentTelemetryFile, FILE_APPEND);
    if (!f) {
      DBG_PRINTF("Failed to open %s for appending!\n", currentTelemetryFile.c_str());
      // If the SD was pulled, stop hammering the card. We’ll resume when user re-enables.
      if (!sdPresent()) {
        setTelemetryEnabled(false);    // auto-disable to avoid repeated errors
      }      
      return;
    }
    size_t estimatedSize = telemetryBuffer.size() * sizeof(TelemetrySample);
    if (f.size() + estimatedSize > TELEMETRY_FILE_MAX_SIZE) {
      f.close();
      idx++;
      String nextFile = "/telemetry/telemetry_";
      nextFile += (idx < 10 ? "0" : "");
      nextFile += String(idx);
      nextFile += ".csv";
      File nf = SD.open(nextFile, FILE_WRITE);
      if (nf) {
        nf.println(TELEMETRY_CSV_HEADER);
        nf.close();
      }
      currentTelemetryFile = nextFile;
      f = SD.open(currentTelemetryFile, FILE_APPEND);
      if (!f) {
        DBG_PRINTF("Failed to open %s for appending!\n", currentTelemetryFile.c_str());
        return;
      }
    }

    // ---- Write all samples in the buffer ----
    for (const auto& s : telemetryBuffer) {
        if (timeIsValid) {
            time_t rawTime = (time_t)s.timestamp;
            struct tm * timeinfo = localtime(&rawTime);
            char datetime[24] = {0};
            if (timeinfo) {
                strftime(datetime, sizeof(datetime), "%Y-%m-%d %H:%M:%S", timeinfo);
                f.print(datetime);
            } else {
                f.print("");
            }
        } else {
            f.print("");
        }
        f.print(",");
        f.print(s.timestamp);    f.print(",");
        f.print(s.voltage, 2);   f.print(",");
        f.print(s.temp, 1);      f.print(",");
        f.print(s.charger, 2);   f.print(",");
        f.print(s.imu_euler_x, 2); f.print(",");
        f.print(s.imu_euler_y, 2); f.print(",");
        f.print(s.imu_euler_z, 2); f.print(",");
        f.print(s.imu_mag_x, 2);   f.print(",");
        f.print(s.imu_mag_y, 2);   f.print(",");
        f.print(s.imu_mag_z, 2);   f.print(",");
        f.print(s.imu_temp, 1);    f.print(",");
        f.println(s.fps, 1);
    }

    f.close();
  } // 🔓 Lock automatically released here

  telemetryBuffer.clear();
  resetCurrentSample();
}


//----------------------------------------------------------NPT Tine sync------------------------------------------------------------------------
// Call this after WiFi connects
void startNtpSync() {
    configTime(gmtOffset_sec, 0, "pool.ntp.org", "time.nist.gov");
    // Do not block here!
}

// Non blocking NPT time sync
void pollTimeValid() {
    if (timeIsValid) return;
    if (millis() - lastTimeCheck < 1000) return;
    lastTimeCheck = millis();

    struct tm tm;
    if (getLocalTime(&tm) && (tm.tm_year + 1900) >= 2023) {
        timeIsValid = true;
        DBG_PRINT("NTP time is valid: ");
        DBG_PRINTLN(asctime(&tm));
      sntp_set_time_sync_notification_cb([](struct timeval *tv) {
          // This callback is called when time is updated via SNTP
          // No-op here, but required for FATFS to set time
      });        
    }
}

//------------------------------------------------------------Robot Controls---------------------------------------------------------

//------------------------------------Lighting---------------------------------------
void updatePixels() {
    pixels.clear();

    // Highest priority: Emergency
    if (emergencyOn) {
        uint32_t col = blinkState ? pixels.Color(255, 180, 0) : 0;
        pixels.setPixelColor(0, col);
        pixels.setPixelColor(5, col);
        pixels.setPixelColor(6, col);
        pixels.setPixelColor(11, col);
        pixels.show();
        // Stop siren if it was running
        if (sirenPlaying) {
            stopAudio();
            sirenPlaying = false;
        }
        return;
    }

    // Beacon
    if (beaconOn) {
        static int phase = 0;
        static unsigned long lastStep = 0;
        const int stepDelay = 90;
        const int whiteFlashes = 3;
        // 3 blue fill + 3 red fill + 3 flashes + 3 white flashes = 12 phases
        const int phaseCount = 3 + 3 + 3 + whiteFlashes * 2;

        unsigned long now = millis();

        if (now - lastStep > stepDelay) {
            lastStep = now;
            phase++;
            if (phase >= phaseCount) phase = 0;
        }

        pixels.clear();

        // Define which indices are paired for both bars
        const int pairs[3][2] = { {0, 5}, {1, 4}, {2, 3} };

        // BLUE FORWARD FILL
        if (phase == 0) {
            // Only ends
            for (int b = 0; b < 2; b++) {
                pixels.setPixelColor(pairs[0][b], pixels.Color(0, 0, 255));
                pixels.setPixelColor(pairs[0][b]+6, pixels.Color(0, 0, 255));
            }
        } else if (phase == 1) {
            // Ends + next inner
            for (int step = 0; step < 2; step++)
                for (int b = 0; b < 2; b++) {
                    pixels.setPixelColor(pairs[step][b], pixels.Color(0, 0, 255));
                    pixels.setPixelColor(pairs[step][b]+6, pixels.Color(0, 0, 255));
                }
        } else if (phase == 2) {
            // All blue (ends + next inner + center)
            for (int step = 0; step < 3; step++)
                for (int b = 0; b < 2; b++) {
                    pixels.setPixelColor(pairs[step][b], pixels.Color(0, 0, 255));
                    pixels.setPixelColor(pairs[step][b]+6, pixels.Color(0, 0, 255));
                }
        }
        // RED REVERSE FILL
        else if (phase == 3) {
            // Only center
            for (int b = 0; b < 2; b++) {
                pixels.setPixelColor(pairs[2][b], pixels.Color(255, 0, 0));
                pixels.setPixelColor(pairs[2][b]+6, pixels.Color(255, 0, 0));
            }
        } else if (phase == 4) {
            // Center + next outer
            for (int step = 1; step < 3; step++)
                for (int b = 0; b < 2; b++) {
                    pixels.setPixelColor(pairs[step][b], pixels.Color(255, 0, 0));
                    pixels.setPixelColor(pairs[step][b]+6, pixels.Color(255, 0, 0));
                }
        } else if (phase == 5) {
            // All red (center + next outer + ends)
            for (int step = 0; step < 3; step++)
                for (int b = 0; b < 2; b++) {
                    pixels.setPixelColor(pairs[step][b], pixels.Color(255, 0, 0));
                    pixels.setPixelColor(pairs[step][b]+6, pixels.Color(255, 0, 0));
                }
        }
        // FLASHES: RED, BLUE, RED
        else if (phase == 6) {
            for (int i = 0; i < 12; i++) pixels.setPixelColor(i, pixels.Color(255,0,0));
        } else if (phase == 7) {
            for (int i = 0; i < 12; i++) pixels.setPixelColor(i, pixels.Color(0,0,255));
        } else if (phase == 8) {
            for (int i = 0; i < 12; i++) pixels.setPixelColor(i, pixels.Color(255,0,0));
        }
        // WHITE FLASHES: on/off alternately
        else if (phase >= 9 && phase < 9+whiteFlashes*2) {
            if ((phase-9)%2 == 0) {
                for (int i=0; i<12; i++) pixels.setPixelColor(i, pixels.Color(255,255,255));
            }
            // Odd phases: off (pixels.clear() already called)
        }

        pixels.show();

        // --- Siren control: loop while beacon is on ---
        if (!sirenPlaying) {
            playSystemSound("/web/pcm/siren.wav"); // loop if your function supports, else retrigger in handleAnimationTimers
            sirenPlaying = true;
        }
        return;
    } else if (sirenPlaying) {
        // Beacon just turned off, stop the siren
        stopAudio();
        sirenPlaying = false;
    }

    // Main LED white
    if (light) {
        for (int i = 0; i < NEO_COUNT; i++)
            pixels.setPixelColor(i, pixels.Color(255, 255, 255));
        pixels.show();
        return;
    }

    // Turn signals (blinkState toggles on/off)
    if (leftSignalActive && !rightSignalActive) {
        uint32_t col = blinkState ? pixels.Color(255, 180, 0) : 0;
        pixels.setPixelColor(0, col);
        pixels.setPixelColor(6, col);
        pixels.show();
        return;
    }
    if (rightSignalActive && !leftSignalActive) {
        uint32_t col = blinkState ? pixels.Color(255, 180, 0) : 0;
        pixels.setPixelColor(5, col);
        pixels.setPixelColor(11, col);
        pixels.show();
        return;
    }

    pixels.show();
}

void pixelStart(){
  pixels.begin();
  pixels.setBrightness(30);
  //pixels.setPixelColor(0, pixels.Color(255, 165, 0));  // Orange = Booting
  pixels.show();
}

void handleAnimationTimers() {
    static unsigned long lastAnimUpdate = 0;
    static unsigned long lastBlinkUpdate = 0;
    bool needsUpdate = false;

    // Beacon: chase phase update
    if (beaconOn && millis() - lastAnimUpdate > beaconInterval) {
        beaconPhase = (beaconPhase + 1) % 4;
        lastAnimUpdate = millis();
        needsUpdate = true;
    }

    // Emergency and turn signals: blink toggle update
    if ((emergencyOn || leftSignalActive || rightSignalActive) && millis() - lastBlinkUpdate > blinkInterval) {
        blinkState = !blinkState;
        lastBlinkUpdate = millis();
        needsUpdate = true;
    }

    if (needsUpdate) updatePixels();
}


//-----------------------------------------------------------------------OLED--------------------------------------------------------------------
/*screen is 128x64 I2C SSD1306 (“2-color” means 8 rows yellow, 56 blue; common for these displays).*/

void displayMessage(const String& line1 = "", const String& line2 = "", const String& line3 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);      // <--- CRITICAL LINE!

  if (line1 != "") {
    display.setCursor(0, 0);
    display.println(line1);
  }
  if (line2 != "") {
    display.setCursor(0, 16);
    display.println(line2);
  }
  if (line3 != "") {
    display.setCursor(0, 26);
    display.println(line3);
  }

  display.display();
}

void drawCenteredIP(const String& ip) {
  display.setTextColor(SSD1306_WHITE);
  display.fillRect(0, 0, 128, 16, SSD1306_BLACK); // Clear top
  int textSize = 2;
  int charW = 6 * textSize;
  int ipLen = ip.length();
  int maxChars = 128 / (charW); // e.g. 128/12 = 10

  if (ipLen > maxChars) textSize = 1, charW = 6;

  display.setTextSize(textSize);
  int x = (128 - ipLen * charW) / 2;
  if (x < 0) x = 0;
  display.setCursor(x, 0);
  display.print(ip);
  display.setTextSize(1); // Reset for other text
}

void showWiFiScreen(const String& ip, const String& body, int bodySize = 2) {
  display.clearDisplay();

  // Top row: IP, yellow color on 2-color OLED (SSD1306 lib: set first 8 rows)
  drawCenteredIP(ip);

  // Body: Wi-Fi status
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(bodySize);
  int bodyY = 18;
  display.setCursor(0, bodyY);

  if (bodySize == 2) {
    display.println(body); // Just one or two lines
  } else {
    // Split by \n for up to 3 lines
    int y = bodyY;
    int start = 0, idx;
    while ((idx = body.indexOf('\n', start)) != -1) {
      String line = body.substring(start, idx);
      display.setCursor(0, y);
      display.println(line);
      y += 8;
      start = idx + 1;
    }
    if (start < body.length()) {
      display.setCursor(0, y);
      display.println(body.substring(start));
    }
  }

  display.display();
}

void showWiFiStep(const String& msg, bool force = false) {
  String ipToShow = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "WiFi";
  int bodySize = (msg.length() < 18) ? 2 : 1;
  showWiFiScreen(ipToShow, msg, bodySize);
  wifiOledLastStep = msg;
  wifiOledLastMsg = millis();
}

void showWiFiProgress(int percent) {
  String ipToShow = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "WiFi";
  display.clearDisplay();
  drawCenteredIP(ipToShow);

  display.setCursor(0, 20);
  display.setTextSize(1);
  display.println("Connecting...");

  display.drawRect(0, 35, 128, 10, SSD1306_WHITE);
  int barWidth = map(percent, 0, 100, 0, 124);
  display.fillRect(2, 37, barWidth, 6, SSD1306_WHITE);

  display.display();
}

void showCheckmark() {
  display.clearDisplay();
  drawCenteredIP(WiFi.localIP().toString());
  display.setTextSize(2);
  display.setCursor(30, 25);
  display.println("✔");
  display.display();
  delay(800);
}

void drawGear(int cx, int cy, int r, int teeth, float angle) {
  float toothWidth = PI / teeth;
  float outerR = r;
  float innerR = r - 3;

  for (int i = 0; i < teeth; i++) {
    float a = angle + i * 2 * PI / teeth;

    float ax = cx + cos(a) * innerR;
    float ay = cy + sin(a) * innerR;
    float bx = cx + cos(a + toothWidth / 2) * outerR;
    float by = cy + sin(a + toothWidth / 2) * outerR;
    float cx2 = cx + cos(a + toothWidth) * innerR;
    float cy2 = cy + sin(a + toothWidth) * innerR;

    // Draw tooth triangle
    display.drawLine(ax, ay, bx, by, SSD1306_WHITE);
    display.drawLine(bx, by, cx2, cy2, SSD1306_WHITE);
  }

  // Optional: draw gear hub
  display.drawCircle(cx, cy, 2, SSD1306_WHITE);
}

void drawBatteryBar(int x, int y, int width, int height, int percent, bool charging) {
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.drawRect(x + width, y + height / 3, 2, height / 3, SSD1306_WHITE); // tip

  int fillHeight = map(percent, 0, 100, 0, height - 2);
  display.fillRect(x + 1, y + height - 1 - fillHeight, width - 2, fillHeight, SSD1306_WHITE);

  if (charging) {
    // Simple animated lightning bolt
    static bool flash = false;
    flash = !flash;

    if (flash) {
      display.setCursor(x + 1, y - 8);
      display.setTextSize(1);
      display.print("⚡");  // Alternatively: use custom shape if ⚡ doesn't render
    }
  }
}

void drawWiFiBars(int x, int y, int quality) {
  int levels = map(quality, 0, 100, 0, 4); // 0 to 4 bars
  int barWidth = 3, spacing = 2;
  
  for (int i = 0; i < 4; i++) {
    int barHeight = (i + 1) * 4;
    int bx = x + i * (barWidth + spacing);
    int by = y + 16 - barHeight;
    if (i < levels) {
      display.fillRect(bx, by, barWidth, barHeight, SSD1306_WHITE);
    } else {
      display.drawRect(bx, by, barWidth, barHeight, SSD1306_WHITE);
    }
  }
}

void drawWebSocketStatus() {
  if (lastWsClientIP != "" && millis() - lastWsConnectTime < 15000) {
    String wsText = "WS: " + lastWsClientIP;
    //playSystemSound("/web/pcm/click.wav");
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(wsText, 0, 0, &x1, &y1, &w, &h);

    if (w < 128) {
      display.setCursor((128 - w) / 2, 56);  // bottom center
      display.print(wsText);
    } else {
      static int scrollX = 0;
      scrollX = (scrollX + 2) % (w + 128);
      display.setCursor(128 - scrollX, 56);
      display.print(wsText);
    }
  }
}

void animateGears() {
  static float angle1 = 0;
  const int r1 = 10, r2 = 15;
  const int teeth1 = 8, teeth2 = 12;
  const int x1 = 50, y1 = 32;
  const int x2 = x1 + r1 + r2 - 1;

  // Load user OLED settings (no SD lock needed here - Preferences is thread-safe)
  String oledLayout = oledPrefs.getString("layout", "default");
  bool showIP       = oledPrefs.getBool("showIP", true);
  bool showBattery  = oledPrefs.getBool("showBattery", true);
  bool showWiFi     = oledPrefs.getBool("showWiFi", true);

  display.clearDisplay();

  // Show IP at top center (if enabled)
  if (showIP) {
    String ipStr = (WiFi.status() == WL_CONNECTED)
                   ? WiFi.localIP().toString()
                   : "WiFi";
    drawCenteredIP(ipStr);
  }

  // AP mode indicator (top-left)
  if (WiFi.getMode() == WIFI_AP && WiFi.status() != WL_CONNECTED) {
    drawAPIndicator(0, 0);
  }

  // Apply layout logic
  if (oledLayout == "default") {
    // Gears + battery + Wi-Fi bars
    if (showBattery)
      drawBatteryBar(0, 20, 8, 24, batteryPercentDisplay, isCharging);

    if (showWiFi)
      drawWiFiBars(110, 20, wifiSignalStrength);

    // Gears animation
    drawGear(x1, y1, r1, teeth1, angle1);
    drawGear(x2, y1, r2, teeth2, -angle1 * ((float)teeth1 / teeth2) + PI / teeth2);
    angle1 += 0.2f;

  } else if (oledLayout == "text") {
    // Just show centered message
    displayMessage("MiniExco", "System Ready", "");

  } else if (oledLayout == "animation") {
    // Try to load a frame from SD and show it
    static int frame = 0;
    char filename[32];
    sprintf(filename, "/oled_anim/frame_%03d.raw", frame);

    {
      SdLock lock; // 🔒 Protect SD access
      File f = SD.open(filename, FILE_READ);
      if (f) {
        // ---- Allocate buffer dynamically in PSRAM if enabled ----
        uint8_t* buffer = nullptr;
        #if USE_PSRAM
          buffer = (uint8_t*)ps_malloc(2048);
        #else
          buffer = (uint8_t*)malloc(1024);
        #endif
        if (buffer) {
          f.read(buffer, 1024);
          f.close();
          display.drawBitmap(0, 0, buffer, 128, 64, SSD1306_WHITE);
          frame = (frame + 1) % TOTAL_OLED_FRAMES;
          free(buffer);  // Always free after use!
        } else {
          displayMessage("⚠️ Animation", "buffer error", "");
          f.close();
        }
      } else {
        displayMessage("⚠️ Animation", "frame missing", "");
      }
    } // 🔓 Lock released here
  }

  drawWebSocketStatus();
  display.display();
}


void drawAPIndicator(int x, int y) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print("AP");
}

//---------------------------------------------------------------I2C---------------------------------------------------------------------------------------

void i2cStart(){

  Wire.begin(BNO_SDA, BNO_SCL);  // I2C pins
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DBG_PRINTLN(F("❌ OLED init failed"));
  } else {
    displayMessage("", "🔋 MiniExco Booting", "Please wait");
  }

}

void initBNO055() {
  // Start I2C bus for BNO055 (custom pins if needed)
  Wire.begin(BNO_SDA, BNO_SCL);

  DBG_PRINT("🔄 Initializing BNO055 IMU... ");
  if (!bno055.begin()) {
    DBG_PRINTLN("❌ BNO055 not detected! Check wiring/address.");
    while (1) { delay(1000); }  // Stay here to indicate error
  }
  DBG_PRINTLN("✅ Found BNO055!");

  // Use external 32kHz crystal for better stability (recommended!)
  bno055.setExtCrystalUse(true);

  // (Optional) Set operating mode, axis remap, etc, if needed
  delay(10);
}

void spiStart() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  bool ok = false;

  // --- Prefer new-style config if available (Arduino-ESP32 2.x/3.x) ---
  // SDFSConfig lets us set MaxOpenFiles; header is provided by SD.h in new cores.
  #if defined(SDFS_CONFIG_H) || defined(ARDUINO_ESP32_RELEASE_3_0_0) || defined(ARDUINO_ESP32_RELEASE_2_0_0)
  {
    SDFSConfig cfg;
    cfg.setSPI(SPI);
    cfg.setCSPin(SD_CS);
    cfg.setMaxOpenFiles(24);               // <-- raise from default 5
    cfg.setAllocationUnitSize(16 * 1024);  // optional; keeps FAT happy

    ok = SD.begin(cfg);
  }
  #else
  // --- Fallback: older core signature supports max_files as 5th arg ---
  // SD.begin(cs, spi, freq, mountpoint, max_files)
  {
    ok = SD.begin(SD_CS, SPI, 40000000U, "/sd", 24);   // <-- raise to 24
  }
  #endif

  if (!ok) {
    DBG_PRINTLN("SD Card initialization failed!");
    return;
  }

  DBG_PRINTLN("SD Card initialized.");
  // SD.setTimeCallback(getFatTime); // if you use timestamps

  // Helpers to make dirs safely
  auto ensureDir = [](const char* p) {
    SdLock lock;
    if (!SD.exists(p)) {
      SD.mkdir(p);
      DBG_PRINTF("Created %s folder on SD card.\n", p);
    }
  };

  // Base folders
  ensureDir("/media");
  ensureDir("/web");
  ensureDir("/web/pcm");
  ensureDir("/firmware");
  ensureDir("/telemetry");

  // Media subfolders
  const char* mediaFolders[] = {"/capture", "/wav", "/video", "/mp3", "/anim"};
  for (size_t i = 0; i < sizeof(mediaFolders) / sizeof(mediaFolders[0]); ++i) {
    String sub = String("/media") + mediaFolders[i];
    ensureDir(sub.c_str());
  }
}


//------------------------------------------------------------WiFi Stack Management-----------------------------------------------------------------

bool connectToWiFiWithRetries(const String& ssid, const String& password, int retries) {
  WiFi.begin(ssid.c_str(), password.c_str());

  for (int attempt = 1; attempt <= retries; attempt++) {
      int progress = map(attempt - 1, 0, retries, 0, 100);
      showWiFiProgress(progress); // <-- add this line

      DBG_PRINTF("🔁 Attempt %d to connect to WiFi SSID: %s\n", attempt, ssid.c_str());
      unsigned long startAttemptTime = millis();

      while (millis() - startAttemptTime < 2000) {
        if (WiFi.status() == WL_CONNECTED) {
          DBG_PRINTLN("✅ Connected to WiFi!");
          displayMessage("", "✅ WiFi Connected\n" + WiFi.localIP().toString());
          delay(500); // Small delay before switching to checkmark
          showCheckmark(); // <-- optional: add checkmark here
          return true;
        }
        delay(200);
      }

      WiFi.disconnect();  // Reset for retry
      delay(100);
      WiFi.begin(ssid.c_str(), password.c_str());
  }


  DBG_PRINTLN("❌ Failed to connect after retries.");
  return false;
}

void handleWiFiSetup(AsyncWebServerRequest *request) {
  if (SD.exists("/WiFiPages.html")) {
    request->send(SD, "/WiFiPages.html", "text/html");
  } else {
    request->send(404, "text/plain", "WiFiPages.html not found on SD card");
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

void handleSaveWiFi(AsyncWebServerRequest *request) {
  if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
    String ssid = normalizedSSID(request->getParam("ssid", true)->value());
    String password = request->getParam("password", true)->value();
    if (request->hasParam("aRt", true)) {
      bool autoReconnect = request->getParam("aRt", true)->value() == "1";
      wifiPrefs.putBool(("aRt_" + ssid).c_str(), autoReconnect);
    }
   
    wifiPrefs.putString("ssid", ssid);
    wifiPrefs.putString("password", password);
    wifiPrefs.putString(("wifi_" + ssid).c_str(), password);

    String existingList = wifiPrefs.getString("networks", "");
    if (existingList.indexOf(ssid) == -1) {
      existingList += (existingList.length() > 0 ? "," : "") + ssid;
      wifiPrefs.putString("networks", existingList);
    }

    if (SD.exists("/wifi_success.html")) {
      request->send(SD, "/wifi_success.html", "text/html");
    } else {
      request->send(200, "text/html", "<html><body><h1>✅ Wi-Fi Credentials Saved</h1><p>Rebooting and trying to connect...</p></body></html>");
    }

    delay(3000);
    ESP.restart();
  } else {
    request->send(400, "text/html", "<html><body><h1>Missing SSID or Password</h1></body></html>");
  }
}

// 👇 Helper: Stop server + WebSocket cleanly before Wi-Fi switch
void stopWebServerAndWS() {
    wsCarInput.closeAll();  // Close all websocket clients
    server.end();           // Fully stop the HTTP server
    DBG_PRINTLN("✅ WebServer + WebSocket stopped.");
}

void webServerReboot() {
    Serial.println("[WebServer] Forcibly restarting AsyncWebServer stack...");
    stopWebServerAndWS();   // Your function to stop/end server and WS
    delay(100);
    serverStart();          // Or startWebServer(), whatever re-adds handlers and begins server
    Serial.println("[WebServer] AsyncWebServer stack restarted.");
}


void handleWifiStateMachine() {
  unsigned long now = millis();

  // Periodic status check (every 1s)
  if (now - wifiLastCheck > 1000) {
    wifiLastCheck = now;

    if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
      if (wifiState != WIFI_STA_CONNECTED) {
        showWiFiStep("Connected:\n" + WiFi.localIP().toString(), true);
        wifiState = WIFI_STA_CONNECTED;
        wifiRetrySubState = 0;
      }
    } else if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) {
      if (wifiState != WIFI_STA_RETRYING) {
        showWiFiStep("Wi-Fi lost!\nRetrying...");
        wifiState = WIFI_STA_RETRYING;
        wifiRetryStart = now;
        wifiAutoRetryCount = 0;
        wifiRetrySubState = 0;
      }
    }
  }

  // STA retry logic (non-blocking)
  if (wifiState == WIFI_STA_RETRYING) {
    switch (wifiRetrySubState) {
      case 0: // Start retry
        if (wifiAutoRetryCount < 3) {
          showWiFiStep("Retry STA...\n(" + String(wifiAutoRetryCount+1) + "/3)");
          WiFi.disconnect(true, true);
          wifiRetryStepTime = now;
          wifiRetrySubState = 1;
        } else {
          // All retries failed, go AP
          showWiFiStep("No Wi-Fi.\nAP Mode...");
          WiFi.disconnect(true, true);
          wifiRetryStepTime = now;
          wifiRetrySubState = 10;
        }
        break;
      case 1: // Wait 100ms, then start STA
        if (now - wifiRetryStepTime > 100) {
          WiFi.mode(WIFI_STA);
          WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
          wifiAutoRetryCount++;
          wifiRetryStepTime = now;
          wifiRetrySubState = 2;
        }
        break;
      case 2: // Wait 500ms, then next retry
        if (now - wifiRetryStepTime > 500) {
          wifiRetrySubState = 0;
        }
        break;
      case 10: // Wait 100ms, then go AP mode
        if (now - wifiRetryStepTime > 100) {
          WiFi.mode(WIFI_AP);
          WiFi.softAP(ap_ssid);
          showWiFiStep("AP:\n" + WiFi.softAPIP().toString());
          wifiState = WIFI_AP_MODE;
          wifiOledLastStep = "";
          wifiApScanTime = now;
          wifiRetrySubState = 0;
        }
        break;
    }
  }

  // AP mode: scan for known networks every 15s (non-blocking)
  if (wifiState == WIFI_AP_MODE && now - wifiApScanTime > 15000) {
    wifiApScanTime = now;
    showWiFiStep("Scanning for\nknown Wi-Fi...");
    int n = WiFi.scanNetworks();
    String savedList = wifiPrefs.getString("networks", "");
    int lastIndex = 0, nextIndex;
    bool found = false;
    String foundSSID, foundPass;
    while ((nextIndex = savedList.indexOf(',', lastIndex)) != -1) {
      String ssid = normalizedSSID(savedList.substring(lastIndex, nextIndex));
      lastIndex = nextIndex + 1;
      bool autoReconnect = wifiPrefs.getBool(("aRt_" + ssid).c_str(), true);
      if (!autoReconnect) continue;
      for (int i = 0; i < n; i++) {
        if (ssid == WiFi.SSID(i)) {
          found = true;
          foundSSID = ssid;
          foundPass = wifiPrefs.getString(("wifi_" + ssid).c_str(), "");
          break;
        }
      }
      if (found) break;
    }
    if (!found && lastIndex < savedList.length()) {
      String ssid = normalizedSSID(savedList.substring(lastIndex));
      bool autoReconnect = wifiPrefs.getBool(("aRt_" + ssid).c_str(), true);
      if (autoReconnect) {
        for (int i = 0; i < n; i++) {
          if (ssid == WiFi.SSID(i)) {
            found = true;
            foundSSID = ssid;
            foundPass = wifiPrefs.getString(("wifi_" + ssid).c_str(), "");
            break;
          }
        }
      }
    }
    if (found && foundSSID.length()) {
      showWiFiStep("Found known AP:\n" + foundSSID + "\nConnecting...");
      WiFi.disconnect(true, true);
      wifiSSID = foundSSID;
      wifiPassword = foundPass;
      wifiState = WIFI_STA_CONNECTING;
      WiFi.mode(WIFI_STA);
      WiFi.begin(foundSSID.c_str(), foundPass.c_str());
      wifiRetryStart = now;
      wifiAutoRetryCount = 0;
      wifiRetrySubState = 0;
    } else {
      showWiFiStep("No known Wi-Fi\nStill in AP");
    }
  }
}

//------------------------------------------------------------SD File Management----------------------------------------------------------------------------

void handleRoot(AsyncWebServerRequest *request) {
  SdLock lock; // 🔒 Protect all SD access in this function

  // 1. Check /web/index.html (new standard location)
  if (SD.exists("/web/index.html")) {
    request->send(SD, "/web/index.html", "text/html");
    return;
  }

  // 2. Fallback: check /index.html (legacy location)
  if (SD.exists("/index.html")) {
    request->send(SD, "/index.html", "text/html");
    return;
  }

  // 3. Nothing found, list SD files
  String message = "index.html not found on SD card.\n\nFiles on SD card:\n";
  File root = SD.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      message += String(file.name()) + "\n";
      file = root.openNextFile();
    }
    root.close();
  } else {
    message += "(Failed to open SD root)\n";
  }

  request->send(404, "text/plain", message);
}


String ensureUniqueFilename(String path) {
  if (!SD.exists(path.c_str())) return path;

  String base = path;
  String ext = "";
  int dot = path.lastIndexOf('.');
  if (dot > 0) {
    base = path.substring(0, dot);
    ext = path.substring(dot);
  }
  int i = 1;
  while (true) {
    String tryName = base + "(" + String(i) + ")" + ext;
    if (!SD.exists(tryName.c_str())) return tryName;
    i++;
  }
}

void ensureFolderExists(const String& fullPath) {
  int lastSlash = fullPath.lastIndexOf('/');
  if (lastSlash > 0) {
    String folderPath = fullPath.substring(0, lastSlash);
    if (!SD.exists(folderPath)) SD.mkdir(folderPath.c_str());
  }
}

void processReindexTask() {
    static File dir, idx;
    static File f;
    static bool init = false;
    static unsigned lastBatch = 0;
    static File tmpDir; // For counting

    if (!pendingReindex) return;

    SdLock lock; // 🔒 All SD access is now thread-safe

    if (!init) {
        // First, count files (async-friendly)
        reindexCounting = true;
        reindexReadyToIndex = false;
        reindexTotal = 0;

        if (tmpDir) tmpDir.close();
        tmpDir = SD.open(reindexPath);
        if (!tmpDir || !tmpDir.isDirectory()) {
            pendingReindex = false;
            DBG_PRINTLN("Failed to open dir for count!");
            reindexCounting = false;
            return;
        }
        File tf = tmpDir.openNextFile();
        while (tf) {
            String name = String(tf.name());
            if (!name.equalsIgnoreCase("System Volume Information") && name != "Thumbs.db") {
                reindexTotal++;
            }
            tf.close();
            tf = tmpDir.openNextFile();
        }
        tmpDir.close();
        DBG_PRINTF("Counted %d files for indexing in %s\n", reindexTotal, reindexPath.c_str());
        reindexCounting = false;
        reindexReadyToIndex = true;
        init = true; // Continue to actual indexing on next call
        return;
    }

    // Only start indexing after counting done
    if (reindexReadyToIndex && init) {
        if (dir) dir.close();
        if (idx) idx.close();
        dir = SD.open(reindexPath);
        idx = SD.open(reindexPath + "/.index", FILE_WRITE);
        if (!dir || !dir.isDirectory() || !idx) {
            pendingReindex = false;
            DBG_PRINTLN("Failed to start reindex!");
            return;
        }
        f = dir.openNextFile();
        reindexCount = 0;
        reindexReadyToIndex = false; // Reset so doesn't re-enter
        DBG_PRINTF("Begin background indexing for %s\n", reindexPath.c_str());
    }

    if (!dir || !idx) return; // Not ready yet

    const int BATCH_SIZE = 50;
    int batchCount = 0;

    while (f && batchCount < BATCH_SIZE) {
        String name = String(f.name());
        if (name.equalsIgnoreCase("System Volume Information") || name == "Thumbs.db") { 
            f.close(); 
            f = dir.openNextFile(); 
            continue; 
        }
        idx.printf("%s,%d,%u,%lu\n",
                   name.c_str(),
                   f.isDirectory() ? 1 : 0,
                   (unsigned)f.size(),
                   (unsigned long)f.getLastWrite());
        f.close();
        f = dir.openNextFile();
        batchCount++;
        reindexCount++;
    }

    if (!f) {
        if (reindexCount == 0) {
            idx.println("__EMPTY__");
        }
        idx.close();
        dir.close();
        pendingReindex = false;
        init = false;
        DBG_PRINTF("Index for %s finished, %d files\n", reindexPath.c_str(), reindexCount);
    }
}

// Utility: read a batch from .index file
void readSdIndexBatch(const String& idxPath, int start, int count, JsonArray& arr, bool showSystem /*= false*/) {
  if (start < 0) start = 0;
  if (count < 1) count = 1;

  SdLock lock;                              // safe with recursive mutex
  File idx = SD.open(idxPath, FILE_READ);
  if (!idx) return;

  delay(0);                                 // initial yield after open

  int idxLine = 0;
  int added   = 0;
  uint32_t tick = 0;
  String line;
  bool isEmptyMarker = false;

  while (idx.available()) {
    line = idx.readStringUntil('\n');
    line.trim();

    if (line == "__EMPTY__") {               // fast exit if index marks empty dir
      isEmptyMarker = true;
      break;
    }

    // Skip until we reach the requested start line
    if (idxLine++ < start) {
      if (((++tick) & 0x7F) == 0) delay(0); // yield ~every 128 iterations
      continue;
    }

    // --- parse: name,isFolder,size[,date] ---
    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);
    int comma3 = line.indexOf(',', comma2 + 1); // optional date

    if (comma1 < 0 || comma2 < 0) {          // malformed line
      if (((++tick) & 0x7F) == 0) delay(0);
      continue;
    }

    String name = line.substring(0, comma1);
    bool isFolder = line.substring(comma1 + 1, comma2).toInt();

    uint32_t size = 0;
    uint32_t date = 0;
    if (comma3 > 0) {
      size = line.substring(comma2 + 1, comma3).toInt();
      date = line.substring(comma3 + 1).toInt();
    } else {
      size = line.substring(comma2 + 1).toInt();
    }

    // --- filters ---
    if (!showSystem && (
        name.endsWith(".path") ||
        name.endsWith(".bak")  ||
        name.endsWith(".meta") ||
        name.startsWith(".")   ||
        name.startsWith(".csv")||
        name.equalsIgnoreCase("System Volume Information") ||
        name.startsWith("FOUND.") ||
        name == "Thumbs.db"
    )) {
      if (((++tick) & 0x7F) == 0) delay(0);
      continue;
    }

    // Pagination: stop once we’ve added 'count' items
    if (added >= count) break;

    // Build JSON entry
    JsonObject obj = arr.createNestedObject();
    obj["name"]     = name;
    obj["isFolder"] = isFolder;
    if (!isFolder) obj["size"] = size;
    obj["type"]     = isFolder ? "folder" : "default";
    if (date > 0)   obj["date"] = (uint32_t)date * 1000; // seconds -> ms

    added++;

    if (((++tick) & 0x7F) == 0) delay(0);     // periodic yield to feed WDT
  }

  idx.close();
  delay(0);                                   // final cooperative yield

  // If __EMPTY__ marker encountered and nothing was added, leave arr empty
  if (isEmptyMarker && arr.size() == 0) {
    // intentionally empty
  }
}


// helpers to Fix upload corruption-----

// ---------- SD helpers ----------
static String baseName(const String& path) {
  int s = path.lastIndexOf('/'); return (s >= 0) ? path.substring(s + 1) : path;
}
static String dirName(const String& path) {
  int s = path.lastIndexOf('/'); return (s >= 0) ? path.substring(0, s) : String("/");
}
static String uniqueInDir(const String& dir, const String& wantName) {
  String base = wantName, ext;
  int d = wantName.lastIndexOf('.');
  if (d >= 0) { base = wantName.substring(0, d); ext = wantName.substring(d); }
  String test = dir + "/" + wantName; int n = 1;
  while (SD.exists(test)) test = dir + "/" + base + "_" + String(n++) + ext;
  return test;
}
static inline void sdTinyYield(uint32_t ms = 2) {
#if defined(ESP32)
  vTaskDelay(pdMS_TO_TICKS(ms));
#else
  delay(ms);
#endif
}

// ---------- HEX helpers ----------
static String toHexLower(const uint8_t* p, size_t n) {
  static const char* hexd = "0123456789abcdef";
  String s; s.reserve(n*2);
  for (size_t i=0;i<n;i++){ s += hexd[p[i]>>4]; s += hexd[p[i]&0x0F]; }
  return s;
}
static String normHex64(String s) { s.trim(); s.toLowerCase(); return s; }

// ---------- Boot cleanup: delete orphan temp files ----------
static void removeOrphanTempsInDir(const String& path) {
  File dir; { SdLock lock; dir = SD.open(path); }
  if (!dir) return;

  while (true) {
    File f; { SdLock lock; f = dir.openNextFile(); }
    if (!f) break;

    String name = f.name();
    bool isDir = f.isDirectory();
    f.close();                  // close before any yield

    if (isDir) {
      if (name != "." && name != "..") {
        // Recurse into subdir
        removeOrphanTempsInDir(path + "/" + name);
      }
    } else {
      if (name.endsWith(".upload.tmp")) {
        String doomed = path + "/" + name;
        { SdLock lock; SD.remove(doomed); }   // lock only around SD op
        DBG_PRINTF("[CLEAN] removed orphan temp: %s\n", doomed.c_str());
      }
    }

    // Give the scheduler a breath after each entry (no SD lock held)
#if defined(ESP32)
    vTaskDelay(1);
#else
    yield();
#endif
  }

  dir.close();
}


static void cleanOrphanTempUploads() { removeOrphanTempsInDir("/"); }


//------------------------------------------------------------------System Sound----------------------------------------------------------------------------

void playSystemSound(const char* filename) {
    if (!sSndEnabled || !filename || !filename[0]) return;

    unsigned long now = millis();
    // Ignore if file is same and within debounce period
    if (lastPlayedFile == filename && (now - lastPlayedTime < soundRepeatDelay)) return;

    // If something is playing, enqueue the request if there's space
    if (isSystemSoundPlaying) {
        // Add to queue only if not a duplicate in the queue
        for (uint8_t i = queueHead; i != queueTail; i = (i + 1) % MAX_SYSTEM_SOUND_QUEUE) {
            if (systemSoundQueue[i] == filename) return; // Already in queue
        }
        uint8_t nextTail = (queueTail + 1) % MAX_SYSTEM_SOUND_QUEUE;
        if (nextTail != queueHead) { // Queue not full
            systemSoundQueue[queueTail] = filename;
            queueTail = nextTail;
        }
        return;
    }

    // Not playing, start playing this one
    sSndVolume = constrain(sSndVolume, 0, 21);
    audio.setVolume(sSndVolume);
    isSystemSoundPlaying = true;
    lastPlayedFile = filename;
    lastPlayedTime = now;
    playWavFileOnSpeaker(filename); // This should be non-blocking
}

void onSystemSoundFinished() {
    isSystemSoundPlaying = false;
    // Play next in queue if available
    if (queueHead != queueTail) {
        String nextFile = systemSoundQueue[queueHead];
        queueHead = (queueHead + 1) % MAX_SYSTEM_SOUND_QUEUE;
        playSystemSound(nextFile.c_str());
    }
}

//------------------------------------------------------------------Robot Controls----------------------------------------------------------------------------

void rightMotorForward(uint8_t pwm = 255) {
  ledcWrite(CH_RIGHT_MOTOR_IN1, pwm);
  ledcWrite(CH_RIGHT_MOTOR_IN2, 0);
}
void rightMotorBackward(uint8_t pwm = 255) {
  ledcWrite(CH_RIGHT_MOTOR_IN1, 0);
  ledcWrite(CH_RIGHT_MOTOR_IN2, pwm);
}
void rightMotorStop() {
  ledcWrite(CH_RIGHT_MOTOR_IN1, 0);
  ledcWrite(CH_RIGHT_MOTOR_IN2, 0);
}
void leftMotorForward(uint8_t pwm = 255) {
  ledcWrite(CH_LEFT_MOTOR_IN1, pwm);
  ledcWrite(CH_LEFT_MOTOR_IN2, 0);
}
void leftMotorBackward(uint8_t pwm = 255) {
  ledcWrite(CH_LEFT_MOTOR_IN1, 0);
  ledcWrite(CH_LEFT_MOTOR_IN2, pwm);
}
void leftMotorStop() {
  ledcWrite(CH_LEFT_MOTOR_IN1, 0);
  ledcWrite(CH_LEFT_MOTOR_IN2, 0);
}
void armMotorUp(uint8_t pwm = 255) {
  ledcWrite(CH_ARM_MOTOR_IN1, pwm);
  ledcWrite(CH_ARM_MOTOR_IN2, 0);
}
void armMotorDown(uint8_t pwm = 255) {
  ledcWrite(CH_ARM_MOTOR_IN1, 0);
  ledcWrite(CH_ARM_MOTOR_IN2, pwm);
}
void armMotorStop() {
  ledcWrite(CH_ARM_MOTOR_IN1, 0);
  ledcWrite(CH_ARM_MOTOR_IN2, 0);
}

void moveCar(int inputValue, int pwm = 255) {
  if (!horScreen) { // Normal orientation
    switch (inputValue) {
      case UP:
        rightMotorForward(pwm);
        leftMotorForward(pwm);
        break;
      case DOWN:
        rightMotorBackward(pwm);
        leftMotorBackward(pwm);
        break;
      case LEFT:
        rightMotorForward(pwm);
        leftMotorBackward(pwm);
        break;
      case RIGHT:
        rightMotorBackward(pwm);
        leftMotorForward(pwm);
        break;
      case STOP:
        rightMotorStop();
        leftMotorStop();
        armMotorStop();
        break;
      case ARMUP:
        armMotorUp(pwm);
        break;
      case ARMDOWN:
        armMotorDown(pwm);
        break;
      default:
        rightMotorStop();
        leftMotorStop();
        armMotorStop();
        break;
    }
  } else { // Horizontal/rotated screen (swap both axes!)
    switch (inputValue) {
      case UP:      // Rotate UP -> RIGHT
        rightMotorBackward(pwm);
        leftMotorForward(pwm);
        break;
      case DOWN:    // Rotate DOWN -> LEFT
        rightMotorForward(pwm);
        leftMotorBackward(pwm);
        break;
      case LEFT:    // Rotate LEFT -> DOWN
        rightMotorBackward(pwm);
        leftMotorBackward(pwm);
        break;
      case RIGHT:   // Rotate RIGHT -> UP
        rightMotorForward(pwm);
        leftMotorForward(pwm);
        break;
      case STOP:
        rightMotorStop();
        leftMotorStop();
        armMotorStop();
        break;
      case ARMUP:
        armMotorUp(pwm);
        break;
      case ARMDOWN:
        armMotorDown(pwm);
        break;
      default:
        rightMotorStop();
        leftMotorStop();
        armMotorStop();
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
    DBG_PRINTLN("🪣 Bucket PWM configured (low-speed)");
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
    DBG_PRINTLN("🔧 AUX PWM configured (low-speed)");
  }

  writeServo((ledc_channel_t)CH_AUX_SERVO, auxServoValue);

  lastAuxValue = auxServoValue;
  preferences.putInt("auxAngle", lastAuxValue);
  if (!holdAux) auxDetachTime = millis() + 300;
}

void setUpPinModes() {
  const int pwmFreq = 1000;
  const int pwmRes = 8;

  // --- RIGHT motor ---
  pinMode(PIN_RIGHT_MOTOR_IN1, OUTPUT); 
  pinMode(PIN_RIGHT_MOTOR_IN2, OUTPUT);
  ledcSetup(CH_RIGHT_MOTOR_IN1, pwmFreq, pwmRes);
  ledcAttachPin(PIN_RIGHT_MOTOR_IN1, CH_RIGHT_MOTOR_IN1);
  ledcSetup(CH_RIGHT_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(PIN_RIGHT_MOTOR_IN2, CH_RIGHT_MOTOR_IN2);

  // --- LEFT motor ---
  pinMode(PIN_LEFT_MOTOR_IN1, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_IN2, OUTPUT);
  ledcSetup(CH_LEFT_MOTOR_IN1, pwmFreq, pwmRes);
  ledcAttachPin(PIN_LEFT_MOTOR_IN1, CH_LEFT_MOTOR_IN1);
  ledcSetup(CH_LEFT_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(PIN_LEFT_MOTOR_IN2, CH_LEFT_MOTOR_IN2);

  // --- ARM motor ---
  pinMode(PIN_ARM_MOTOR_IN1, OUTPUT);
  pinMode(PIN_ARM_MOTOR_IN2, OUTPUT);
  ledcSetup(CH_ARM_MOTOR_IN1, pwmFreq, pwmRes);
  ledcAttachPin(PIN_ARM_MOTOR_IN1, CH_ARM_MOTOR_IN1);
  ledcSetup(CH_ARM_MOTOR_IN2, pwmFreq, pwmRes);
  ledcAttachPin(PIN_ARM_MOTOR_IN2, CH_ARM_MOTOR_IN2);

  // --- Force stop all motors at start ---
  ledcWrite(CH_RIGHT_MOTOR_IN1, 0);
  ledcWrite(CH_RIGHT_MOTOR_IN2, 0);
  ledcWrite(CH_LEFT_MOTOR_IN1, 0);
  ledcWrite(CH_LEFT_MOTOR_IN2, 0);
  ledcWrite(CH_ARM_MOTOR_IN1, 0);
  ledcWrite(CH_ARM_MOTOR_IN2, 0);

  // Optionally also drive pins LOW in case FETs "leak":
  digitalWrite(PIN_RIGHT_MOTOR_IN1, LOW);
  digitalWrite(PIN_RIGHT_MOTOR_IN2, LOW);
  digitalWrite(PIN_LEFT_MOTOR_IN1, LOW);
  digitalWrite(PIN_LEFT_MOTOR_IN2, LOW);
  digitalWrite(PIN_ARM_MOTOR_IN1, LOW);
  digitalWrite(PIN_ARM_MOTOR_IN2, LOW);
}

void controlMotorByDirection(const std::string& dir, int speed) {
  // Normalize speed
  int pwm = abs(speed);

  // Handle stopping
  if (pwm == 0) {
    if (dir == "Arm" || dir == "ArmUp" || dir == "ArmDown") {
      armMotorStop();
    } else {
      rightMotorStop();
      leftMotorStop();
    }
    return;
  }

  // Drive logic
  if (dir == "Forward") {
    rightMotorForward(pwm);
    leftMotorForward(pwm);

  } else if (dir == "Backward") {
    rightMotorBackward(pwm);
    leftMotorBackward(pwm);

  } else if (dir == "Left") {
    rightMotorForward(pwm);
    leftMotorBackward(pwm);

  } else if (dir == "Right") {
    rightMotorBackward(pwm);
    leftMotorForward(pwm);

  } else if (dir == "Arm") {
    // If your UI uses negative for down, positive for up:
    if (speed > 0) armMotorUp(pwm);
    else           armMotorDown(pwm);
  } else if (dir == "ArmUp") {
    armMotorUp(pwm);
  } else if (dir == "ArmDown") {
    armMotorDown(pwm);
  }
}

void lightControl() {
  if (!light) {
    //digitalWrite(lightPin1, HIGH);
    //digitalWrite(lightPin2, LOW);
    light = true;
    DBG_PRINTLN("Lights ON");
    playSystemSound("/web/pcm/hit.wav");
  } else {
    //digitalWrite(lightPin1, LOW);
    //digitalWrite(lightPin2, LOW);
    light = false;
    DBG_PRINTLN("Lights OFF");
    playSystemSound("/web/pcm/click.wav");
  }
}

void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      DBG_PRINTF("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      lastWsClientIP = client->remoteIP().toString();
      lastWsConnectTime = millis();

      client->text("HoldBucket," + String(holdBucket ? 1 : 0));
      client->text("HoldAux," + String(holdAux ? 1 : 0));
      client->text("Switch," + String(horScreen ? 1 : 0));
      client->text("DarkMode," + String(darkMode ? 1 : 0));
      client->text("RecordTelemetry," + String(tlmEnabled ? 1 : 0));
      client->text("SystemSounds," + String(sSndEnabled ? 1 : 0));
      client->text("SystemVolume," + String(sSndVolume));


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
      DBG_PRINTF("WebSocket client #%u disconnected\n", client->id());
      lastWsClientIP = ""; // Clear
      moveCar(STOP);
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

        // ---------- JSON branch for key events ----------
        if (len > 0 && ((const char*)data)[0] == '{') {
          StaticJsonDocument<128> jd;
          DeserializationError derr = deserializeJson(jd, (const char*)data, len);
          if (!derr) {
            const char* t = jd["t"] | "";
            if (strcmp(t, "key") == 0) {
              const char* code = jd["code"]  | "";
              const char* st   = jd["state"] | "down";
              Action a = actionForKeyToken(String(code));
              if (a != ACT_NONE) {
                bool pressed = (strcmp(st, "down") == 0);
                dispatchAction(a, pressed);
              }
              return; // handled as JSON; do not run CSV path
            }
          }
          // If it's JSON but not our type, fall through to CSV parsing
        }

        // ---------- CSV parsing path ----------
        // Format: "KEY,value1,value2"
        std::string key, value1, value2;
        char* buf = (char*)alloca(len + 1);   // stack buffer to avoid heap fragmentation
        memcpy(buf, data, len);
        buf[len] = 0;

        char* save = nullptr;
        char* t1 = strtok_r(buf, ",", &save);
        char* t2 = strtok_r(nullptr, ",", &save);
        char* t3 = strtok_r(nullptr, ",", &save);

        if (t1) key.assign(t1);
        if (t2) value1.assign(t2);
        if (t3) value2.assign(t3);

        // --- Media control handlers ---
        if (key == "MEDIA_PLAY") {
          DBG_PRINTF("[WS] MEDIA_PLAY: %s\n", value1.c_str());
          String filename = value1.c_str();
          if (!filename.startsWith("/media/")) filename = "/media/" + filename;

          currentTrack = -1;
          for (int i = 0; i < playlist.size(); i++) {
            if (playlist[i] == filename || playlist[i].endsWith(filename.substring(filename.lastIndexOf('/') + 1))) {
              currentTrack = i;
              break;
            }
          }
          if (currentTrack >= 0) {
            playCurrentTrack();
          } else {
            playWavFileOnSpeaker(filename);
            playbackStarted = true;
            isPaused = false;
          }
          wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + filename);
          lastMediaProgressSend = millis();
        }
        else if (key == "MEDIA_NEXT") {
          if (!playlist.empty()) {
            nextTrack();
            if (currentTrack >= 0 && currentTrack < playlist.size() && audio.isRunning()) {
              wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + playlist[currentTrack]);
            }
          }
        }
        else if (key == "MEDIA_STOP")   { stopAudio(); wsCarInput.textAll("MEDIA_DEVICE_STOPPED"); isPaused = false; }
        else if (key == "MEDIA_PAUSE")  { pauseAudio(); wsCarInput.textAll("MEDIA_DEVICE_PAUSED"); }
        else if (key == "MEDIA_RESUME") { resumeAudio(); wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + playlist[currentTrack]); }
        else if (key == "MEDIA_PREV")   { prevTrack();  wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + playlist[currentTrack]); }
        else if (key == "MEDIA_LOOP_ON")     { loopMode = true;  }
        else if (key == "MEDIA_LOOP_OFF")    { loopMode = false; }
        else if (key == "MEDIA_SHUFFLE_ON")  { shuffleMode = true;  }
        else if (key == "MEDIA_SHUFFLE_OFF") { shuffleMode = false; }

        // --- MIC STREAM COMMANDS ---
        if (key == "START_MIC_STREAM") { micStreamActive = true;  client->text("MIC_STREAM_ON"); }
        else if (key == "STOP_MIC_STREAM") { micStreamActive = false; client->text("MIC_STREAM_OFF"); }

        // --- Turn signals via Slider ---
        if (key == "Slider") {
          String side = value1.c_str();
          int val = atoi(value2.c_str());
          if (side == "Left") {
            if (val > 5 && !leftSignalActive)  { leftSignalActive = true;  updatePixels(); wsCarInput.textAll("TURN_LEFT,1"); }
            else if (val <= 5 && leftSignalActive) { leftSignalActive = false; updatePixels(); wsCarInput.textAll("TURN_LEFT,0"); }
          } else if (side == "Right") {
            if (val > 5 && !rightSignalActive) { rightSignalActive = true;  updatePixels(); wsCarInput.textAll("TURN_RIGHT,1"); }
            else if (val <= 5 && rightSignalActive) { rightSignalActive = false; updatePixels(); wsCarInput.textAll("TURN_RIGHT,0"); }
          }
        }

        if (key == "Beacon")    { beaconOn    = !beaconOn;    updatePixels(); }
        if (key == "Emergency") { emergencyOn = !emergencyOn; updatePixels(); }

        if (key == "Motor") {
          int pwm = atoi(value2.c_str());
          controlMotorByDirection(value1, pwm);
        }

        DBG_PRINTF("Key [%s] Value1[%s] Value2[%s]\n", key.c_str(), value1.c_str(), value2.c_str());
        int valueInt = atoi(value1.c_str());

        if (key == "MoveCar") {
          int pwm = (value2.empty() ? 255 : atoi(value2.c_str()));
          moveCar(valueInt, pwm);
          DBG_PRINTF("[WS] moveCar called with valueInt=%d pwm=%d\n", valueInt, pwm);
        }
        else if (key == "AUX")    auxControl(valueInt);
        else if (key == "Bucket") bucketTilt(valueInt);
        else if (key == "Light") {
          lightControl(); updatePixels();
          wsCarInput.textAll("Light," + String(light ? 1 : 0));
        }
        else if (key == "Switch")          { horScreen = (valueInt != 0); uiPrefs.putBool("Switch", horScreen); }
        else if (key == "HoldBucket")      { holdBucket = (valueInt != 0); uiPrefs.putBool("HoldBucket", holdBucket); }
        else if (key == "HoldAux")         { holdAux = (valueInt != 0); uiPrefs.putBool("HoldAux", holdAux); }
        else if (key == "DarkMode")        { darkMode = (valueInt != 0); uiPrefs.putBool("darkMode", darkMode); }
        else if (key == "RecordTelemetry") { setTelemetryEnabled(valueInt != 0); }
        else if (key == "SystemSounds")    { int v = atoi(value1.c_str()); sSndEnabled = (v != 0); uiPrefs.putBool("SystemSounds", sSndEnabled); }
        else if (key == "SystemVolume")    { int v = atoi(value1.c_str()); sSndVolume = v; uiPrefs.putInt("SystemVolume", sSndVolume); }

      } // if (frame ok)
      break;           // <-- IMPORTANT: end of WS_EVT_DATA case
    }                  // <-- end of case block
  }
}

// ---------- Helpers to (re)seed and read maps ----------
static void initInputMapsIfEmpty() {
  for (size_t i = 0; i < KEYMAP_N; ++i) {
    const char* k = KEYMAP_DEFAULTS[i].action;
    String cur = keymapPrefs.getString(k, "");
    if (cur.length() == 0) keymapPrefs.putString(k, KEYMAP_DEFAULTS[i].def);
  }
  for (size_t i = 0; i < JOYMAP_N; ++i) {
    const char* k = JOYMAP_DEFAULTS[i].action;
    String cur = joymapPrefs.getString(k, "");
    if (cur.length() == 0) joymapPrefs.putString(k, JOYMAP_DEFAULTS[i].defBtn);
  }
}

static inline void keymapLoadToJson(JsonObject root) {
  for (size_t i=0;i<KEYMAP_N;++i) {
    const char* k = KEYMAP_DEFAULTS[i].action;
    const char* d = KEYMAP_DEFAULTS[i].def;
    root[k] = keymapPrefs.getString(k, d);
  }
}
static inline void joymapLoadToJson(JsonObject root) {
  for (size_t i=0;i<JOYMAP_N;++i) {
    const char* k = JOYMAP_DEFAULTS[i].action;
    const char* d = JOYMAP_DEFAULTS[i].defBtn;
    root[k] = joymapPrefs.getString(k, d);
  }
}

static inline void keymapResetDefaults() {
  for (size_t i=0;i<KEYMAP_N;++i)
    keymapPrefs.putString(KEYMAP_DEFAULTS[i].action, KEYMAP_DEFAULTS[i].def);
}
static inline void joymapResetDefaults() {
  for (size_t i=0;i<JOYMAP_N;++i)
    joymapPrefs.putString(JOYMAP_DEFAULTS[i].action, JOYMAP_DEFAULTS[i].defBtn);
}

// ---------- Resolve helpers you can call at runtime ----------
static Action actionFromName(const String& name) {
  if (name=="forward") return ACT_FORWARD;
  if (name=="backward") return ACT_BACKWARD;
  if (name=="left") return ACT_LEFT;
  if (name=="right") return ACT_RIGHT;
  if (name=="stop") return ACT_STOP;
  if (name=="bucket_up") return ACT_BUCKET_UP;
  if (name=="bucket_down") return ACT_BUCKET_DOWN;
  if (name=="aux_up") return ACT_AUX_UP;
  if (name=="aux_down") return ACT_AUX_DOWN;
  if (name=="light_toggle") return ACT_LIGHT_TOGGLE;
  if (name=="beacon_toggle") return ACT_BEACON_TOGGLE;
  if (name=="emergency_toggle") return ACT_EMERGENCY_TOGGLE;
  if (name=="horn") return ACT_HORN;
  return ACT_NONE;
}

// Normalize keyboard key token (e.g. "W" -> "w", "Space" -> " ")
static String normKey(String k) {
  k.trim();
  k.toLowerCase();
  // common aliases
  if (k=="space" || k=="spacebar") return " ";
  if (k=="arrowup")    return "arrowup";
  if (k=="arrowdown")  return "arrowdown";
  if (k=="arrowleft")  return "arrowleft";
  if (k=="arrowright") return "arrowright";
  return k;
}

static Action actionForKeyToken(String keyToken) {
  String k = normKey(keyToken);
  for (size_t i=0;i<KEYMAP_N;++i) {
    String bound = keymapPrefs.getString(KEYMAP_DEFAULTS[i].action, KEYMAP_DEFAULTS[i].def);
    bound = normKey(bound);
    if (k == bound) return actionFromName(KEYMAP_DEFAULTS[i].action);
  }
  return ACT_NONE;
}

// Joy button names are symbolic (“A”, “B”, “DPAD_UP”, “L1”, “R1”, etc.)
static String normBtn(String b) {
  b.trim();
  b.toUpperCase();
  // allow aliases
  if (b=="LB") b="L1"; if (b=="RB") b="R1";
  if (b=="LT") b="L2"; if (b=="RT") b="R2";
  return b;
}
static Action actionForJoyButton(String btnName) {
  String b = normBtn(btnName);
  for (size_t i=0;i<JOYMAP_N;++i) {
    String bound = normBtn(joymapPrefs.getString(JOYMAP_DEFAULTS[i].action, JOYMAP_DEFAULTS[i].defBtn));
    if (b == bound) return actionFromName(JOYMAP_DEFAULTS[i].action);
  }
  return ACT_NONE;
}

// ---------- Central dispatcher (call your actual robot functions here) ----------
// NOTE: Replace the TODOs with your real control functions.
static void dispatchAction(Action a, bool pressed) {
  // Movement: act on press; release to stop if needed
  switch (a) {
    case ACT_FORWARD:       /* TODO: on press go forward; on release stop/neutral */ break;
    case ACT_BACKWARD:      /* TODO */ break;
    case ACT_LEFT:          /* TODO */ break;
    case ACT_RIGHT:         /* TODO */ break;
    case ACT_STOP:          if (pressed) {/* TODO: stop all motion */} break;

    case ACT_BUCKET_UP:     if (pressed) {/* TODO: bucketTilt(+step) or start hold */} break;
    case ACT_BUCKET_DOWN:   if (pressed) {/* TODO: bucketTilt(-step) or start hold */} break;
    case ACT_AUX_UP:        if (pressed) {/* TODO: auxControl(+step) */} break;
    case ACT_AUX_DOWN:      if (pressed) {/* TODO: auxControl(-step) */} break;

    case ACT_LIGHT_TOGGLE:  if (pressed) {/* TODO: toggle light */} break;
    case ACT_BEACON_TOGGLE: if (pressed) {/* TODO */} break;
    case ACT_EMERGENCY_TOGGLE: if (pressed) {/* TODO */} break;
    case ACT_HORN:          /* on press: start horn; on release: stop horn */ break;
    default: break;
  }
}

//--------------------------------Bluepad32 Functions-----------------------------------------------------------------------

#if USE_BLUEPAD32

  void onConnectedController(ControllerPtr ctl) {
      for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
          if (myControllers[i] == nullptr) {
              myControllers[i] = ctl;
              DBG_PRINTF("Controller connected! Index=%d\n", i);
              break;
          }
      }
  }

  void onDisconnectedController(ControllerPtr ctl) {
      for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
          if (myControllers[i] == ctl) {
              myControllers[i] = nullptr;
              DBG_PRINTF("Controller disconnected! Index=%d\n", i);
              break;
          }
      }
  }
  // --- New Handle BLE Controller Logic ---
  void handleControllerInput(ControllerPtr ctl) {
      // --- RIGHT STICK: DRIVE (car movement) ---
      int rx = ctl->axisRY();
      int ry = ctl->axisRX();
      const int deadzone = 100;

      // Swap forward/backward so up = forward
      //ry = -ry;

      // Deadzone
      if (abs(rx) < deadzone) rx = 0;
      if (abs(ry) < deadzone) ry = 0;

      // Map joystick range (-511...511) to PWM (-255...255)
      int speed = map(ry, -511, 511, -255, 255);  // Forward positive, reverse negative
      int turn  = map(rx, -511, 511, -255, 255);  // Left negative, right positive

      // Calculate motor PWM values (arcade drive)
      int leftPWM  = speed + turn;
      int rightPWM = speed - turn;

      // Clip to valid PWM range
      leftPWM  = constrain(leftPWM,  -255, 255);
      rightPWM = constrain(rightPWM, -255, 255);

      // --- Motor control ---
      static int lastLeftPWM = 0, lastRightPWM = 0;

      if (leftPWM != lastLeftPWM || rightPWM != lastRightPWM) {
          // Forward/reverse/stop logic
          if (leftPWM == 0 && rightPWM == 0) {
              rightMotorStop();
              leftMotorStop();
          } else {
              // Left motor
              if (leftPWM > 0)
                  leftMotorForward(abs(leftPWM));
              else if (leftPWM < 0)
                  leftMotorBackward(abs(leftPWM));

              // Right motor
              if (rightPWM > 0)
                  rightMotorForward(abs(rightPWM));
              else if (rightPWM < 0)
                  rightMotorBackward(abs(rightPWM));
          }
          lastLeftPWM = leftPWM;
          lastRightPWM = rightPWM;
      }

      // --- LEFT STICK (Y only): ARM UP/DOWN ---
      int ly = ctl->axisY();
      ly = -ly; // Up = arm up, down = arm down

      int armPWM = 0;
      if (abs(ly) > deadzone) {
          armPWM = map(ly, -511, 511, -255, 255);
      }

      static int lastArmPWM = 0;
      if (armPWM != lastArmPWM) {
          if (armPWM == 0) {
              armMotorStop();
          } else if (armPWM > 0) {
              armMotorUp(abs(armPWM));
          } else if (armPWM < 0) {
              armMotorDown(abs(armPWM));
          }
          lastArmPWM = armPWM;
      }

      // --- BUTTONS ---
      static bool lastR1 = false, lastR2 = false, lastL1 = false, lastL2 = false;
      static bool lastXBtn = false, lastCircle = false, lastTriangle = false, lastSquare = false;

      // BUCKET: R1 (up), R2 (down)
      bool r1 = ctl->r1();
      bool r2 = ctl->r2();
      if (r1 && !lastR1) {
          bucketTilt(lastBucketValue + 5);
      }
      if (r2 && !lastR2) {
          bucketTilt(lastBucketValue - 5);
      }
      lastR1 = r1; lastR2 = r2;

      // AUX: L1 (up), L2 (down)
      bool l1 = ctl->l1();
      bool l2 = ctl->l2();
      if (l1 && !lastL1) {
          auxControl(lastAuxValue + 5);
      }
      if (l2 && !lastL2) {
          auxControl(lastAuxValue - 5);
      }
      lastL1 = l1; lastL2 = l2;

      // LIGHT: X button
      bool xBtn = ctl->x();
      if (xBtn && !lastXBtn) {
          lightControl();
          updatePixels();
      }
      lastXBtn = xBtn;

      // BEACON: Circle/B button
      bool circleBtn = ctl->b();
      if (circleBtn && !lastCircle) {
          beaconOn = !beaconOn;
          updatePixels();
      }
      lastCircle = circleBtn;

      // EMERGENCY: Triangle/Y button
      bool triangleBtn = ctl->y();
      if (triangleBtn && !lastTriangle) {
          emergencyOn = !emergencyOn;
          updatePixels();
      }
      lastTriangle = triangleBtn;

      // HORN: Square/A button (future)
      bool squareBtn = ctl->a();
      if (squareBtn && !lastSquare) {
          // Horn logic (if needed)
      }
      lastSquare = squareBtn;
  }

#endif

//-----------------------------------------------------------------------Robot I2S Audio-----------------------------------------------------------------

void stopAudio() {
  audio.stopSong();
  digitalWrite(I2S_SPK_PA, LOW);
  playbackStarted = false;
}

  // Call once when enabling mic (after i2s_driver_install)
void setupI2SMic() {
  static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  static const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num  = I2S_MIC_WS,
    .data_out_num = -1,
    .data_in_num  = I2S_MIC_SD
  };

  i2s_driver_install(MY_I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(MY_I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(MY_I2S_PORT);

  if (!micBuf) {
#if USE_PSRAM
    micBuf = (int16_t*)ps_malloc(micBufBytes);
#else
    micBuf = (int16_t*)malloc(micBufBytes);
#endif
  }
  micLastSendMs = millis();
}

void enableMic() {
  if (currentI2SMode == I2S_MIC) { DBG_PRINTLN("[I2S] Mic already enabled."); return; }
  DBG_PRINTLN("[I2S] Switching to MIC: stopping audio, uninstalling I2S, and initializing mic...");
  stopAudio();
  esp_err_t res = i2s_driver_uninstall(MY_I2S_PORT);
  if (res == ESP_OK)          DBG_PRINTLN("[I2S] I2S driver uninstalled successfully (for MIC).");
  else if (res == ESP_ERR_INVALID_STATE) DBG_PRINTLN("[I2S] I2S driver not installed, nothing to uninstall.");
  else                      DBG_PRINTF("[I2S] I2S driver uninstall for MIC returned code: %d\n", res);

  setupI2SMic();
  currentI2SMode = I2S_MIC;
  DBG_PRINTLN("[I2S] MIC enabled and ready.");
}

void disableMic() {
  if (currentI2SMode == I2S_MIC) {
    DBG_PRINTLN("[I2S] Disabling MIC: uninstalling I2S driver...");
    esp_err_t res = i2s_driver_uninstall(MY_I2S_PORT);
    if (res == ESP_OK) DBG_PRINTLN("[I2S] I2S driver uninstalled successfully (MIC off).");
    else               DBG_PRINTF("[I2S] I2S driver uninstall for MIC returned code: %d\n", res);
    currentI2SMode = I2S_NONE;
  } else {
    DBG_PRINTLN("[I2S] MIC not enabled, nothing to disable.");
  }
  if (micBuf) { free(micBuf); micBuf = nullptr; }
}

void streamMicToWebSocket() {
  if (!micStreamActive) return;
  if (currentI2SMode != I2S_MIC) return;

  // Only if someone’s listening
  if (wsCarInput.count() == 0) {
    // Small sleep to avoid spinning CPU if called in a tight loop
    delay(1);
    return;
  }

  // Throttle a bit (e.g., ~50 packets/sec) to avoid WS queue piling up
  const uint32_t now = millis();
  if (now - micLastSendMs < 20) return;

  if (!micBuf) return; // should not happen; safety

  size_t bytesRead = 0;
  // Short timeout; don't block the async loop
  esp_err_t res = i2s_read(MY_I2S_PORT,
                           (void*)micBuf,
                           micBufBytes,
                           &bytesRead,
                           10 / portTICK_PERIOD_MS);

  if (res == ESP_OK && bytesRead > 0) {
    // Send as binary; clients that don't care (UI) ignore non-text frames
    wsCarInput.binaryAll((uint8_t*)micBuf, bytesRead);
    micLastSendMs = now;
  }

  // Cooperative yield
  delay(0);
}


void playWavFileOnSpeaker(const String& filename) {
    DBG_PRINTF("[AUDIO] playWavFileOnSpeaker: %s\n", filename.c_str());
    if (!SD.exists(filename.c_str())) {
        DBG_PRINTF("[ERROR] File not found on SD: %s\n", filename.c_str());
        return;
    }
    disableMic();
    stopAudio();
    delay(100);
    enableSpeaker();
    audio.connecttoFS(SD, filename.c_str());
    currentlyPlayingFile = filename;           // <--- ADD THIS LINE!
    lastMediaProgressSend = 0;                 // <--- (reset progress timer)
    delay(50);
    if (!audio.isRunning()) DBG_PRINTLN("[AUDIO] Playback did not start!");
    else DBG_PRINTLN("[AUDIO] Playback started OK.");
}

void makeWavHeader(uint8_t* header, int sampleRate, int channels, int bitsPerSample, uint32_t dataLength) {
    // PCM WAV header
    uint32_t byteRate = sampleRate * channels * bitsPerSample / 8;
    uint16_t blockAlign = channels * bitsPerSample / 8;
    memcpy(header, "RIFF", 4);
    uint32_t chunkSize = 36 + dataLength;
    memcpy(header+4, &chunkSize, 4);
    memcpy(header+8, "WAVE", 4);
    memcpy(header+12, "fmt ", 4);
    uint32_t subChunk1Size = 16;
    memcpy(header+16, &subChunk1Size, 4);
    uint16_t audioFormat = 1;
    memcpy(header+20, &audioFormat, 2);
    memcpy(header+22, &channels, 2);
    memcpy(header+24, &sampleRate, 4);
    memcpy(header+28, &byteRate, 4);
    memcpy(header+32, &blockAlign, 2);
    memcpy(header+34, &bitsPerSample, 2);
    memcpy(header+36, "data", 4);
    memcpy(header+40, &dataLength, 4);
}

// This is your speaker setup from audio library
void enableSpeaker() {
    // Always ensure amp enable pin is set before playback
    pinMode(I2S_SPK_PA, OUTPUT);
    digitalWrite(I2S_SPK_PA, HIGH);  // Enable NS4168 PA

    if (currentI2SMode == I2S_SPEAKER) {
        DBG_PRINTLN("[I2S] Speaker already enabled.");
        return;
    }
    DBG_PRINTLN("[I2S] Switching to SPEAKER: stopping audio, uninstalling I2S, and initializing speaker...");

    // Always stop audio regardless of state
    audio.stopSong();

    // Only uninstall if previously MIC or SPEAKER was initialized
    if (currentI2SMode != I2S_NONE) {
        esp_err_t res = i2s_driver_uninstall(MY_I2S_PORT);
        delay(20); // Allow I2S peripheral to settle
        if (res == ESP_OK) {
            DBG_PRINTLN("[I2S] I2S driver uninstalled successfully (for SPEAKER).");
        } else {
            DBG_PRINTF("[I2S] I2S driver uninstall for SPEAKER returned code: %d\n", res);
        }
    }

    // Re-setup speaker I2S pinout and volume every time
    audio.setPinout(I2S_SPK_BCLK, I2S_SPK_LRCK, I2S_SPK_SD); // (10, 45, 9)
    audio.setVolume(15);  // Use your tested value (or 21 for max)
    currentI2SMode = I2S_SPEAKER;
    DBG_PRINTLN("[I2S] SPEAKER enabled and ready.");
}

void sendDeviceMediaProgress() {
    // Only send if playing a file
    if (currentlyPlayingFile.length() > 0 && audio.isRunning()) {
        unsigned long now = millis();
        if (now - lastMediaProgressSend > 500) {  // Send progress update every 0.5s
            lastMediaProgressSend = now;
            // Duration and position may not be available in Audio.h; if so, just fake for now
            int duration = audio.getAudioFileDuration() / 1000; // seconds (implement this if not present)
            int pos = audio.getAudioCurrentTime() / 1000; // seconds (implement this if not present)
            wsCarInput.textAll("MEDIA_DEVICE_PROGRESS," + currentlyPlayingFile + "," + String(pos) + "," + String(duration));
        }
    }
}

void pauseAudio() {
  if (audio.isRunning()) {
    audio.stopSong();
    isPaused = true;
    DBG_PRINTLN("[AUDIO] Paused (returns to start on resume)");
  }
  playbackStarted = false;
}

void resumeAudio() {
  if (!isPaused || playlist.empty()) return;
  DBG_PRINTF("[AUDIO] Resuming %s from start\n", playlist[currentTrack].c_str());
  enableSpeaker();
  audio.connecttoFS(SD, playlist[currentTrack].c_str());
  playbackStarted = true;
  isPaused = false;
}

void playCurrentTrack() {
    if (playlist.empty() || currentTrack < 0 || currentTrack >= playlist.size()) {
        DBG_PRINTLN("[playCurrentTrack] Playlist empty or currentTrack invalid!");
        return;
    }
    String filename = playlist[currentTrack];
    if (!filename.startsWith("/media/")) filename = "/media/mp3/" + filename;

    playWavFileOnSpeaker(filename);
    playbackStarted = true;
    isPaused = false;
}

void nextTrack() {
    if (playlist.empty()) return;
    if (shuffleMode) {
        int nextIdx;
        do {
            nextIdx = random(0, playlist.size());
        } while (playlist.size() > 1 && nextIdx == currentTrack);
        currentTrack = nextIdx;
    } else {
        currentTrack++;
        if (currentTrack >= (int)playlist.size()) {
            if (loopMode) currentTrack = 0;
            else { audio.stopSong(); isPaused = false; return; }
        }
    }
    playCurrentTrack();
}

void prevTrack() {
  if (playlist.empty()) return;
  if (shuffleMode) {
    int prevIdx;
    do {
      prevIdx = random(0, playlist.size());
    } while (playlist.size() > 1 && prevIdx == currentTrack);
    currentTrack = prevIdx;
  } else {
    currentTrack--;
    if (currentTrack < 0) {
      if (loopMode) currentTrack = playlist.size() - 1;
      else { audio.stopSong(); isPaused = false; return; }
    }
  }
  playCurrentTrack();
}

void setVolume(int vol) {
  if (vol < 0) vol = 0;
  if (vol > 21) vol = 21;
  currentVolume = vol;
  audio.setVolume(currentVolume);
  DBG_PRINTF("[VOLUME] %d\n", currentVolume);
}

void randomTrack() {
  if (playlist.empty()) return;
  currentTrack = random(0, playlist.size());
  DBG_PRINTF("[RANDOM] %s (Volume: %d)\n", playlist[currentTrack].c_str(), currentVolume);
  playCurrentTrack();
}

void nextFolder() {
  if (folderIndex.size() <= 1) {
    DBG_PRINTLN("Only one folder in list.");
    return;
  }
  // Find which folder currentTrack is in
  int folder = 0;
  for (int i = 0; i < (int)folderIndex.size(); ++i) {
    if (currentTrack < (i + 1 < (int)folderIndex.size() ? folderIndex[i + 1] : (int)playlist.size())) {
      folder = i;
      break;
    }
  }
  folder = (folder + 1) % folderIndex.size();
  currentTrack = folderIndex[folder];
  DBG_PRINTF("[NEXTFOLDER] Now playing from folder %s\n", mediaFolders[folder]);
  playCurrentTrack();
}

void resetESP() {
  DBG_PRINTLN("Resetting ESP32...");
  delay(200);
  ESP.restart();
}

//----------------------PSRAM VERSION---------------------------------------------------------

bool loadPlaylistFromIndex(const char* folder) {
    SdLock lock; // 🔒 protect SD card access

    playlist.clear();
    String indexPath = String(folder) + "/.index";
    File idx = SD.open(indexPath);
    if (!idx) {
        DBG_PRINTF("[ERROR] Can't open index: %s\n", indexPath.c_str());
        return false;
    }

    char lineBuf[256]; // Reasonable max line size for filenames
    size_t lineLen = 0;

    while (idx.available()) {
        lineLen = idx.readBytesUntil('\n', lineBuf, sizeof(lineBuf) - 1);
        lineBuf[lineLen] = '\0';

        // Remove trailing CR/LF/spaces/tabs
        while (lineLen > 0 && 
               (lineBuf[lineLen - 1] == '\r' || lineBuf[lineLen - 1] == ' ' || lineBuf[lineLen - 1] == '\t')) {
            lineBuf[--lineLen] = '\0';
        }
        if (lineLen == 0) continue;

        // Find first comma and cut it off
        char* firstComma = strchr(lineBuf, ',');
        if (firstComma) *firstComma = '\0';

        // Trim leading spaces/tabs
        char* fileOnly = lineBuf;
        while (*fileOnly == ' ' || *fileOnly == '\t') ++fileOnly;

        // Trim trailing spaces/tabs
        char* end = fileOnly + strlen(fileOnly) - 1;
        while (end > fileOnly && (*end == ' ' || *end == '\t')) *end-- = '\0';

        // Build full path
        String fullPath;
        if (fileOnly[0] == '/') {
            fullPath = fileOnly;
        } else {
            fullPath.reserve(strlen(folder) + 1 + strlen(fileOnly));
            fullPath = folder;
            if (!fullPath.isEmpty() && fullPath[fullPath.length() - 1] != '/') fullPath += '/';
            fullPath += fileOnly;
        }

        playlist.push_back(fullPath); // PSRAM-friendly push_back
    }

    idx.close();
    currentTrack = 0;
    return !playlist.empty();
}


//--------------------------------------------------------------Firmware OTA----------------------------------------------------------------------

void otaUpdate(){
    // OTA (unchanged)
  ElegantOTA.begin(&server);  // Works with AsyncWebServer
  DBG_PRINTLN("OTA ready: http://<device_ip>/update");
  ElegantOTA.onEnd([](bool success) {
      DBG_PRINTLN("ElegantOTA finished, restarting...");
      //keymapPrefs.end();
      //preferences.end();
      delay(500);
      ESP.restart();
  });
}

String urlDecode(String input) {
    String s = "";
    char a, b;
    for (size_t i = 0; i < input.length(); i++) {
        if ((input[i] == '%') && ((a = input[i + 1]) && (b = input[i + 2])) &&
            (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a') a -= 'a' - 'A';
            if (a >= 'A') a = a - 'A' + 10;
            else a -= '0';
            if (b >= 'a') b -= 'a' - 'A';
            if (b >= 'A') b = b - 'A' + 10;
            else b -= '0';
            s += char(16 * a + b);
            i += 2;
        } else if (input[i] == '+') {
            s += ' ';
        } else {
            s += input[i];
        }
    }
    return s;
}

String normalizedSSID(const String& raw) {
  String out = raw;
  out.trim();
  return out;
}

String getContentType(const String& filename) {
    if (filename.endsWith(".mp3"))  return "audio/mpeg";
    if (filename.endsWith(".wav"))  return "audio/wav";
    if (filename.endsWith(".ogg"))  return "audio/ogg";
    if (filename.endsWith(".mp4"))  return "video/mp4";
    if (filename.endsWith(".webm")) return "video/webm";
    if (filename.endsWith(".mov"))  return "video/quicktime";
    return "application/octet-stream";
}

//------------------------------------------------MEDIA CAPTURE FUNCTIONS-------------------------------------------

// Utility: Get timestamp filename

#if USE_PSRAM
std::vector<std::vector<uint8_t, psram_allocator<uint8_t>>, psram_allocator<std::vector<uint8_t, psram_allocator<uint8_t>>>> frameBuffer;
#else
std::vector<std::vector<uint8_t>> frameBuffer;
#endif

// Usage in your function:
void bufferFramesExample() {
    for (int i = 0; i < 10; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            // Buffer frame into PSRAM-backed vector
            frameBuffer.emplace_back(fb->len);
            memcpy(frameBuffer.back().data(), fb->buf, fb->len);
            esp_camera_fb_return(fb);
        }
    }
    // Later: write all buffered frames to SD
}


String getMediaTimestamp(const char* prefix, const char* ext) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm* tm_info = localtime(&tv.tv_sec);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", tm_info);
    char path[80];
    snprintf(path, sizeof(path), "/media/capture/%s/%s.%s", prefix, buf, ext);
    return String(path);
}

void recordVideoTask(void* parameter) {
  const int durationSec = *((int*)parameter);
  delete (int*)parameter;

  // Ensure folder exists
  {
    SdLock lk;
    if (!SD.exists("/media/capture/video")) SD.mkdir("/media/capture/video");
  }

  const String filePath = getMediaTimestamp("video", "mjpeg");

  File f;
  {
    SdLock lk;                                  // lock only while opening
    f = SD.open(filePath, FILE_WRITE);
  }
  if (!f) {
    videoRecording  = false;
    videoTaskActive = false;
    vTaskDelete(NULL);
    return;
  }

  videoRecording  = true;
  videoTaskActive = true;

  static constexpr char BOUNDARY[] =
      "\r\n--123456789000000000000987654321\r\n";

  const unsigned long endTime = millis() + (unsigned long)durationSec * 1000UL;

  while (videoRecording && millis() < endTime) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      // Prepare header outside the lock
      char header[64];
      const int hdrLen = snprintf(header, sizeof(header),
          "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);

      // Do the SD writes under a short lock
      {
        SdLock lk;
        f.write((const uint8_t*)BOUNDARY, sizeof(BOUNDARY) - 1);
        f.write((const uint8_t*)header,   hdrLen);
        f.write(fb->buf, fb->len);
        // Optionally flush every few frames if you want extra safety
        // if ((millis() & 0x1F) == 0) f.flush();
      }

      esp_camera_fb_return(fb);
    }

    // Be cooperative; don't block the system
    vTaskDelay(pdMS_TO_TICKS(90));   // ~10–11 fps
  }

  {
    SdLock lk;                        // lock while closing
    f.close();
  }

  videoRecording  = false;
  videoTaskActive = false;

  // Safe to play the WAV now: SD is free and we’re outside the handler
  playSystemSound("/web/pcm/vreccomplete.wav");

  vTaskDelete(NULL);
}

//---------------------DEBUG Helpers for get/set_keymap handlers------------------------------------------------------

#ifndef DBG_PRINTF
  #define DBG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#endif

// Print current stored keymap from Preferences
static void dumpKeymapToSerial(const char* tag) {
  DBG_PRINTF("[KEYMAP] %s\n", tag ? tag : "");
  for (size_t i = 0; i < KEYMAP_N; ++i) {
    const char* a = KEYMAP_DEFAULTS[i].action;
    String cur = keymapPrefs.getString(a, KEYMAP_DEFAULTS[i].def);
    DBG_PRINTF("  %-16s = '%s'\n", a, cur.c_str());
  }
}

// For POST: tell whether a JSON field name is recognized (snake_case or alias)
static bool isKnownActionField(const String& field) {
  for (size_t i = 0; i < ACTION_ALIASES_N; ++i) {
    if (field.equalsIgnoreCase(ACTION_ALIASES[i].fw)) return true;
    if (ACTION_ALIASES[i].alias && field.equalsIgnoreCase(ACTION_ALIASES[i].alias)) return true;
  }
  return false;
}

//--------------------------------------------------------------------------------------------------------------------

void serverStart(){
  
  wifiRetryCount = wifiPrefs.getInt("wifiRetryCount", 5);
  wifiSSID = wifiPrefs.getString("ssid", "");
  wifiPassword = wifiPrefs.getString("password", "");

  // --- Wi-Fi Logic ---
  bool wifiConnected = false;
  if (wifiSSID.length() > 0) {
    DBG_PRINTF("🟡 Found saved Wi-Fi ('%s'), trying to connect...\n", wifiSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

    // Try for up to 8 seconds (32x 250ms)
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 8000) {
      delay(250);
      DBG_PRINT(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      playSystemSound("/web/pcm/click (2).wav");
      wifiConnected = true;
      DBG_PRINTLN("\n✅ Connected to WiFi!");
      DBG_PRINT("STA IP: ");
      DBG_PRINTLN(WiFi.localIP());
      //startCameraServer();
    } else {
      DBG_PRINTLN("\n❌ WiFi connect failed, switching to AP mode.");
    }
  }

  if (!wifiConnected) {
    playSystemSound("/web/pcm/error (5).wav");
    DBG_PRINTLN("🟠 No saved Wi-Fi, or connect failed. Starting AP mode for setup.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid);
    DBG_PRINT("Started AP: ");
    DBG_PRINTLN(ap_ssid);
    DBG_PRINT("AP IP: ");
    DBG_PRINTLN(WiFi.softAPIP());
  }

  // ---- SD card static file serving setup ----
  server.on("/", HTTP_GET, handleRoot);
  //server.serveStatic("/", SD, "/");

  // WebServer Routes (dynamic ESP endpoints)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
      if (SD.exists("/favicon.ico")) {
          request->send(SD, "/favicon.ico", "image/x-icon");
      } else {
          request->send(404, "text/plain", "favicon.ico not found on SD card");
      }
  });
  
  server.on("/setup", HTTP_GET, handleWiFiSetup);
  server.on("/savewifi", HTTP_POST, handleSaveWiFi);
  server.on("/listwifi", HTTP_GET, handleListWiFi);
  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });
  server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });

  // /wifi_try_connect  (POST)  ->  {"ok":bool, "ssid":"...", "ip":"x.x.x.x" | "error":"..."}
  server.on("/wifi_try_connect", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (request->hasParam("ssid", true)) {
      String ssid = normalizedSSID(request->getParam("ssid", true)->value());
      String password = wifiPrefs.getString(("wifi_" + ssid).c_str(), "");
      bool ok = connectToWiFiWithRetries(ssid, password, 5);

      root["ok"]   = ok;
      root["ssid"] = ssid;
      if (ok) { root["ip"] = WiFi.localIP().toString(); resp->setCode(200); }
      else    { root["error"] = "connect_failed";       resp->setCode(500); }
    } else {
      root["ok"] = false; root["error"] = "missing_ssid";
      resp->setCode(400);
    }
    resp->setLength();
    request->send(resp);
  });

  // /calibrate_imu  (POST)  ->  {"status":"stored", sys, gyro, accel, mag} | {"status":"requested"}
  server.on("/calibrate_imu", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    bool stored = imuPrefs.getBool("stored", false);
    int sys=imuPrefs.getInt("sys",-1), gyro=imuPrefs.getInt("gyro",-1),
        accel=imuPrefs.getInt("accel",-1), mag=imuPrefs.getInt("mag",-1);

    if (stored && sys>=0 && gyro>=0 && accel>=0 && mag>=0) {
      root["status"]="stored"; root["sys"]=sys; root["gyro"]=gyro; root["accel"]=accel; root["mag"]=mag;
    } else {
      root["status"]="requested";
    }
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // /get_calibration  (GET)  ->  {"sys":..., "gyro":..., "accel":..., "mag":..., "stored":bool}
  server.on("/get_calibration", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    bool stored = imuPrefs.getBool("stored", false);
    int sys=imuPrefs.getInt("sys",-1), gyro=imuPrefs.getInt("gyro",-1),
        accel=imuPrefs.getInt("accel",-1), mag=imuPrefs.getInt("mag",-1);

    if (stored && sys>=0 && gyro>=0 && accel>=0 && mag>=0) {
      root["sys"]=sys; root["gyro"]=gyro; root["accel"]=accel; root["mag"]=mag; root["stored"]=true;
    } else {
      root["sys"]=0; root["gyro"]=0; root["accel"]=0; root["mag"]=0; root["stored"]=false;
    }
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // GET /get_keymap
  server.on("/get_keymap", HTTP_GET, [](AsyncWebServerRequest* request){
    auto* resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot().to<JsonObject>();

    // Build response
    for (size_t i = 0; i < KEYMAP_N; ++i) {
      const char* k = KEYMAP_DEFAULTS[i].action;
      String v = keymapPrefs.getString(k, KEYMAP_DEFAULTS[i].def);
      root[k] = v;  // normalized ("arrowup", " ")
    }

    // Log the JSON we are about to send
    String out; serializeJson(root, out);
    DBG_PRINTF("[/get_keymap] response: %s\n", out.c_str());

    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---- POST /set_keymap ----
  {
    auto* h = new AsyncCallbackJsonWebHandler("/set_keymap",
      [](AsyncWebServerRequest* request, JsonVariant& json){
        auto* out = new AsyncJsonResponse();
        JsonObject root = out->getRoot().to<JsonObject>();

        if (!json.is<JsonObject>()) {
          root["error"] = "object json expected";
          out->setCode(400); out->setLength(); request->send(out); return;
        }

        JsonObject obj = json.as<JsonObject>();

        // Log raw payload
        String in; serializeJson(obj, in);
        DBG_PRINTF("[/set_keymap] payload: %s\n", in.c_str());

        // Log all fields we received and flag unknowns
        for (JsonPair kv : obj) {
          String k = kv.key().c_str();
          String v = kv.value().as<String>();
          DBG_PRINTF("    recv field '%s' = '%s'%s\n",
            k.c_str(), v.c_str(), isKnownActionField(k) ? "" : "  (UNKNOWN)");
        }

        size_t updated = 0;

        // Accept both firmware snake_case and UI camelCase aliases
        for (size_t i = 0; i < ACTION_ALIASES_N; ++i) {
          const char* fw    = ACTION_ALIASES[i].fw;
          const char* alias = ACTION_ALIASES[i].alias;

          String raw; const char* used = nullptr;
          if (obj.containsKey(fw))         { raw = obj[fw].as<String>(); used = fw; }
          else if (alias && obj.containsKey(alias)) { raw = obj[alias].as<String>(); used = alias; }
          else continue;

          String norm = normKeyToken(raw);     // "ArrowUp"->"arrowup", "Space"->" ", "E"->"e"
          DBG_PRINTF("    apply %-16s from '%s': '%s' -> '%s'\n",
                    fw, used, raw.c_str(), norm.c_str());

          keymapPrefs.putString(fw, norm);
          ++updated;
        }

        if (updated == 0) {
          DBG_PRINTF("[/set_keymap] WARNING: no fields matched known actions (updated=0)\n");
        }

        // Dump what is now stored after writes
        dumpKeymapToSerial("after set_keymap");

        root["status"]  = "ok";
        root["updated"] = (int)updated;
        out->setCode(200); out->setLength(); request->send(out);
      });
    h->setMethod(HTTP_POST);
    h->setMaxContentLength(1024);
    server.addHandler(h);

    wsCarInput.textAll("KEYMAP_UPDATED");
  }



  // POST /reset_keymap
  server.on("/reset_keymap", HTTP_POST, [](AsyncWebServerRequest* request){
    keymapResetDefaults();
    auto* ok = new AsyncJsonResponse(); ok->getRoot()["status"]="ok";
    ok->setCode(200); ok->setLength(); request->send(ok);
  });

  // GET /get_joymap
  server.on("/get_joymap", HTTP_GET, [](AsyncWebServerRequest* request){
    auto* resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot().to<JsonObject>();
    joymapLoadToJson(root);
    resp->setCode(200); resp->setLength(); request->send(resp);
  });

  // POST /set_joymap   body: {"forward":"DPAD_UP","horn":"A",...}
  {
    auto* h = new AsyncCallbackJsonWebHandler("/set_joymap",
      [](AsyncWebServerRequest* request, JsonVariant& json){
        if (!json.is<JsonObject>()) {
          auto* err = new AsyncJsonResponse();
          err->getRoot()["error"]="object json expected";
          err->setCode(400); err->setLength(); request->send(err); return;
        }
        JsonObject obj = json.as<JsonObject>();
        size_t updated=0;
        for (size_t i=0;i<JOYMAP_N;++i) {
          const char* key = JOYMAP_DEFAULTS[i].action;
          if (obj.containsKey(key)) { joymapPrefs.putString(key, obj[key].as<String>()); ++updated; }
        }
        auto* ok = new AsyncJsonResponse();
        ok->getRoot()["status"]="ok";
        ok->getRoot()["updated"]=(int)updated;
        ok->setCode(200); ok->setLength(); request->send(ok);
      });
    h->setMethod(HTTP_POST);
    h->setMaxContentLength(1024);
    server.addHandler(h);
  }

  // POST /reset_joymap
  server.on("/reset_joymap", HTTP_POST, [](AsyncWebServerRequest* request){
    joymapResetDefaults();
    auto* ok = new AsyncJsonResponse(); ok->getRoot()["status"]="ok";
    ok->setCode(200); ok->setLength(); request->send(ok);
  });

  server.on("/ota/upload", HTTP_POST, [](AsyncWebServerRequest *request){},
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0) {
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

  // ---------- /version (GET) ----------
  server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    resp->getRoot()["current"] = FIRMWARE_VERSION;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /list_saved_wifi (GET) ----------
  server.on("/list_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonArray arr = resp->getRoot().to<JsonArray>();

    String savedList = wifiPrefs.getString("networks", "");
    int last = 0;
    while (true) {
      int next = savedList.indexOf(',', last);
      String ssid = (next == -1) ? savedList.substring(last) : savedList.substring(last, next);
      ssid = normalizedSSID(ssid);
      if (ssid.length()) {
        String wifiKey = "wifi_" + ssid;
        String pass    = wifiPrefs.getString(wifiKey.c_str(), "");
        int    retry   = wifiPrefs.getInt(("retry_" + ssid).c_str(), 5);
        bool   autoRec = wifiPrefs.getBool(("aRt_" + ssid).c_str(), false);
        JsonObject it = arr.createNestedObject();
        it["ssid"]=ssid; it["password"]=pass; it["retry"]=retry; it["autoReconnect"]=autoRec;
      }
      if (next == -1) break;
      last = next + 1;
    }
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /wifi_set_autoreconnect (POST, form/query params) ----------
  server.on("/wifi_set_autoreconnect", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (request->hasParam("ssid", true) && request->hasParam("enabled", true)) {
      String ssid = normalizedSSID(request->getParam("ssid", true)->value());
      String enabledStr = request->getParam("enabled", true)->value();
      bool enabled = (enabledStr == "1" || enabledStr == "true");
      String key = "aRt_" + ssid;
      wifiPrefs.putBool(key.c_str(), enabled);
      bool verify = wifiPrefs.getBool(key.c_str(), !enabled);
      root["ok"]=(verify==enabled); root["ssid"]=ssid; root["enabled"]=enabled; root["verified"]=(verify==enabled);
      resp->setCode((verify==enabled)?200:500);
    } else {
      root["ok"]=false; root["error"]="missing_params";
      resp->setCode(400);
    }
    resp->setLength();
    request->send(resp);
  });

  // ---------- /update_wifi_password (GET, query params) ----------
  server.on("/update_wifi_password", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (request->hasParam("ssid") && request->hasParam("password")) {
      String ssid = normalizedSSID(request->getParam("ssid")->value());
      String password = request->getParam("password")->value();
      String wifiKey = "wifi_" + ssid;
      bool ok = wifiPrefs.putString(wifiKey.c_str(), password);
      root["ok"]=ok; root["ssid"]=ssid; root["saved"]=ok;
      resp->setCode(ok?200:500);
    } else {
      root["ok"]=false; root["error"]="missing_params";
      resp->setCode(400);
    }
    resp->setLength();
    request->send(resp);
  });

  // ---------- /connect_saved_wifi (GET -> JSON) ----------
  server.on("/connect_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("ssid")) {
      root["ok"]=false; root["error"]="missing_ssid";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    String ssid = normalizedSSID(request->getParam("ssid")->value());
    String pass = wifiPrefs.getString(("wifi_" + ssid).c_str(), "");
    if (pass == "") {
      root["ok"]=false; root["error"]="ssid_not_found"; root["ssid"]=ssid;
      resp->setCode(404); resp->setLength(); request->send(resp); return;
    }

    wifiPrefs.putString("ssid", ssid);
    wifiPrefs.putString("password", pass);

    DBG_PRINTLN("🔄 Switching to saved Wi-Fi: " + ssid);
    stopWebServerAndWS();
    delay(300);
    connectToWiFiWithRetries(ssid, pass, wifiRetryCount);
    wifiSSID = ssid; wifiPassword = pass; wifiConnecting = true; wifiConnectStartTime = millis();

    root["ok"]=true; root["ssid"]=ssid; root["status"]="switching";
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /update_retry_count (GET -> JSON) ----------
  server.on("/update_retry_count", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!(request->hasParam("ssid") && request->hasParam("count"))) {
      root["ok"]=false; root["error"]="missing_params";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    String ssid = normalizedSSID(request->getParam("ssid")->value());
    int count = request->getParam("count")->value().toInt();
    if (count < 1 || count > 10) {
      root["ok"]=false; root["error"]="invalid_range"; root["min"]=1; root["max"]=10;
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    bool ok = wifiPrefs.putInt(("retry_" + ssid).c_str(), count);
    root["ok"]=ok; root["ssid"]=ssid; root["count"]=count;
    resp->setCode(ok?200:500);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /setsettings (POST JSON -> apply + save) ----------
  {
    auto *setSettings = new AsyncCallbackJsonWebHandler(
      "/setsettings",
      [](AsyncWebServerRequest *request, JsonVariant &json){
        if (!json.is<JsonObject>()) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["error"] = "Invalid JSON (object expected)";
          err->setCode(400); err->setLength(); request->send(err); return;
        }
        JsonObject obj = json.as<JsonObject>();
        for (JsonPair kv : obj) if (kv.value().is<int>()) camPrefs.putInt(kv.key().c_str(), kv.value().as<int>());

        sensor_t *s = esp_camera_sensor_get();
        if (s) {
          auto getI = [&](const char* k, int def=0){ return obj.containsKey(k) ? obj[k].as<int>() : def; };
          if (obj.containsKey("res"))            s->set_framesize(s, (framesize_t)getI("res"));
          if (obj.containsKey("quality"))        s->set_quality(s, getI("quality"));
          if (obj.containsKey("contrast"))       s->set_contrast(s, getI("contrast"));
          if (obj.containsKey("brightness"))     s->set_brightness(s, getI("brightness"));
          if (obj.containsKey("saturation"))     s->set_saturation(s, getI("saturation"));
          if (obj.containsKey("gray"))           s->set_special_effect(s, getI("gray") ? 2 : 0);
          if (obj.containsKey("hmirror"))        s->set_hmirror(s, getI("hmirror"));
          if (obj.containsKey("vflip"))          s->set_vflip(s, getI("vflip"));
          if (obj.containsKey("awb"))            s->set_whitebal(s, getI("awb"));
          if (obj.containsKey("wb_mode"))        s->set_wb_mode(s, getI("wb_mode"));
          if (obj.containsKey("aec"))            s->set_exposure_ctrl(s, getI("aec"));
          if (obj.containsKey("ae_level"))       s->set_ae_level(s, getI("ae_level"));
          if (obj.containsKey("aec_value"))      s->set_aec_value(s, getI("aec_value"));
          if (obj.containsKey("agc"))            s->set_gain_ctrl(s, getI("agc"));
          if (obj.containsKey("agc_gain"))       s->set_agc_gain(s, getI("agc_gain"));
          if (obj.containsKey("gainceiling"))    s->set_gainceiling(s, (gainceiling_t)getI("gainceiling"));
          if (obj.containsKey("awb_gain"))       s->set_awb_gain(s, getI("awb_gain"));
          if (obj.containsKey("colorbar"))       s->set_colorbar(s, getI("colorbar"));
          if (obj.containsKey("lenc"))           s->set_lenc(s, getI("lenc"));
          if (obj.containsKey("bpc"))            s->set_bpc(s, getI("bpc"));
          if (obj.containsKey("wpc"))            s->set_wpc(s, getI("wpc"));
          if (obj.containsKey("dcw"))            s->set_dcw(s, getI("dcw"));
          if (obj.containsKey("raw_gma"))        s->set_raw_gma(s, getI("raw_gma"));
          if (obj.containsKey("special_effect")) s->set_special_effect(s, getI("special_effect"));
        }

        playSystemSound("/web/pcm/click.wav");

        auto *ok = new AsyncJsonResponse();
        ok->getRoot()["status"] = "saved";
        ok->setCode(200);
        ok->setLength();
        request->send(ok);
      }
    );
    setSettings->setMethod(HTTP_POST);
    setSettings->setMaxContentLength(2048);
    server.addHandler(setSettings);
  }

  // ---------- /getsettings (GET) ----------
  server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Build JSON in RAM, then send with a known Content-Length (no chunked)
    DynamicJsonDocument doc(1536);
    JsonObject root = doc.to<JsonObject>();

    // Camera block
    const char* modelName = "Unknown";
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
      switch (s->id.PID) {
        case OV2640_PID: modelName = "OV2640"; break;
        case OV3660_PID: modelName = "OV3660"; break;
        case OV5640_PID: modelName = "OV5640"; break;
        case GC2145_PID: modelName = "GC2145"; break;
        default:         modelName = "Unknown"; break;
      }
      root["model"]          = modelName;
      root["res"]            = (int)s->status.framesize;
      root["quality"]        = (int)s->status.quality;
      root["contrast"]       = (int)s->status.contrast;
      root["brightness"]     = (int)s->status.brightness;
      root["saturation"]     = (int)s->status.saturation;
      root["gray"]           = (int)(s->status.special_effect == 2 ? 1 : 0);
      root["hmirror"]        = (int)s->status.hmirror;
      root["vflip"]          = (int)s->status.vflip;
      root["awb"]            = (int)s->status.awb;
      root["wb_mode"]        = (int)s->status.wb_mode;
      root["aec"]            = (int)s->status.aec;
      root["ae_level"]       = (int)s->status.ae_level;
      root["aec_value"]      = (int)s->status.aec_value;
      root["agc"]            = (int)s->status.agc;
      root["agc_gain"]       = (int)s->status.agc_gain;
      root["gainceiling"]    = (int)s->status.gainceiling;
      root["awb_gain"]       = (int)s->status.awb_gain;
      root["colorbar"]       = (int)s->status.colorbar;
      root["lenc"]           = (int)s->status.lenc;
      root["bpc"]            = (int)s->status.bpc;
      root["wpc"]            = (int)s->status.wpc;
      root["dcw"]            = (int)s->status.dcw;
      root["raw_gma"]        = (int)s->status.raw_gma;
      root["special_effect"] = (int)s->status.special_effect;
    } else {
      root["model"] = modelName;
    }

    // UI / app flags (assumes these globals exist)
    root["darkMode"]         = (int)(darkMode ? 1 : 0);
    root["holdBucket"]       = (int)(holdBucket ? 1 : 0);
    root["holdAux"]          = (int)(holdAux ? 1 : 0);
    root["horizontalScreen"] = (int)(horScreen ? 1 : 0);
    root["RecordTelemetry"]  = (int)(tlmEnabled ? 1 : 0);
    root["SystemSounds"]     = (int)(sSndEnabled ? 1 : 0);
    root["SystemVolume"]     = (int)sSndVolume;

    String payload;
    serializeJson(doc, payload);

    AsyncWebServerResponse *resp = request->beginResponse(200, "application/json", payload);
    resp->addHeader("Cache-Control", "no-store");
    request->send(resp);
  });


  // ---------- /list_sd_files (GET -> JSON, chunked) ----------
  server.on("/list_sd_files", HTTP_GET, [](AsyncWebServerRequest *request) {
    DBG_PRINTLN("📁 /list_sd_files requested (INDEX)");

    int start = request->hasParam("start") ? request->getParam("start")->value().toInt() : 0;
    int count = request->hasParam("count") ? request->getParam("count")->value().toInt() : 40;
    if (count < 1 || count > 256) count = 40;

    String path = request->hasParam("path") ? request->getParam("path")->value() : "/";
    if (!path.startsWith("/")) path = "/" + path;
    if (path.endsWith("/") && path.length() > 1) path.remove(path.length() - 1);

    bool showSystem = request->hasParam("showSystem") && (request->getParam("showSystem")->value().toInt() != 0);

    // -- Auto-clear pendingReindex if index is present now
    {
      SdLock lk;
      if (pendingReindex && SD.exists(path + "/.index")) {
        pendingReindex = false;
        DBG_PRINTLN("[Auto-clear] pendingReindex reset by file presence");
      }
    }

    // -- If folder doesn't exist: return []
    {
      SdLock lk;
      if (!SD.exists(path)) {
        auto *resp = new AsyncJsonResponse(false);      // array
        resp->getRoot().to<JsonArray>();                // []
        resp->setCode(200);
        resp->setLength();
        request->send(resp);
        return;
      }
    }

    // -- If index missing OR reindex pending
    bool needIndex = false;
    {
      SdLock lk;
      needIndex = (!SD.exists(path + "/.index") || pendingReindex);
    }
    if (needIndex) {
      bool isEmpty = false;
      {
        SdLock lk;
        File dir = SD.open(path);
        if (dir && dir.isDirectory()) {
          File f = dir.openNextFile();
          if (!f) isEmpty = true;
          else f.close();
          dir.close();
        }
      }

      if (isEmpty) {
        auto *resp = new AsyncJsonResponse(false);      // array
        resp->getRoot().to<JsonArray>();                // []
        resp->setCode(200);
        resp->setLength();
        request->send(resp);
        return;
      }

      if (!pendingReindex) {
        reindexPath  = path;
        pendingReindex = true;
        reindexCount  = 0;
      }

      auto *resp = new AsyncJsonResponse();             // object
      resp->getRoot()["status"] = "reindexing";
      resp->setCode(202);
      resp->setLength();
      request->send(resp);
      return;
    }

    // -- Index exists: read a page
    auto *resp = new AsyncJsonResponse(false);          // array
    JsonArray arr = resp->getRoot().to<JsonArray>();

    // readSdIndexBatch() already locks & yields
    readSdIndexBatch(path + "/.index", start, count, arr, showSystem);

    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });


  // ---------- /sd_reindex (POST -> JSON) ----------
  server.on("/sd_reindex", HTTP_POST, [](AsyncWebServerRequest *request){
    String path = "/";

    // accept ?path=... (query) or body param
    if (request->hasParam("path", true))        path = urlDecode(request->getParam("path", true)->value());
    else if (request->hasParam("path", false))  path = urlDecode(request->getParam("path", false)->value());
    if (!path.startsWith("/")) path = "/" + path;
    while (path.indexOf("//") >= 0) path.replace("//","/");
    if (path.endsWith("/") && path.length() > 1) path.remove(path.length()-1);

    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    // 404 if path doesn't exist
    {
      SdLock lock;
      if (!SD.exists(path)) {
        root["ok"]    = false;
        root["error"] = "not_found";
        root["path"]  = path;
        resp->setCode(404);
        resp->setLength();
        request->send(resp);
        return;
      }
    }

    if (pendingReindex) {
      root["ok"]    = false;
      root["error"] = "already_in_progress";
      root["path"]  = path;
      resp->setCode(409);
      resp->setLength();
      request->send(resp);
      return;
    }

    // kick off background reindex (your loop/task should consume these)
    reindexPath      = path;
    pendingReindex   = true;
    reindexCount     = 0;          // how many files processed so far
    reindexTotal     = 0;          // optional: fill later when you count
    reindexCounting  = true;       // optional flag if you use it

    root["ok"]     = true;
    root["status"] = "started";
    root["path"]   = path;
    resp->setCode(202);
    resp->setLength();             // ← required
    request->send(resp);
  });

  // ---------- /sd_reindex_status (GET -> JSON) ----------
  server.on("/sd_reindex_status", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncJsonResponse *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    root["pending"]  = pendingReindex;
    root["path"]     = String(reindexPath);
    root["count"]    = reindexCount;
    root["total"]    = reindexTotal;
    root["counting"] = reindexCounting;

    resp->setCode(200);
    resp->setLength();             // ← required
    request->send(resp);
  });

  // ---------- /sd_info (GET -> JSON) ----------
  server.on("/sd_info", HTTP_GET, [](AsyncWebServerRequest *request){
      uint64_t total = SD.totalBytes();
      uint64_t used = SD.usedBytes();
      uint64_t free = total - used;

      String json = "{\"total\":" + String(total) +
                    ",\"used\":" + String(used) +
                    ",\"free\":" + String(free) + "}";
      request->send(200, "application/json", json);
  });

  // ---------- /download_sd (GET -> file download) ----------
  server.on("/download_sd", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("path")) {
      auto *resp = new AsyncJsonResponse();
      resp->getRoot()["error"] = "missing_path";
      resp->setCode(400);
      resp->setLength();
      request->send(resp);
      return;
    }
    String path = urlDecode(request->getParam("path")->value());
    if (!path.startsWith("/")) path = "/" + path;

    {
      SdLock lock;
      if (!SD.exists(path)) {
        auto *resp = new AsyncJsonResponse();
        resp->getRoot()["error"] = "not_found";
        resp->getRoot()["path"]  = path;
        resp->setCode(404);
        resp->setLength();
        request->send(resp);
        return;
      }
    }
    request->send(SD, path, String(), true); // attachment
  });

  // ---------- /recover_sd (GET -> JSON) ----------
  server.on("/recover_sd", HTTP_GET, [](AsyncWebServerRequest *request) {
    SdLock lock;

    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("name")) {
      root["ok"] = false; root["error"] = "missing_name";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    String name = urlDecode(request->getParam("name")->value());
    String recycleFile = "/recycle/" + name;
    String pathFile    = recycleFile + ".path";
    String dst         = "/" + name;

    if (SD.exists(pathFile)) {
      File f = SD.open(pathFile, FILE_READ);
      if (f) { String orig = f.readString(); f.close(); orig.trim(); if (orig.startsWith("/")) dst = orig; }
    }

    if (SD.exists(dst)) {
      root["ok"] = false; root["error"] = "dest_exists"; root["dest"] = dst;
      resp->setCode(409); resp->setLength(); request->send(resp); return;
    }

    int lastSlash = dst.lastIndexOf('/');
    if (lastSlash > 0) {
      String folderPath = dst.substring(0, lastSlash);
      if (!SD.exists(folderPath)) SD.mkdir(folderPath.c_str());
    }

    if (SD.rename(recycleFile, dst)) {
      if (SD.exists(pathFile)) SD.remove(pathFile);
      if (SD.exists("/recycle/.index")) SD.remove("/recycle/.index");
      String folder = dst.substring(0, dst.lastIndexOf('/')); if (folder == "") folder = "/";
      String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);

      root["ok"] = true; root["file"] = name; root["path"] = dst;
      resp->setCode(200);
    } else {
      root["ok"] = false; root["error"] = "recover_failed";
      resp->setCode(500);
    }
    resp->setLength(); request->send(resp);
  });

  // ---------- /permadelete_sd (POST -> JSON) ----------
  server.on("/permadelete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
    SdLock lock;

    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("path")) {
      root["ok"] = false; root["error"] = "missing_path";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    String path = urlDecode(request->getParam("path")->value());
    if (!path.startsWith("/")) path = "/" + path;

    if (!SD.exists(path)) {
      root["ok"] = false; root["error"] = "not_found"; root["path"] = path;
      resp->setCode(404); resp->setLength(); request->send(resp); return;
    }

    File f = SD.open(path);
    if (!f) { root["ok"]=false; root["error"]="open_failed"; root["path"]=path;
      resp->setCode(404); resp->setLength(); request->send(resp); return; }
    bool isDir = f.isDirectory(); f.close();

    bool ok = isDir ? SD.rmdir(path.c_str()) : SD.remove(path.c_str());
    if (ok) {
      String folder = path.substring(0, path.lastIndexOf('/')); if (folder == "") folder = "/";
      String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);
      root["ok"] = true; root["path"] = path; root["type"] = isDir ? "dir" : "file";
      resp->setCode(200);
    } else {
      root["ok"] = false; root["error"] = "delete_failed";
      resp->setCode(500);
    }
    resp->setLength(); request->send(resp);
  });

  // ---------- /delete_sd (POST -> JSON) ----------
  server.on("/delete_sd", HTTP_POST, [](AsyncWebServerRequest *request) {
    SdLock lock;

    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    String originalPath;
    bool permanent = false;

    if (request->hasParam("permanent")) {
      String perm = request->getParam("permanent")->value();
      permanent = (perm == "1" || perm == "true" || perm == "yes");
    } else if (request->hasArg("permanent")) {
      String perm = request->arg("permanent");
      permanent = (perm == "1" || perm == "true" || perm == "yes");
    }

    if (request->hasParam("path"))        originalPath = urlDecode(request->getParam("path")->value());
    else if (request->hasArg("path"))     originalPath = urlDecode(request->arg("path"));
    else if (request->contentType().startsWith("application/x-www-form-urlencoded")) {
      String body = request->arg(0);
      int idx = body.indexOf("path=");
      if (idx != -1) {
        originalPath = body.substring(idx + 5);
        int amp = originalPath.indexOf('&');
        if (amp != -1) originalPath = originalPath.substring(0, amp);
        originalPath.replace('+', ' ');
        originalPath = urlDecode(originalPath);
      }
    }

    if (!originalPath.length()) {
      root["ok"] = false; root["error"] = "missing_path";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    if (!originalPath.startsWith("/")) originalPath = "/" + originalPath;
    if (!SD.exists(originalPath)) {
      root["ok"] = false; root["error"] = "not_found"; root["path"] = originalPath;
      resp->setCode(404); resp->setLength(); request->send(resp); return;
    }

    if (permanent) {
      if (SD.remove(originalPath)) {
        String folder = originalPath.substring(0, originalPath.lastIndexOf('/')); if (folder == "") folder = "/";
        String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);
        root["ok"] = true; root["mode"] = "permanent"; root["path"] = originalPath;
        resp->setCode(200);
      } else {
        root["ok"] = false; root["error"] = "permanent_failed";
        resp->setCode(500);
      }
      resp->setLength(); request->send(resp); return;
    }

    // Recycle flow
    if (!SD.exists("/recycle")) SD.mkdir("/recycle");
    String filename = originalPath.substring(originalPath.lastIndexOf("/") + 1);
    String recyclePath = "/recycle/" + filename;
    int count = 1;
    String testRecyclePath = recyclePath;
    while (SD.exists(testRecyclePath)) {
      int dot = filename.lastIndexOf('.');
      String base = (dot >= 0) ? filename.substring(0, dot) : filename;
      String ext  = (dot >= 0) ? filename.substring(dot) : "";
      testRecyclePath = "/recycle/" + base + "_" + String(count++) + ext;
    }
    recyclePath = testRecyclePath;

    // Save metadata
    String pathMetaFile = recyclePath + ".path";
    { File meta = SD.open(pathMetaFile, FILE_WRITE); if (meta) { meta.print(originalPath); meta.close(); } }

    if (SD.rename(originalPath, recyclePath)) {
      String folder = originalPath.substring(0, originalPath.lastIndexOf('/')); if (folder == "") folder = "/";
      String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);
      root["ok"] = true; root["mode"] = "recycle"; root["original"] = originalPath; root["recyclePath"] = recyclePath;
      resp->setCode(200);
    } else {
      if (SD.exists(pathMetaFile)) SD.remove(pathMetaFile);
      root["ok"] = false; root["error"] = "move_failed";
      resp->setCode(500);
    }
    resp->setLength(); request->send(resp);
  });

  // /upload_sd  — safe upload: write to temp, then atomically replace on success (+ optional sha256)
  server.on("/upload_sd", HTTP_POST,

    // Final JSON response
    [](AsyncWebServerRequest *request) {
      auto *ctx = reinterpret_cast<UploadCtx*>(request->_tempObject);

      auto *resp = new AsyncJsonResponse();
      if (!ctx || ctx->error.length()) {
        resp->getRoot()["ok"]    = false;
        resp->getRoot()["error"] = ctx ? ctx->error : "unknown";
        resp->setCode(500);
      } else {
        resp->getRoot()["ok"]     = true;
        resp->getRoot()["path"]   = ctx->uploadPath;
        resp->getRoot()["bytes"]  = (int)ctx->bytesWritten;
        if (ctx->verifySha) {
          resp->getRoot()["sha256"] = toHexLower(ctx->digest, sizeof(ctx->digest));
          resp->getRoot()["sha_ok"] = true;
        }
        resp->setCode(200);
      }
      resp->setLength();
      request->send(resp);

      if (ctx && ctx->error.length() == 0) playSystemSound("/web/pcm/click.wav");
      else                                  playSystemSound("/web/pcm/error.wav");

      delete ctx; request->_tempObject = nullptr;
    },

    // Chunk handler
    [](AsyncWebServerRequest *request, String filename, size_t index,
      uint8_t *data, size_t len, bool final) {

      auto *ctx = reinterpret_cast<UploadCtx*>(request->_tempObject);

      if (index == 0) {
        if (!ctx) { ctx = new UploadCtx(); request->_tempObject = (void*)ctx; }

        // Resolve final destination path
        String uploadPath;
        if (request->hasParam("path", true))       uploadPath = urlDecode(request->getParam("path", true)->value());
        else if (request->hasParam("path", false)) uploadPath = urlDecode(request->getParam("path", false)->value());
        else                                       uploadPath = "/" + filename;
        if (!uploadPath.startsWith("/")) uploadPath = "/" + uploadPath;
        ctx->uploadPath = uploadPath;

        // Optional sha256 (query/body)
        String shaHex;
        if (request->hasParam("sha256", true))       shaHex = request->getParam("sha256", true)->value();
        else if (request->hasParam("sha256", false)) shaHex = request->getParam("sha256", false)->value();
        shaHex = normHex64(shaHex);
        if (shaHex.length() == 64) {
          ctx->verifySha = true;
          ctx->expectedShaHex = shaHex;
          mbedtls_sha256_init(&ctx->sha);
          mbedtls_sha256_starts_ret(&ctx->sha, 0 /* 0 = SHA-256, 1 = SHA-224 */);
        }

        DBG_PRINTF(">>> Starting upload: %s (verify=%d)\n", ctx->uploadPath.c_str(), ctx->verifySha);

        // Ensure directory exists for final & temp
        { SdLock lock; ensureFolderExists(ctx->uploadPath); }

        // Build temp path in same directory, hidden-ish, unique
        String dir  = dirName(ctx->uploadPath);
        String name = baseName(ctx->uploadPath);
        { SdLock lock;
          String tmpWant = dir + "/." + name + ".upload.tmp";
          ctx->tmpPath = SD.exists(tmpWant) ? uniqueInDir(dir, "." + name + ".upload.tmp") : tmpWant;
        }

        // Open temp for write. Do NOT touch the final yet.
        { SdLock lock; ctx->uploadFile = SD.open(ctx->tmpPath, FILE_WRITE); }
        if (!ctx->uploadFile) { ctx->error = "open_tmp_failed"; return; }
      }

      // Stream chunk -> temp, with periodic flush + tiny yield, and update SHA if enabled
      // Robust, throttled streaming write (chunked + retries + reopen)
      if (ctx && ctx->uploadFile && len && ctx->error.length() == 0) {
        const size_t CHUNK       = 4096;   // write granularity
        const int    RETRIES     = 4;      // attempts per write before escalate
        const size_t FLUSH_EVERY = 8192;   // periodic flush/yield

        size_t off = 0;
        while (off < len) {
          size_t toWrite = len - off;
          if (toWrite > CHUNK) toWrite = CHUNK;

          size_t wrote = 0;
          int attempts = 0;

          // Try a few times, with tiny backoff
          while (attempts < RETRIES && wrote == 0) {
            { SdLock lock; wrote = ctx->uploadFile.write(data + off, toWrite); }
            if (wrote == 0) {
              { SdLock lock; ctx->uploadFile.flush(); }
              sdTinyYield(3 + attempts * 2);   // backoff 3ms,5ms,7ms,9ms
            }
            attempts++;
          }

          // Last resort: reopen file and try again
          if (wrote == 0) {
            { SdLock lock; ctx->uploadFile.close(); }
            sdTinyYield(10);
            { SdLock lock; ctx->uploadFile = SD.open(ctx->tmpPath, FILE_APPEND); }
            if (!ctx->uploadFile) {
              ctx->error = "sd_reopen_failed";
              DBG_PRINTF("!!! SD reopen failed at index=%u\n", (unsigned)index);
            } else {
              attempts = 0;
              while (attempts < RETRIES && wrote == 0) {
                { SdLock lock; wrote = ctx->uploadFile.write(data + off, toWrite); }
                if (wrote == 0) {
                  { SdLock lock; ctx->uploadFile.flush(); }
                  sdTinyYield(4 + attempts * 2);
                }
                attempts++;
              }
            }
          }

          // Still failed → abort safely (keep original file intact)
          if (wrote == 0 || ctx->error.length() != 0) {
            if (ctx->error.length() == 0) ctx->error = "sd_write_error";
            { SdLock lock; if (ctx->uploadFile) ctx->uploadFile.close(); }
            { SdLock lock; SD.remove(ctx->tmpPath); }
            DBG_PRINTF("!!! SD write error at index=%u\n", (unsigned)index);
            break;
          }

          // Hash exactly what we wrote
          if (ctx->verifySha) mbedtls_sha256_update_ret(&ctx->sha, data + off, wrote);

          off += wrote;
          ctx->bytesWritten    += wrote;
          ctx->bytesSinceYield += wrote;

          // Periodic flush + breather
          if (ctx->bytesSinceYield >= FLUSH_EVERY) {
            { SdLock lock; ctx->uploadFile.flush(); }
            sdTinyYield(2);
            ctx->bytesSinceYield = 0;
          }
        }

        // If we set an error above, bail so the final block skips promotion
        if (ctx->error.length() != 0) return;
      }


      if (final) {
        // Close temp file first, then give SD a short pause
        {
          SdLock lock;
          if (ctx && ctx->uploadFile) { ctx->uploadFile.flush(); ctx->uploadFile.close(); }
        }
        sdTinyYield(5);   // final breather before sha/rename

        // Finalize SHA and verify (if requested)
        if (ctx && ctx->verifySha && ctx->error.length() == 0) {
          mbedtls_sha256_finish_ret(&ctx->sha, ctx->digest);
          mbedtls_sha256_free(&ctx->sha);
          String got = toHexLower(ctx->digest, sizeof(ctx->digest));
          if (got != ctx->expectedShaHex) {
            ctx->error = "sha256_mismatch";
            DBG_PRINTF("!!! SHA256 mismatch: got=%s expected=%s\n",
                      got.c_str(), ctx->expectedShaHex.c_str());
            SdLock lock; if (SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
          }
        }

        if (ctx && ctx->error.length() == 0) {
          SdLock lock;

          // 1) Move existing FINAL to /recycle (only now)
          if (SD.exists(ctx->uploadPath)) {
            String recDir = "/recycle";
            if (!SD.exists(recDir)) SD.mkdir(recDir);
            String recPath = uniqueInDir(recDir, baseName(ctx->uploadPath));
            if (!SD.rename(ctx->uploadPath, recPath)) {
              ctx->error = "rename_to_recycle_failed";
              DBG_PRINTF("!!! Failed to move old file to recycle: %s\n", recPath.c_str());
            } else {
              DBG_PRINTF("Moved old file to recycle: %s\n", recPath.c_str());
            }
          }

          // 2) Promote temp -> final if OK
          if (ctx->error.length() == 0) {
            if (!SD.rename(ctx->tmpPath, ctx->uploadPath)) {
              ctx->error = "rename_tmp_to_final_failed";
              DBG_PRINTF("!!! Failed to promote temp to final: %s -> %s\n",
                        ctx->tmpPath.c_str(), ctx->uploadPath.c_str());
            } else {
              DBG_PRINTF("<<< Upload finished. Promoted %s -> %s (bytes=%u)\n",
                        ctx->tmpPath.c_str(), ctx->uploadPath.c_str(), (unsigned)ctx->bytesWritten);

              // 3) Invalidate .index in the same folder
              String folder = dirName(ctx->uploadPath);
              if (folder == "") folder = "/";
              String idxPath = folder + "/.index";
              if (SD.exists(idxPath)) SD.remove(idxPath);
            }
          }

          // 4) Cleanup temp if anything failed
          if (ctx->error.length() != 0 && SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
        } else {
          // Had error: ensure temp removed
          SdLock lock;
          if (ctx && SD.exists(ctx->tmpPath)) SD.remove(ctx->tmpPath);
        }
        // JSON goes out via the first lambda
      }
    }
  );

  server.on("/create_file", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("path")) {
      root["ok"]=false; root["error"]="missing_path";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }
    String path = urlDecode(request->getParam("path")->value());
    if (!path.startsWith("/")) path = "/" + path;

    {
      SdLock lock;
      path = ensureUniqueFilename(path);
      File file = SD.open(path, FILE_WRITE);
      if (!file) {
        root["ok"]=false; root["error"]="create_failed"; root["path"]=path;
        resp->setCode(500); resp->setLength(); request->send(resp); return;
      }
      file.close();
      String folder = path.substring(0, path.lastIndexOf('/')); if (folder == "") folder="/";
      String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);
    }

    root["ok"]=true; root["path"]=path;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  server.on("/create_folder", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("path")) {
      root["ok"]=false; root["error"]="missing_path";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }

    String path = urlDecode(request->getParam("path")->value());
    if (!path.startsWith("/")) path = "/" + path;

    String newPath;
    {
      SdLock lock;
      String base = path; newPath = path; int count = 1;
      while (SD.exists(newPath.c_str())) newPath = base + "(" + String(count++) + ")";
      if (SD.mkdir(newPath.c_str())) {
        String folder = newPath.substring(0, newPath.lastIndexOf('/')); if (folder=="") folder="/";
        String idxPath = folder + "/.index"; if (SD.exists(idxPath)) SD.remove(idxPath);
        root["ok"]=true; root["path"]=newPath; resp->setCode(200);
      } else {
        root["ok"]=false; root["error"]="mkdir_failed"; root["path"]=newPath; resp->setCode(500);
      }
    }
    resp->setLength();
    request->send(resp);
  });

  server.on("/reboot", HTTP_POST, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    resp->getRoot()["ok"] = true;
    resp->getRoot()["status"] = "rebooting";
    resp->setCode(200);
    resp->setLength();
    request->send(resp);

    delay(100);
    playSystemSound("/web/pcm/reboot.wav");
    delay(2000);
    resetESP();
  });

  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    sensor_t *s = esp_camera_sensor_get();
    if (!s) { root["ok"]=false; root["error"]="no_sensor"; resp->setCode(500); resp->setLength(); request->send(resp); return; }

    bool changed=false;
    auto updatePref = [&](const char* key, int value){ if (camPrefs.getInt(key, INT32_MIN)!=value){ camPrefs.putInt(key,value); changed=true; } };

    if (request->hasParam("res"))  { int v=request->getParam("res")->value().toInt();  if (camPrefs.getInt("camRes",FRAMESIZE_QVGA)!=v) camPrefs.putInt("camRes", v); }
    if (request->hasParam("fps"))  { int v=request->getParam("fps")->value().toInt();  updatePref("camFps", v); s->set_quality(s, v); }
    if (request->hasParam("rot"))  { int v=request->getParam("rot")->value().toInt();  updatePref("camRot", v); s->set_hmirror(s,(v==1||v==3)); s->set_vflip(s,(v==2||v==3)); }
    if (request->hasParam("sat"))  { int v=request->getParam("sat")->value().toInt();  updatePref("camSat", v); s->set_saturation(s, v); }
    if (request->hasParam("gray")) { int v=request->getParam("gray")->value().toInt(); updatePref("camGray", v); s->set_special_effect(s, v?2:0); }
    if (request->hasParam("led"))  { int v=request->getParam("led")->value().toInt();  updatePref("camLed", v); }
    if (request->hasParam("bright")){int v=request->getParam("bright")->value().toInt();updatePref("camBright", v); s->set_brightness(s, v); }
    if (request->hasParam("contrast")){int v=request->getParam("contrast")->value().toInt();updatePref("camContrast", v); s->set_contrast(s, v); }
    if (request->hasParam("sharp")){ int v=request->getParam("sharp")->value().toInt(); updatePref("camSharp", v); if (s->id.PID==OV2640_PID) s->set_sharpness(s, v); }
    if (request->hasParam("denoise")){int v=request->getParam("denoise")->value().toInt();updatePref("camDenoise", v); if (s->id.PID==OV2640_PID && s->set_denoise) s->set_denoise(s, v); }
    if (request->hasParam("gamma")) { int v=request->getParam("gamma")->value().toInt(); updatePref("camGamma", v); }
    if (request->hasParam("compression")){int v=request->getParam("compression")->value().toInt();updatePref("camCompression", v); if (s->id.PID==OV2640_PID && s->set_quality) s->set_quality(s, v); }
    if (request->hasParam("quality")){int v=request->getParam("quality")->value().toInt(); updatePref("camQuality", v); s->set_quality(s, v); }

    root["ok"]=true; root["changed"]=changed;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    String path = request->url();
    int q = path.indexOf('?');                 // strip query for FS lookup (e.g. /main.css?v=123)
    if (q >= 0) path = path.substring(0, q);

    // /telemetry/* (as CSV)
    if (path.startsWith("/telemetry/")) {
      SdLock lock;
      if (SD.exists(path)) { request->send(SD, path, "text/csv"); return; }
      request->send(404, "text/plain", "File Not Found");
      return;
    }

    // /media/* (whole file; no manual SD.open => no FD leak)
    if (path.startsWith("/media/")) {
      SdLock lock;
      if (!SD.exists(path)) { request->send(404, "text/plain", "File not found"); return; }
      auto* resp = request->beginResponse(SD, path, getContentType(path), /*download=*/true);
      resp->addHeader("Accept-Ranges", "bytes");
      request->send(resp);
      return;
    }

    // Everything else from /web
    if (path == "/") path = "/index.html";

    // Accept "/img/..." OR already prefixed "/web/..."
    String webPath = path.startsWith("/web/") ? path : ("/web" + path);

    // Prefer precompressed .gz if present
    SdLock lock;
    String tryPath = webPath;
    if (!SD.exists(tryPath)) {
      String gz = webPath + ".gz";
      if (SD.exists(gz)) tryPath = gz;
    }

    if (SD.exists(tryPath)) {
      auto* resp = request->beginResponse(SD, tryPath, String(), /*download=*/false);
      resp->addHeader("Cache-Control", "public, max-age=86400");   // reduce repeated opens
      request->send(resp);
    } else if (SD.exists("/web/404.html")) {
      request->send(SD, "/web/404.html", "text/html");
    } else {
      request->send(404, "text/plain", "File Not Found");
    }
  });

  // ---------- /list_media_files (GET -> JSON, chunked) ----------
  server.on("/list_media_files", HTTP_GET, [](AsyncWebServerRequest *request){
    DBG_PRINTF(">> Handler start: %s, heap=%u\n", __func__, ESP.getFreeHeap());

    // Pagination
    const int start = request->hasParam("start") ? request->getParam("start")->value().toInt() : 0;
    int count       = request->hasParam("count") ? request->getParam("count")->value().toInt() : 40;
    if (count < 1 || count > 256) count = 40;

    // If any folder needs indexing -> trigger and return light JSON
    for (const auto &folder : mediaFolders) {              // folder is const char*
      String idxPath = String(folder) + "/.index";
      idxPath.replace("//","/");
      bool hasIndex = false;
      { SdLock lk; hasIndex = SD.exists(idxPath); }
      if (!hasIndex) {
        { SdLock lk; if (!pendingReindex) { reindexPath = folder; pendingReindex = true; reindexCount = 0; } }
        auto *resp = new AsyncJsonResponse();
        auto root  = resp->getRoot().to<JsonObject>();
        root["status"] = "reindexing";
        root["path"]   = folder;                           // const char*
        resp->setCode(200);
        resp->setLength();
        request->send(resp);
        DBG_PRINTF("<< Handler end (reindexing %s): heap=%u\n", folder, ESP.getFreeHeap());
        return;
      }
    }

    auto *resp  = new AsyncJsonResponse();                 // object root
    JsonObject root = resp->getRoot().to<JsonObject>();
    JsonArray files = root.createNestedArray("files");

    // helper: case-insensitive media ext check
    auto isMedia = [](const String &name) -> bool {
      int dot = name.lastIndexOf('.');
      if (dot < 0) return false;
      String ext = name.substring(dot + 1);
      ext.toLowerCase();
      return ext == "mp3" || ext == "wav" || ext == "ogg" ||
            ext == "mp4" || ext == "webm"|| ext == "mov";
    };

    int total = 0;
    int emitted = 0;
    uint32_t tick = 0;

    for (const auto &folder : mediaFolders) {
      String idxPath = String(folder) + "/.index";
      idxPath.replace("//","/");

      SdLock lk;                                           // hold lock while reading index
      File idx = SD.open(idxPath, FILE_READ);
      if (!idx) continue;

      String line;
      while (idx.available()) {
        line = idx.readStringUntil('\n');
        line.trim();
        if (!line.length()) { if (((++tick) & 0x7F) == 0) delay(0); continue; }

        // parse "name,isFolder,size[,date]"
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        if (c1 < 0 || c2 < 0) { if (((++tick) & 0x7F) == 0) delay(0); continue; }

        bool isFolder = line.substring(c1 + 1, c2).toInt();
        if (isFolder) { if (((++tick) & 0x7F) == 0) delay(0); continue; }

        String name = line.substring(0, c1);
        if (!isMedia(name)) { if (((++tick) & 0x7F) == 0) delay(0); continue; }

        // count every matching file
        total++;

        // emit only requested window
        if (total > start && emitted < count) {
          String full = String(folder) + "/" + name;
          full.replace("//","/");
          files.add(full);
          emitted++;
        }

        if (((++tick) & 0x7F) == 0) delay(0);             // cooperative yield
      }
      idx.close();
      // still under SdLock scope; it will release here
      delay(0);                                            // tiny gap between folders
    }

    root["start"] = start;
    root["count"] = emitted;
    root["total"] = total;

    resp->setCode(200);
    resp->setLength();                                     // avoid chunked
    request->send(resp);

    DBG_PRINTF("<< Handler end: %s, heap=%u\n", __func__, ESP.getFreeHeap());
  });

  // ---------- /play_on_device (GET -> JSON) ----------
  server.on("/play_on_device", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("file")) {
      root["ok"]=false; root["error"]="missing_file";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }
    String file = urlDecode(request->getParam("file")->value());

    if (!hasSupportedExtension(file)) {
      root["ok"]=false; root["error"]="unsupported_type"; root["file"]=file;
      resp->setCode(403); resp->setLength(); request->send(resp); return;
    }

    { SdLock lock; if (!SD.exists(file)) {
        root["ok"]=false; root["error"]="not_found"; root["file"]=file;
        resp->setCode(404); resp->setLength(); request->send(resp); return;
      }
    }

    DBG_PRINTF("[MEDIA] Switching to speaker, playing: %s\n", file.c_str());
    playWavFileOnSpeaker(file);

    root["ok"]=true; root["status"]="playing"; root["file"]=file;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /stop_playback (GET -> text) ----------
  server.on("/stop_playback", HTTP_GET, [](AsyncWebServerRequest *request){
    DBG_PRINTLN("[AUDIO] stop_playback called");
    stopAudio();                       // your function that stops speaker I2S
    request->send(200, "text/plain", "OK");
  });
  
  // ---------- /enable_mic (GET -> text) ----------
  server.on("/enable_mic", HTTP_GET, [](AsyncWebServerRequest *request){
    DBG_PRINTLN("[MIC] enable_mic called");
    enableMic();                          // your function
    request->send(200, "text/plain", "OK");
  });

  // ---------- /disable_mic (GET -> text) ----------
  server.on("/disable_mic", HTTP_GET, [](AsyncWebServerRequest *request){
    DBG_PRINTLN("[MIC] disable_mic called (flagging stream stop)");
    micStreamActive = false;
    request->send(200, "text/plain", "OK");
  });

  server.on("/mic_stream", HTTP_GET, [](AsyncWebServerRequest *request){
    struct MicState { bool headerSent = false; };
    auto *state = new MicState();

    micStreamActive = true;

    auto *response = request->beginChunkedResponse("audio/wav",
      [state](uint8_t *buffer, size_t maxLen, size_t /*index*/) -> size_t {
        if (!state->headerSent) {
          uint8_t wavHeader[44];
          makeWavHeader(wavHeader, 16000, 1, 16, 0x7FFFFFFF);
          memcpy(buffer, wavHeader, 44);
          state->headerSent = true;
          return 44;
        }

        if (!micStreamActive) {
          // return 0 once -> AsyncWebServer will close the response cleanly
          return 0;
        }

        size_t bytesRead = 0;
        int16_t audioBuffer[I2S_READ_LEN];
        esp_err_t res = i2s_read(MY_I2S_PORT, (void*)audioBuffer, sizeof(audioBuffer),
                                &bytesRead, 30 / portTICK_RATE_MS);
        if (res != ESP_OK || bytesRead == 0) {
          // brief yield to avoid tight loop on transient errors
          delay(0);
          return 0; // end the stream; client can reconnect
        }

        memcpy(buffer, audioBuffer, bytesRead);
        // Cooperative yield every N chunks (optional)
        return bytesRead;
      }
    );
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  // ---------- /set_volume (GET -> JSON) ----------
  server.on("/set_volume", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    if (!request->hasParam("value")) {
      root["ok"]=false; root["error"]="missing_value";
      resp->setCode(400); resp->setLength(); request->send(resp); return;
    }
    int v = request->getParam("value")->value().toInt();
    if (v < 0) v = 0; if (v > 21) v = 21;
    audio.setVolume(v);

    root["ok"]=true; root["volume"]=v;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /get_oled_settings (GET -> JSON) ----------
  server.on("/get_oled_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();
    root["layout"] = oledPrefs.getString("layout", "default");
    root["showIP"] = oledPrefs.getBool("showIP", true);
    root["showBattery"] = oledPrefs.getBool("showBattery", true);
    root["showWiFi"] = oledPrefs.getBool("showWiFi", true);
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /set_oled_settings (POST JSON -> save) ----------
  {
    auto *setOLED = new AsyncCallbackJsonWebHandler(
      "/set_oled_settings",
      [](AsyncWebServerRequest *request, JsonVariant &json){
        if (!json.is<JsonObject>()) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["error"] = "Invalid JSON (object expected)";
          err->setCode(400); err->setLength(); request->send(err); return;
        }

        JsonObject obj = json.as<JsonObject>();
        String layout      = obj.containsKey("layout")      ? obj["layout"].as<String>() : "default";
        bool   showIP      = obj.containsKey("showIP")      ? obj["showIP"].as<bool>()   : true;
        bool   showBattery = obj.containsKey("showBattery") ? obj["showBattery"].as<bool>() : true;
        bool   showWiFi    = obj.containsKey("showWiFi")    ? obj["showWiFi"].as<bool>() : true;

        oledPrefs.putString("layout", layout);
        oledPrefs.putBool("showIP", showIP);
        oledPrefs.putBool("showBattery", showBattery);
        oledPrefs.putBool("showWiFi", showWiFi);

        auto *ok = new AsyncJsonResponse();
        ok->getRoot()["status"] = "OK";
        ok->setCode(200);
        ok->setLength();
        request->send(ok);
      }
    );
    setOLED->setMethod(HTTP_POST);
    setOLED->setMaxContentLength(512);
    server.addHandler(setOLED);
  }

  // ---------- /upload_oled_anim (POST -> upload file, JSON ack) ----------
  server.on("/upload_oled_anim", HTTP_POST,
    [](AsyncWebServerRequest *request){ /* response sent in upload lambda */ },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
      static File uploadFile;

      if (index == 0) {
        DBG_PRINTF("📤 Uploading OLED animation file: %s\n", filename.c_str());
        SdLock lock;
        uploadFile = SD.open("/oled_anim.xbm", FILE_WRITE);
        if (!uploadFile) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["ok"]=false; err->getRoot()["error"]="open_failed";
          err->setCode(500); err->setLength(); request->send(err); return;
        }
      }

      if (uploadFile) {
        if (len) {
          SdLock lock;
          if (uploadFile.write(data, len) != len) {
            uploadFile.close();
            auto *err = new AsyncJsonResponse();
            err->getRoot()["ok"]=false; err->getRoot()["error"]="write_failed";
            err->setCode(500); err->setLength(); request->send(err); return;
          }
        }
        if (final) {
          SdLock lock; uploadFile.close();
          auto *ok = new AsyncJsonResponse();
          ok->getRoot()["ok"]=true; ok->getRoot()["path"]="/oled_anim.xbm";
          ok->setCode(200); ok->setLength(); request->send(ok);
        }
      }
    }
  );

  // ---------- /list_telemetry_files (GET -> JSON, chunked) ----------
  server.on("/list_telemetry_files", HTTP_GET, [](AsyncWebServerRequest *request){
      AsyncJsonResponse *resp = new AsyncJsonResponse(false); // chunked
      JsonArray arr = resp->getRoot().to<JsonArray>();

      {
          SdLock lock; // 🔒 Ensure SD card is safe to access
          File dir = SD.open("/telemetry");
          if (dir && dir.isDirectory()) {
              for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
                  String name = f.name();
                  if (name.endsWith(".csv")) {
                      if (!name.startsWith("/")) name = "/telemetry/" + name;
                      arr.add(name);
                  }
              }
              dir.close();
          }
      } // lock released here

      resp->setCode(200);
      request->send(resp);
  });


  //-------------Media Capture Handlers------------------------------------------//
  // ---------- /capture_photo (GET -> JSON) ----------
  server.on("/capture_photo", HTTP_GET, [](AsyncWebServerRequest *request){
    // 1) Grab a frame first (no SD yet)
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      auto *err = new AsyncJsonResponse();
      err->getRoot()["status"]  = "error";
      err->getRoot()["message"] = "Camera capture failed";
      err->setCode(500);
      err->setLength();
      request->send(err);
      return;
    }

    String filePath;
    bool   okWrite = true;

    // 2) SD write (guarded, chunked, with yields)
    {
      SdLock lock; // 🔒 Only hold SD lock while actually touching SD
      String folder = "/media/capture/photo";
      if (!SD.exists(folder)) SD.mkdir(folder);

      // If your getMediaTimestamp already returns a full path, keep as-is:
      filePath = getMediaTimestamp("photo", "jpg");  // e.g. "/media/capture/photo/2025-08-13_12-34-56.jpg"
      File f = SD.open(filePath, FILE_WRITE);
      if (!f) {
        okWrite = false;
      } else {
        const uint8_t* p   = fb->buf;
        size_t         rem = fb->len;
        while (rem && okWrite) {
          size_t n = rem > 4096 ? 4096 : rem;
          size_t w = f.write(p, n);
          if (w != n) {
            okWrite = false;
            f.close();
            SD.remove(filePath);                  // delete partial file
            break;
          }
          p   += w;
          rem -= w;
          vTaskDelay(1);                          // give time to lwIP/WiFi/etc.
        }
        if (okWrite) f.close();
      }
    } // 🔓 SD lock released here

    // 3) Return frame buffer to camera driver ASAP
    esp_camera_fb_return(fb);

    // 4) Respond
    if (!okWrite) {
      auto *err = new AsyncJsonResponse();
      err->getRoot()["status"]  = "error";
      err->getRoot()["message"] = "SD write/open failed";
      err->setCode(500);
      err->setLength();
      request->send(err);
      return;
    }

    auto *resp = new AsyncJsonResponse();
    resp->getRoot()["status"] = "ok";
    resp->getRoot()["path"]   = filePath;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);

    // 5) Play the sound AFTER all SD/camera work is done (no contention)
    playSystemSound("/web/pcm/screenshot.wav");
  });

  // ---------- /start_record_video (GET -> JSON) ----------
  server.on("/start_record_video", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    // Block duplicates (race-safe)
    if (videoTaskActive || videoRecording) {
      root["status"]  = "error";
      root["message"] = "Already recording";
      resp->setCode(409);
      resp->setLength();
      request->send(resp);
      return;
    }

    // Parse & clamp duration
    int duration = 5;
    if (request->hasParam("duration")) {
      duration = request->getParam("duration")->value().toInt();
      if (duration < 1)  duration = 1;
      if (duration > 60) duration = 60;
    }

    // Play sound BEFORE starting the task (keeps SD contention low)
    playSystemSound("/web/pcm/videorecord.wav");

    // Create task
    int* arg = new int(duration);
    BaseType_t ok =
    #if CONFIG_FREERTOS_UNICORE
        xTaskCreate(recordVideoTask, "RecordVideo", 12288, arg, 1, &videoTaskHandle);
    #else
        xTaskCreatePinnedToCore(recordVideoTask, "RecordVideo", 12288, arg, 1, &videoTaskHandle, ARDUINO_RUNNING_CORE);
    #endif

    if (ok != pdPASS) {
      delete arg;
      root["status"]  = "error";
      root["message"] = "task_create_failed";
      resp->setCode(500);
      resp->setLength();
      request->send(resp);
      return;
    }

    // Pre-arm the guard so a 2nd start can’t slip in before the task sets flags
    videoTaskActive = true;

    root["status"]   = "recording";
    root["duration"] = duration;
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---------- /stop_record_video (GET -> JSON) ----------
  server.on("/stop_record_video", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot();

    // If a stop arrives very early, treat it as success-like
    if (!videoTaskActive && !videoRecording) {
      root["status"] = "stopped";
      resp->setCode(200);
      resp->setLength();
      request->send(resp);
      return;
    }

    // Signal the task to exit; it will close the file and then play the sound
    videoRecording = false;

    // Match the frontend expectation so it doesn’t toast an error
    root["status"] = "stopped";
    resp->setCode(200);
    resp->setLength();
    request->send(resp);
  });

  // ---- /play_sd_mjpeg (per-request state; simpler loop) ----
  server.on("/play_sd_mjpeg", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("path")) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["error"] = "missing_path";
          err->setCode(400);
          request->send(err);
          return;
      }

      String path = request->getParam("path")->value();

      File f;
      {
          SdLock lock; // 🔒 Lock only while opening
          f = SD.open(path, FILE_READ);
      }

      if (!f) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["error"] = "not_found";
          err->getRoot()["path"]  = path;
          err->setCode(404);
          request->send(err);
          return;
      }

      struct MJState {
          File file;
          bool inFrame = false;
          std::vector<uint8_t> frame;
          uint8_t readBuf[4096]; // SD read buffer

          MJState(File &&f) : file(std::move(f)) {}
      };

      auto *st = new MJState(std::move(f));

      AsyncWebServerResponse *response = request->beginChunkedResponse(
          "multipart/x-mixed-replace;boundary=frame",
          [st](uint8_t *buffer, size_t maxLen, size_t /*index*/) -> size_t {
              st->frame.clear();

              // Read from SD in chunks until one JPEG frame is complete
              {
                  SdLock lock; // 🔒 Lock during SD read
                  while (st->file && st->file.available()) {
                      size_t toRead = sizeof(st->readBuf);
                      if (toRead > (size_t)st->file.available()) {
                          toRead = st->file.available();
                      }
                      size_t bytesRead = st->file.read(st->readBuf, toRead);
                      if (bytesRead == 0) break;

                      for (size_t i = 0; i < bytesRead; i++) {
                          uint8_t b = st->readBuf[i];
                          if (!st->inFrame) {
                              if (b == 0xFF && i + 1 < bytesRead && st->readBuf[i + 1] == 0xD8) {
                                  st->inFrame = true;
                                  st->frame.push_back(b);
                              }
                          } else {
                              st->frame.push_back(b);
                              size_t n = st->frame.size();
                              if (n >= 2 && st->frame[n - 2] == 0xFF && st->frame[n - 1] == 0xD9) {
                                  st->inFrame = false; // frame complete
                                  goto frame_done;
                              }
                          }

                          if (st->frame.size() >= 200000) { // sanity stop
                              st->inFrame = false;
                              goto frame_done;
                          }
                      }
                  }
              } // lock released here

          frame_done:
              if (st->frame.empty()) {
                  {
                      SdLock lock; // 🔒 Lock during file close
                      st->file.close();
                  }
                  delete st;
                  return 0; // end stream
              }

              static String head;
              head  = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
              head += st->frame.size();
              head += "\r\n\r\n";

              size_t written = 0;
              size_t hlen = head.length();
              size_t toCopy = min(maxLen, hlen);
              memcpy(buffer, head.c_str(), toCopy);
              written += toCopy;

              size_t remain = maxLen - written;
              if (remain > 0) {
                  size_t fn = min(remain, st->frame.size());
                  memcpy(buffer + written, st->frame.data(), fn);
                  written += fn;
              }

              return written;
          }
      );

      request->send(response);
  });

  // ---------- /get_mqtt (GET -> JSON) ----------
  server.on("/get_mqtt", HTTP_GET, [](AsyncWebServerRequest *request){
    DBG_PRINTLN("[MQTT] /get_mqtt requested.");
    loadMqttConfig();

    auto *resp = new AsyncJsonResponse();            // object
    JsonObject root = resp->getRoot().to<JsonObject>();
    root["enable"]       = mqttCfg.enable;
    root["host"]         = mqttCfg.host;
    root["port"]         = mqttCfg.port;
    root["user"]         = mqttCfg.user;
    root["pass"]         = mqttCfg.pass;
    root["topic_prefix"] = mqttCfg.topic_prefix;

    resp->setCode(200);
    resp->setLength();                                // ✅ avoid chunked
    request->send(resp);
  });


  // ---------- /set_mqtt (POST JSON -> save) ----------
  {
    auto *setMqtt = new AsyncCallbackJsonWebHandler(
      "/set_mqtt",
      [](AsyncWebServerRequest *request, JsonVariant &json){
        if (!json.is<JsonObject>()) {
          auto *err = new AsyncJsonResponse();
          err->getRoot()["error"] = "Invalid JSON (object expected)";
          err->setCode(400); err->setLength(); request->send(err); return;
        }
        JsonObject obj = json.as<JsonObject>();

        MQTTConfig cfg;
        cfg.enable       = obj.containsKey("enable")       ? obj["enable"].as<bool>()    : false;
        cfg.host         = obj.containsKey("host")         ? obj["host"].as<String>()    : "";
        cfg.port         = obj.containsKey("port")         ? obj["port"].as<uint16_t>()  : 1883;
        cfg.user         = obj.containsKey("user")         ? obj["user"].as<String>()    : "";
        cfg.pass         = obj.containsKey("pass")         ? obj["pass"].as<String>()    : "";
        cfg.topic_prefix = obj.containsKey("topic_prefix") ? obj["topic_prefix"].as<String>() : "";

        saveMqttConfig(cfg);
        mqttNeedsReconnect = true;

        DBG_PRINTF("[MQTT] Saved: enable=%d host='%s' port=%d user='%s' topic_prefix='%s'\n",
          cfg.enable, cfg.host.c_str(), cfg.port, cfg.user.c_str(), cfg.topic_prefix.c_str());

        // Frontend expects text here
        request->send(200, "text/plain", "OK");
      }
    );
    setMqtt->setMethod(HTTP_POST);
    setMqtt->setMaxContentLength(1024);
    server.addHandler(setMqtt);
  }

  // ---------- /mqtt_test (POST -> text) ----------
  server.on("/mqtt_test", HTTP_POST, [](AsyncWebServerRequest *request){
    DBG_PRINTLN("[MQTT] /mqtt_test requested.");
    loadMqttConfig();

    // Quick validation
    if (mqttCfg.host.isEmpty() || mqttCfg.port == 0) {
      request->send(200, "text/plain", "❌ Invalid host/port");
      return;
    }

    // Keep this short & cooperative
    WiFiClient wifiClient;
    #if defined(ARDUINO_ARCH_ESP32)
      wifiClient.setTimeout(2000);  // read timeout safeguard
    #endif

    PubSubClient testClient(wifiClient);
    testClient.setServer(mqttCfg.host.c_str(), mqttCfg.port);
    testClient.setSocketTimeout(2);   // seconds
    String clientId = "MiniExcoTest-" + String((uint32_t)millis());

    bool connected = false;
    // Try connect (keep it snappy)
    yield();
    if (mqttCfg.user.length() > 0) {
      DBG_PRINTF("[MQTT] Connecting (user='%s')\n", mqttCfg.user.c_str());
      connected = testClient.connect(clientId.c_str(),
                                    mqttCfg.user.c_str(),
                                    mqttCfg.pass.c_str());
    } else {
      DBG_PRINTLN("[MQTT] Connecting anonymously.");
      connected = testClient.connect(clientId.c_str());
    }
    yield();

    if (connected) testClient.disconnect();

    request->send(200, "text/plain",
                  connected ? "✅ MQTT connection OK" : "❌ MQTT connection failed");
  });

  // ---------- /mqtt_status (GET -> JSON) ----------
  server.on("/mqtt_status", HTTP_GET, [](AsyncWebServerRequest *request){
    auto *resp = new AsyncJsonResponse();
    JsonObject root = resp->getRoot().to<JsonObject>();
    root["connected"]  = mqttConnected;
    if (!mqttConnected && mqttLastError.length()) {
      root["last_error"] = mqttLastError;
    }
    resp->setCode(200);
    resp->setLength();                                // ✅
    request->send(resp);
  });

  // ---------- /mqtt_discovery (POST -> text) ----------
  server.on("/mqtt_discovery", HTTP_POST, [](AsyncWebServerRequest *request){
    if (mqtt.connected()) {
      publishMqttDiscovery();
      mqttDiscoveryPublished = true;
      request->send(200, "text/plain", "📢 Discovery published");
    } else {
      request->send(200, "text/plain", "❌ MQTT not connected");
    }
  });

  // ---------- /camera_enable (GET -> JSON) ----------
  server.on("/camera_enable", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *resp = request->beginResponseStream("application/json");
    StaticJsonDocument<192> doc;

    if (!request->hasParam("val")) {
      doc["ok"] = false;
      doc["error"] = "missing_val";
      serializeJson(doc, *resp);
      request->send(resp);
      return;
    }

    const bool wantEnable = (request->getParam("val")->value().toInt() == 1);

    if (wantEnable == cameraEnabled) {
      doc["ok"] = true;
      doc["status"] = "nochange";
      doc["enabled"] = cameraEnabled;
      serializeJson(doc, *resp);
      request->send(resp);
      return;
    }

    bool ok = wantEnable ? enableCamera() : disableCamera();

    // double-check sensor after enabling
    if (wantEnable && ok) ok = (esp_camera_sensor_get() != nullptr);

    if (ok) {
      cameraEnabled = wantEnable;
      doc["ok"] = true;
      doc["status"] = wantEnable ? "enabled" : "disabled";
      doc["enabled"] = cameraEnabled;
    } else {
      doc["ok"] = false;
      doc["status"] = wantEnable ? "enable_failed" : "disable_failed";
      doc["enabled"] = cameraEnabled;
    }

    serializeJson(doc, *resp);
    request->send(resp);
  });

  // ---------- /camera_status (GET) ----------
  server.on("/camera_status", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *resp = request->beginResponseStream("application/json");

    StaticJsonDocument<96> doc;
    doc["enabled"] = cameraEnabled;

    serializeJson(doc, *resp);
    request->send(resp);
  });


  //---------------------------------------------------------------------------------------------------HANDLERS END-----------------------------------------------------------

  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);
  server.begin();
  DBG_PRINTLN("HTTP server started");

  // OLED Display Status
  if (wifiConnected) {
      wifiState = WIFI_STA_CONNECTED;
    } else {
      //displayMessage(WiFi.softAPIP().toString(), "🌐 AP Setup Ready");
      wifiState = WIFI_AP_MODE;
  }
}

//-------------------------------------------Telemetry-----------------------------------------------------------------------

// ----------- Battery, Charger, WiFi, Temp, Uptime Telemetry -----------
void sendBatteryTelemetryIfIdle() {
    static unsigned long lastTelemetrySend = 0;
    static float filteredBatteryVoltage = 0;
    static float filteredChargerVoltage = 0;
    const float filterAlpha = 0.15; // 0.05–0.3, lower = smoother, slower

    if (!audio.isRunning() && millis() - lastTelemetrySend > 1000) {
        lastTelemetrySend = millis();

        int adcRaw = analogRead(BLevel);
        int adcRawCharging = analogRead(CSense);

        const float battR1 = 22000.0, battR2 = 10000.0;
        const float correctionFactorB = 8.4 / 10;
        float batteryVoltage = (adcRaw * 3.3 / 4095.0) * ((battR1 + battR2) / battR2) * correctionFactorB;

        const float chgR1 = 10000.0, chgR2 = 6800.0;
        float chargerVoltage = (adcRawCharging * 3.3 / 4095.0) * ((chgR1 + chgR2) / chgR2);

        // --- SOFTWARE FILTER: Exponential Moving Average (EMA) ---
        if (filteredBatteryVoltage == 0) filteredBatteryVoltage = batteryVoltage;
        filteredBatteryVoltage = filterAlpha * batteryVoltage + (1 - filterAlpha) * filteredBatteryVoltage;

        if (filteredChargerVoltage == 0) filteredChargerVoltage = chargerVoltage;
        filteredChargerVoltage = filterAlpha * chargerVoltage + (1 - filterAlpha) * filteredChargerVoltage;

        // --- Use filteredBatteryVoltage & filteredChargerVoltage below! ---
        const float MIN_BATTERY_VOLTAGE = 6.6;
        const float MAX_BATTERY_VOLTAGE = 8.3;
        int batteryPercent = constrain((int)(((filteredBatteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0), 0, 100);

        int rssi = WiFi.RSSI();
        int wifiQuality = constrain(2 * (rssi + 100), 0, 100);

        const char* chargingStatus = nullptr;

        // --- CHARGER & SOUND LOGIC ---
        if (filteredChargerVoltage > 4) {
            chargingStatus = "YES";
            isCharging = true;
            if (!wasCharging) {
                playSystemSound("/web/pcm/charging.wav");
                wasCharging = true;
                endChargingPlayed = false;
            }
            if (batteryPercent >= 100) {
                if (!chargingCompletePlayed) {
                    playSystemSound("/web/pcm/chargeComplete.wav");
                    chargingCompletePlayed = true;
                }
                wasFullyCharged = true;
            } else {
                chargingCompletePlayed = false;
                wasFullyCharged = false;
            }
        } else if (filteredChargerVoltage < 2.5) {
            chargingStatus = "NO";
            isCharging = false;
            if (wasCharging) {
                if (!endChargingPlayed) {
                    playSystemSound("/web/pcm/endCharging.wav");
                    endChargingPlayed = true;
                }
                wasCharging = false;
            }
            chargingCompletePlayed = false;
            wasFullyCharged = false;
        } else {
            chargingStatus = "FAULT";
            isCharging = false;
            playSystemSound("/web/pcm/error (5).wav");
            wasCharging = false;
            chargingCompletePlayed = false;
            endChargingPlayed = false;
            wasFullyCharged = false;
        }

        // --- LOW BATTERY ALERT LOGIC ---
        if (batteryPercent <= 5) {
            lowBatteryAlertActive = true;
        } else {
            lowBatteryAlertActive = false;
            lastLowBatteryBeep = 0;
        }

        if (lowBatteryAlertActive && !audio.isRunning()) {
            if (millis() - lastLowBatteryBeep > 30000) {
                playSystemSound("/web/pcm/beep.wav");
                lastLowBatteryBeep = millis();
            }
        }

        unsigned long uptimeSecs = millis() / 1000;
        float chipTemp = temperatureRead();

        // ---- Fill currentSample for MQTT! ----
        currentSample.batteryPercent = batteryPercent;
        currentSample.voltage        = filteredBatteryVoltage;
        currentSample.charger        = filteredChargerVoltage;
        currentSample.wifi           = wifiQuality;
        currentSample.temp           = chipTemp;

        wsCarInput.textAll("BATT," + String(batteryPercent) + "," + String(filteredBatteryVoltage, 2) + "," + String(wifiQuality));
        wsCarInput.textAll("CHARGE," + String(chargingStatus));
        wsCarInput.textAll("STATS," + String(uptimeSecs) + "," + String(chipTemp, 1));

        batteryPercentDisplay = batteryPercent;
        wifiSignalStrength = wifiQuality;
    }
}

// ----------- IMU Telemetry -----------
void sendImuTelemetry() {
  static unsigned long lastImuSend = 0;
  if (millis() - lastImuSend > 500) {
    lastImuSend = millis();
    imu::Vector<3> euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> mag   = bno055.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    float temp = bno055.getTemp();
    String imuMsg = "IMU," + String(euler.x(), 1) + "," + String(euler.y(), 1) + "," + String(euler.z(), 1)
                  + "," + String(mag.x(), 1) + "," + String(mag.y(), 1) + "," + String(mag.z(), 1)
                  + "," + String(temp, 1);
    wsCarInput.textAll(imuMsg);

    // ---- Fill currentSample for MQTT! ----
    currentSample.imu_euler_x = euler.x();
    currentSample.imu_euler_y = euler.y();
    currentSample.imu_euler_z = euler.z();
    currentSample.imu_mag_x   = mag.x();
    currentSample.imu_mag_y   = mag.y();
    currentSample.imu_mag_z   = mag.z();
    currentSample.imu_temp    = temp;
  }
}

// ----------- FPS Telemetry -----------
void sendFpsTelemetry() {
  static unsigned long lastFpsSend = 0;
  extern volatile int frameCount;  // define globally!
  static int lastFps = 0;
  if (millis() - lastFpsSend > 1000) {
    lastFpsSend = millis();
    lastFps = frameCount;
    wsCarInput.textAll("FPS," + String(lastFps));
    frameCount = 0;

    // ---- Fill currentSample for MQTT! ----
    currentSample.fps = lastFps;
  }
}

//-------------------------------------Serial Commands and debug---------------------------------

void handleSerialCommands() {
  if (Serial.available()) {
    String input = "";
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      input += c;
      delay(2);
    }
    input.trim();

    if (input.length() > 0) {
      // Command: Play a WAV file by path (e.g., "P myfile.wav")
      if (input.charAt(0) == 'P' && (input.length() > 1) && !isDigit(input.charAt(1))) {
        String filename = input.substring(1);
        filename.trim();
        DBG_PRINTF("[SERIAL] Requested play: '%s'\n", filename.c_str());
        if (!SD.exists(filename.c_str())) {
          DBG_PRINTF("[ERROR] File not found on SD: '%s'\n", filename.c_str());
        } else {
          playWavFileOnSpeaker(filename.c_str());
        }
      }
      // Command: Legacy motor control (F/B/L/R followed by number)
      else if (input.length() > 1 &&
               (input.charAt(0) == 'F' || input.charAt(0) == 'B' ||
                input.charAt(0) == 'L' || input.charAt(0) == 'R') &&
               isDigit(input.charAt(1))) {
        char cmd = input.charAt(0);
        int value = input.substring(1).toInt();
        if (cmd == 'F') controlMotorByDirection("Forward", value);
        else if (cmd == 'B') controlMotorByDirection("Backward", value);
        else if (cmd == 'L') controlMotorByDirection("Left", value);
        else if (cmd == 'R') controlMotorByDirection("Right", value);
        DBG_PRINTF("✅ Command: %c %d\n", cmd, value);
      }
      // Command: Plain string commands
      else {
        String cmd = input;
        cmd.toLowerCase();

        if (cmd == "next") nextTrack();
        else if (cmd == "heap" || cmd == "debug") {
          printDebugInfo();
        }
        else if (cmd == "serverreboot" || cmd == "rebootserver") {
            DBG_PRINTLN("[SERIAL] Triggering webServerReboot by command...");
            webServerReboot();
        }        
        else if (cmd == "wifi" || cmd == "debug") {
          printWifiDebugInfo();
        }        
        else if (cmd == "previous") prevTrack();
        else if (cmd == "play") playCurrentTrack();
        else if (cmd == "stop") {
          stopAudio();
          DBG_PRINTLN("[STOP]");
        }
        else if (cmd == "random") randomTrack();
        else if (cmd == "nextfolder") nextFolder();
        else if (cmd == "reset" || cmd == "reboot") resetESP();
        else if (cmd == "+") setVolume(currentVolume + 1);
        else if (cmd == "-") setVolume(currentVolume - 1);
        else if (cmd == "list") {
          DBG_PRINTLN("Current playlist:");
          for (size_t i = 0; i < playlist.size(); ++i) {
            DBG_PRINTF("[%d] %s\n", i, playlist[i].c_str());
          }
        } else {
          DBG_PRINTF("[UNKNOWN COMMAND] '%s'\n", input.c_str());
        }
      }
    }
  }
}

void printDebugInfo() {
  DBG_PRINTLN(F("==== ESP32 Debug Info ===="));
  DBG_PRINTF("Free heap:      %u bytes\n", ESP.getFreeHeap());
  DBG_PRINTF("Min free heap:  %u bytes\n", ESP.getMinFreeHeap());
  DBG_PRINTF("Heap size:      %u bytes\n", ESP.getHeapSize());
  DBG_PRINTF("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap());
  DBG_PRINTF("Uptime:         %lu ms\n", millis());
  DBG_PRINTF("Sketch size:    %u bytes\n", ESP.getSketchSize());
  DBG_PRINTF("Flash chip size:%u bytes\n", ESP.getFlashChipSize());
  #ifdef ESP_IDF_VERSION_MAJOR
    DBG_PRINTF("ESP-IDF v%d.%d.%d\n", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  #endif
    DBG_PRINTLN("==========================");
}

void printWifiDebugInfo() {
  DBG_PRINTLN("------ Debug Info ------");

  // General heap/PSRAM status
  DBG_PRINTF("Free Heap:   %u bytes\n", ESP.getFreeHeap());
  #if CONFIG_IDF_TARGET_ESP32S3
    DBG_PRINTF("Free PSRAM:  %u bytes\n", ESP.getFreePsram());
  #endif

  // Wi-Fi stack status
  wl_status_t status = WiFi.status();
  const char* wifiStatusStr =
    (status == WL_CONNECTED)      ? "CONNECTED" :
    (status == WL_NO_SSID_AVAIL)  ? "NO_SSID_AVAIL" :
    (status == WL_IDLE_STATUS)    ? "IDLE_STATUS" :
    (status == WL_SCAN_COMPLETED) ? "SCAN_COMPLETED" :
    (status == WL_CONNECT_FAILED) ? "CONNECT_FAILED" :
    (status == WL_CONNECTION_LOST)? "CONNECTION_LOST" :
    (status == WL_DISCONNECTED)   ? "DISCONNECTED" : "UNKNOWN";
  DBG_PRINTF("WiFi.status(): %d (%s)\n", status, wifiStatusStr);

  // Detailed Wi-Fi info
  DBG_PRINTF("IP Address:   %s\n", WiFi.localIP().toString().c_str());
  DBG_PRINTF("Gateway:      %s\n", WiFi.gatewayIP().toString().c_str());
  DBG_PRINTF("Subnet Mask:  %s\n", WiFi.subnetMask().toString().c_str());
  DBG_PRINTF("DNS 1:        %s\n", WiFi.dnsIP(0).toString().c_str());
  DBG_PRINTF("DNS 2:        %s\n", WiFi.dnsIP(1).toString().c_str());
  DBG_PRINTF("RSSI:         %d dBm\n", WiFi.RSSI());
  DBG_PRINTF("Hostname:     %s\n", WiFi.getHostname());

  // Print AP details if connected
  if (status == WL_CONNECTED) {
    DBG_PRINTF("SSID:         %s\n", WiFi.SSID().c_str());
    DBG_PRINTF("BSSID:        %s\n", WiFi.BSSIDstr().c_str());
    DBG_PRINTF("Channel:      %d\n", WiFi.channel());
    // No PHY mode on ESP32S3, skip
  }

  // WebSocket status
  DBG_PRINTF("WebSocket clients: %u\n", wsCarInput.count());
  for (auto& c : wsCarInput.getClients()) {
      DBG_PRINTF("  WS Client #%u, IP: %s, status: %s\n",
        c.id(), c.remoteIP().toString().c_str(), c.status() ? "Connected" : "Not Connected");
  }


  DBG_PRINTLN("------------------------");
}


//--------------------------------MQTT---------------------------------------------------------------

String getDeviceBlock() {
  String ipStr = WiFi.localIP().toString();
  return
    "\"device\":{"
      "\"identifiers\":[\"" S3_ID "\"],"
      "\"connections\":[[\"ip\",\"" + ipStr + "\"]],"
      "\"manufacturer\":\"https://github.com/elik745i/miniexco.v1\","
      "\"model\":\"MiniExco S3\","
      "\"name\":\"MiniExco Robot\","
      "\"sw_version\":\"" FIRMWARE_VERSION "\""
    "}";
}

void loadMqttConfig() {
  prefs.begin("mqtt", true);
  mqttCfg.enable = prefs.getBool("enable", false);
  mqttCfg.host = prefs.getString("host", "");
  mqttCfg.port = prefs.getInt("port", 1883);
  mqttCfg.user = prefs.getString("user", "");
  mqttCfg.pass = prefs.getString("pass", "");
  mqttCfg.topic_prefix = prefs.getString("topic", "");
  prefs.end();
}

void saveMqttConfig(const MQTTConfig& cfg) {
  prefs.begin("mqtt", false);
  prefs.putBool("enable", cfg.enable);
  prefs.putString("host", cfg.host);
  prefs.putInt("port", cfg.port);
  prefs.putString("user", cfg.user);
  prefs.putString("pass", cfg.pass);
  prefs.putString("topic", cfg.topic_prefix);
  prefs.end();
}

void publishMqttDiscovery() {
  String prefix = getMqttPrefix();

  // ---- Battery percent ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "battery/config").c_str(),
    ("{"
      "\"name\":\"Battery Level\","
      "\"state_topic\":\"" + prefix + "battery\","
      "\"unit_of_measurement\":\"%\","
      "\"device_class\":\"battery\","
      "\"unique_id\":\"" S3_ID "_battery\","
      + getDeviceBlock() +
    "}").c_str(), true);

  /*
  String topic = "homeassistant/sensor/" + prefix + "battery_voltage/config";
  String payload = "{"
      "\"name\":\"Battery Voltage\","
      "\"state_topic\":\"" + prefix + "battery_voltage\","
      "\"unit_of_measurement\":\"V\","
      "\"device_class\":\"voltage\","
      "\"unique_id\":\"" S3_ID "_battvolt\","
      + getDeviceBlock() +
  "}";

  Serial.print("[MQTT] Connected at publish? ");
  Serial.println(mqtt.connected() ? "YES" : "NO");

  Serial.print("[MQTT] Topic: ");
  Serial.println(topic);

  Serial.print("[MQTT] Payload length: ");
  Serial.println(payload.length());

  Serial.print("[MQTT] Payload: ");
  Serial.println(payload);

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);

  Serial.println(ok ? "Published battery voltage config OK" : "Failed to publish battery voltage config");

  // Extra: Try a simple test publish
  bool test = mqtt.publish("test/topic", "Hello world", true);
  Serial.println(test ? "Test topic published OK" : "Test topic publish FAIL");
  */

  
  // ---- Battery voltage ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "battery_voltage/config").c_str(),
    ("{"
      "\"name\":\"Battery Voltage\","
      "\"state_topic\":\"" + prefix + "battery_voltage\","
      "\"unit_of_measurement\":\"V\","
      "\"device_class\":\"voltage\","
      "\"unique_id\":\"" S3_ID "_battvolt\","
      + getDeviceBlock() +
    "}").c_str(), true);
  
  // ---- Charger voltage ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "charger_voltage/config").c_str(),
    ("{"
      "\"name\":\"Charger Voltage\","
      "\"state_topic\":\"" + prefix + "charger_voltage\","
      "\"unit_of_measurement\":\"V\","
      "\"device_class\":\"voltage\","
      "\"unique_id\":\"" S3_ID "_chvolt\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Charging status ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "charging/config").c_str(),
    ("{"
      "\"name\":\"Charging Status\","
      "\"state_topic\":\"" + prefix + "charging\","
      "\"unique_id\":\"" S3_ID "_charging\","
      "\"icon\":\"mdi:battery-charging\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- WiFi Signal ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "wifi/config").c_str(),
    ("{"
      "\"name\":\"WiFi Signal\","
      "\"state_topic\":\"" + prefix + "wifi\","
      "\"unit_of_measurement\":\"%\","
      "\"device_class\":\"signal_strength\","
      "\"unique_id\":\"" S3_ID "_wifi\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Chip Temperature ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "chip_temp/config").c_str(),
    ("{"
      "\"name\":\"Chip Temperature\","
      "\"state_topic\":\"" + prefix + "chip_temp\","
      "\"unit_of_measurement\":\"°C\","
      "\"device_class\":\"temperature\","
      "\"unique_id\":\"" S3_ID "_chiptemp\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- FPS ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "fps/config").c_str(),
    ("{"
      "\"name\":\"Camera FPS\","
      "\"state_topic\":\"" + prefix + "fps\","
      "\"unit_of_measurement\":\"fps\","
      "\"unique_id\":\"" S3_ID "_fps\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Uptime ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "uptime/config").c_str(),
    ("{"
      "\"name\":\"Uptime\","
      "\"state_topic\":\"" + prefix + "uptime\","
      "\"unit_of_measurement\":\"s\","
      "\"icon\":\"mdi:clock-outline\","
      "\"unique_id\":\"" S3_ID "_uptime\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- IMU: Euler ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_roll/config").c_str(),
    ("{"
      "\"name\":\"IMU Roll\","
      "\"state_topic\":\"" + prefix + "imu_roll\","
      "\"unit_of_measurement\":\"°\","
      "\"icon\":\"mdi:axis-x-rotate-clockwise\","
      "\"unique_id\":\"" S3_ID "_imu_roll\","
      + getDeviceBlock() +
    "}").c_str(), true);
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_pitch/config").c_str(),
    ("{"
      "\"name\":\"IMU Pitch\","
      "\"state_topic\":\"" + prefix + "imu_pitch\","
      "\"unit_of_measurement\":\"°\","
      "\"icon\":\"mdi:axis-y-rotate-clockwise\","
      "\"unique_id\":\"" S3_ID "_imu_pitch\","
      + getDeviceBlock() +
    "}").c_str(), true);
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_yaw/config").c_str(),
    ("{"
      "\"name\":\"IMU Yaw\","
      "\"state_topic\":\"" + prefix + "imu_yaw\","
      "\"unit_of_measurement\":\"°\","
      "\"icon\":\"mdi:axis-z-rotate-clockwise\","
      "\"unique_id\":\"" S3_ID "_imu_yaw\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- IMU: Magnetometer ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_mag_x/config").c_str(),
    ("{"
      "\"name\":\"IMU Mag X\","
      "\"state_topic\":\"" + prefix + "imu_mag_x\","
      "\"icon\":\"mdi:magnet\","
      "\"unique_id\":\"" S3_ID "_imu_mag_x\","
      + getDeviceBlock() +
    "}").c_str(), true);
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_mag_y/config").c_str(),
    ("{"
      "\"name\":\"IMU Mag Y\","
      "\"state_topic\":\"" + prefix + "imu_mag_y\","
      "\"icon\":\"mdi:magnet\","
      "\"unique_id\":\"" S3_ID "_imu_mag_y\","
      + getDeviceBlock() +
    "}").c_str(), true);
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_mag_z/config").c_str(),
    ("{"
      "\"name\":\"IMU Mag Z\","
      "\"state_topic\":\"" + prefix + "imu_mag_z\","
      "\"icon\":\"mdi:magnet\","
      "\"unique_id\":\"" S3_ID "_imu_mag_z\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- IMU Temp ----
  mqtt.publish(
    ("homeassistant/sensor/" + prefix + "imu_temp/config").c_str(),
    ("{"
      "\"name\":\"IMU Temperature\","
      "\"state_topic\":\"" + prefix + "imu_temp\","
      "\"unit_of_measurement\":\"°C\","
      "\"device_class\":\"temperature\","
      "\"unique_id\":\"" S3_ID "_imutemp\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Light Control (Switch) ----
  mqtt.publish(
    ("homeassistant/switch/" + prefix + "light/config").c_str(),
    ("{"
      "\"name\":\"Robot Light\","
      "\"command_topic\":\"" + prefix + "light/set\","
      "\"state_topic\":\"" + prefix + "light\","
      "\"unique_id\":\"" S3_ID "_light\","
      "\"icon\":\"mdi:lightbulb\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // --- Beacon Switch ---
  mqtt.publish(
    ("homeassistant/switch/" + prefix + "beacon/config").c_str(),
    ("{"
      "\"name\":\"Beacon\","
      "\"command_topic\":\"" + prefix + "beacon/set\","
      "\"state_topic\":\"" + prefix + "beacon\","
      "\"unique_id\":\"" S3_ID "_beacon\","
      "\"icon\":\"mdi:car-light-high\","
      + getDeviceBlock() +
    "}").c_str(), true
  );

  // --- Emergency Switch ---
  mqtt.publish(
    ("homeassistant/switch/" + prefix + "emergency/config").c_str(),
    ("{"
      "\"name\":\"Emergency\","
      "\"command_topic\":\"" + prefix + "emergency/set\","
      "\"state_topic\":\"" + prefix + "emergency\","
      "\"unique_id\":\"" S3_ID "_emergency\","
      "\"icon\":\"mdi:alarm-light\","
      + getDeviceBlock() +
    "}").c_str(), true
  );

  // ---- Arm Up/Down (Buttons) ----
  mqtt.publish(
    ("homeassistant/button/" + prefix + "arm_up/config").c_str(),
    ("{"
      "\"name\":\"Arm Up\","
      "\"command_topic\":\"" + prefix + "arm_up/set\","
      "\"unique_id\":\"" S3_ID "_armup\","
      "\"icon\":\"mdi:arrow-up-bold-box\","
      + getDeviceBlock() +
    "}").c_str(), true);
  mqtt.publish(
    ("homeassistant/button/" + prefix + "arm_down/config").c_str(),
    ("{"
      "\"name\":\"Arm Down\","
      "\"command_topic\":\"" + prefix + "arm_down/set\","
      "\"unique_id\":\"" S3_ID "_armdown\","
      "\"icon\":\"mdi:arrow-down-bold-box\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Car Movement (Buttons) ----
  const char* dirs[] = {"forward","backward","left","right","stop"};
  const char* icons[] = {
    "mdi:arrow-up-bold-box",
    "mdi:arrow-down-bold-box",
    "mdi:arrow-left-bold-box",
    "mdi:arrow-right-bold-box",
    "mdi:stop-circle-outline"
  };
  for (int i = 0; i < 5; i++) {
    String dirLabel = String(dirs[i]);
    dirLabel.setCharAt(0, toupper(dirLabel.charAt(0))); // Capitalize first letter
    mqtt.publish(
      ("homeassistant/button/" + prefix + dirs[i] + "/config").c_str(),
      ("{"
        "\"name\":\"Move " + dirLabel + "\","
        "\"command_topic\":\"" + prefix + dirs[i] + "/set\","
        "\"unique_id\":\"" S3_ID "_" + dirs[i] + "\","
        "\"icon\":\"" + icons[i] + "\","
        + getDeviceBlock() +
      "}").c_str(), true);
  }

  // ---- Bucket Tilt (number/slider, optional) ----
  mqtt.publish(
    ("homeassistant/number/" + prefix + "bucket_tilt/config").c_str(),
    ("{"
      "\"name\":\"Bucket Tilt\","
      "\"command_topic\":\"" + prefix + "bucket_tilt/set\","
      "\"state_topic\":\"" + prefix + "bucket_tilt\","
      "\"min\":0,"
      "\"max\":180,"
      "\"step\":1,"
      "\"unit_of_measurement\":\"°\","
      "\"unique_id\":\"" S3_ID "_buckettilt\","
      "\"icon\":\"mdi:bucket-outline\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- AUX Tilt (number/slider, optional) ----
  mqtt.publish(
    ("homeassistant/number/" + prefix + "aux_tilt/config").c_str(),
    ("{"
      "\"name\":\"AUX Tilt\","
      "\"command_topic\":\"" + prefix + "aux_tilt/set\","
      "\"state_topic\":\"" + prefix + "aux_tilt\","
      "\"min\":0,"
      "\"max\":180,"
      "\"step\":1,"
      "\"unit_of_measurement\":\"°\","
      "\"unique_id\":\"" S3_ID "_auxtilt\","
      "\"icon\":\"mdi:tools\","
      + getDeviceBlock() +
    "}").c_str(), true);

  // ---- Add more as needed...
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0'; // Ensure null-terminated string
  String msg = (char*)payload;
  String tpc = String(topic);

  String prefix = getMqttPrefix();

  // ---- Light Switch ----
  if (tpc == prefix + "light/set") {
    msg.toUpperCase();
    if (msg == "ON" || msg == "1" || msg == "true" || msg == "TRUE") {
      light = true;
      lightControl(); // Turn light ON
      mqtt.publish((prefix + "light").c_str(), "ON", true);
    } else {
      light = false;
      lightControl(); // Turn light OFF
      mqtt.publish((prefix + "light").c_str(), "OFF", true);
    }
  }

  // ---- Beacon Switch ----
  else if (tpc == prefix + "beacon/set") {
    msg.toUpperCase();
    if (msg == "ON" || msg == "1" || msg == "true" || msg == "TRUE") {
      beaconOn = true;
      updatePixels();
      mqtt.publish((prefix + "beacon").c_str(), "ON", true);
    } else {
      beaconOn = false;
      updatePixels();
      mqtt.publish((prefix + "beacon").c_str(), "OFF", true);
    }
  }

  // ---- Emergency Switch ----
  else if (tpc == prefix + "emergency/set") {
    msg.toUpperCase();
    if (msg == "ON" || msg == "1" || msg == "true" || msg == "TRUE") {
      emergencyOn = true;
      updatePixels();
      mqtt.publish((prefix + "emergency").c_str(), "ON", true);
    } else {
      emergencyOn = false;
      updatePixels();
      mqtt.publish((prefix + "emergency").c_str(), "OFF", true);
    }
  }

  // ---- Arm Up ----
  else if (tpc == prefix + "arm_up/set") {
    armMotorUp();
    mqtt.publish((prefix + "arm_state").c_str(), "UP", true);
  }

  // ---- Arm Down ----
  else if (tpc == prefix + "arm_down/set") {
    armMotorDown();
    mqtt.publish((prefix + "arm_state").c_str(), "DOWN", true);
  }

  // ---- Movement ----
  else if (tpc == prefix + "forward/set") {
    moveCar(UP);
    mqtt.publish((prefix + "move_state").c_str(), "FORWARD", true);
  }
  else if (tpc == prefix + "backward/set") {
    moveCar(DOWN);
    mqtt.publish((prefix + "move_state").c_str(), "BACKWARD", true);
  }
  else if (tpc == prefix + "left/set") {
    moveCar(LEFT);
    mqtt.publish((prefix + "move_state").c_str(), "LEFT", true);
  }
  else if (tpc == prefix + "right/set") {
    moveCar(RIGHT);
    mqtt.publish((prefix + "move_state").c_str(), "RIGHT", true);
  }
  else if (tpc == prefix + "stop/set") {
    moveCar(STOP);
    mqtt.publish((prefix + "move_state").c_str(), "STOP", true);
  }

  // ---- Bucket Tilt ----
  else if (tpc == prefix + "bucket_tilt/set") {
    int angle = msg.toInt();
    angle = constrain(angle, 0, 180);
    bucketTilt(angle);
    mqtt.publish((prefix + "bucket_tilt").c_str(), String(angle).c_str(), true);
  }

  // ---- AUX Tilt ----
  else if (tpc == prefix + "aux_tilt/set") {
    int angle = msg.toInt();
    angle = constrain(angle, 0, 180);
    auxControl(angle);
    mqtt.publish((prefix + "aux_tilt").c_str(), String(angle).c_str(), true);
  }

  // ---- (Optional: add more control topics as needed) ----
}

bool mqttConnectAndSubscribe() {
  if (!mqttCfg.enable) return false; // Not enabled by user

  if (mqtt.connected()) return true; // Already connected

  // Connect with credentials if present
  bool result = false;
  if (mqttCfg.user.length()) {
    result = mqtt.connect(S3_ID, mqttCfg.user.c_str(), mqttCfg.pass.c_str());
  } else {
    result = mqtt.connect(S3_ID);
  }

  if (result) {
    DBG_PRINTLN("✅ MQTT connected.");
    // Subscribe to all relevant command/control topics:
    String prefix = getMqttPrefix();
    mqtt.subscribe((prefix + "light/set").c_str());
    mqtt.subscribe((prefix + "arm_up/set").c_str());
    mqtt.subscribe((prefix + "arm_down/set").c_str());
    mqtt.subscribe((prefix + "forward/set").c_str());
    mqtt.subscribe((prefix + "backward/set").c_str());
    mqtt.subscribe((prefix + "left/set").c_str());
    mqtt.subscribe((prefix + "right/set").c_str());
    mqtt.subscribe((prefix + "stop/set").c_str());
    mqtt.subscribe((prefix + "bucket_tilt/set").c_str());
    mqtt.subscribe((prefix + "aux_tilt/set").c_str());
    // Add more if you expose more controls

    // Send Home Assistant discovery packets
    publishMqttDiscovery();

    return true;
  } else {
    DBG_PRINT("❌ MQTT connect failed: ");
    DBG_PRINTLN(mqtt.state());
    return false;
  }
}

void mqttBegin() {
  loadMqttConfig();

  if (!mqttCfg.enable) {
    mqttConnected = false;
    mqttLastError = "MQTT disabled in config";
    mqtt.disconnect();
    return;
  }

  mqtt.setServer(mqttCfg.host.c_str(), mqttCfg.port);
  mqtt.setCallback(mqttCallback);

  if (WiFi.status() != WL_CONNECTED) {
    mqttConnected = false;
    mqttLastError = "WiFi not connected";
    return;
  }

  bool result = false;
  if (mqttCfg.user.length()) {
    result = mqtt.connect(S3_ID, mqttCfg.user.c_str(), mqttCfg.pass.c_str());
  } else {
    result = mqtt.connect(S3_ID);
  }

  if (result) {
    DBG_PRINTLN("✅ MQTT connected.");
    mqttConnected = true;
    mqttLastError = "";

    String prefix = getMqttPrefix();
    mqtt.subscribe((prefix + "light/set").c_str());
    mqtt.subscribe((prefix + "arm_up/set").c_str());
    mqtt.subscribe((prefix + "arm_down/set").c_str());
    mqtt.subscribe((prefix + "forward/set").c_str());
    mqtt.subscribe((prefix + "backward/set").c_str());
    mqtt.subscribe((prefix + "left/set").c_str());
    mqtt.subscribe((prefix + "right/set").c_str());
    mqtt.subscribe((prefix + "stop/set").c_str());
    mqtt.subscribe((prefix + "bucket_tilt/set").c_str());
    mqtt.subscribe((prefix + "aux_tilt/set").c_str());

    publishMqttDiscovery();
  } else {
    mqttConnected = false;
    mqttLastError = "Connect failed, state=" + String(mqtt.state());
    DBG_PRINT("❌ MQTT connect failed: ");
    DBG_PRINTLN(mqtt.state());
  }
}

void mqttDisconnect() {
  mqtt.disconnect();
  mqttConnected = false;
  mqttLastError = "Manually disconnected";
}

void publishPeriodicTelemetry() {
  unsigned long now = millis();
  if (now - lastMqttTelemetry >= mqttTelemetryInterval) {  
      String prefix = getMqttPrefix();

      mqtt.publish((prefix + "battery").c_str(), String(currentSample.batteryPercent).c_str(), true);
      mqtt.publish((prefix + "battery_voltage").c_str(), String(currentSample.voltage, 2).c_str(), true);
      mqtt.publish((prefix + "charger_voltage").c_str(), String(currentSample.charger, 2).c_str(), true);
      mqtt.publish((prefix + "charging").c_str(), currentSample.charger > 4 ? "YES" : "NO", true);
      mqtt.publish((prefix + "wifi").c_str(), String(currentSample.wifi).c_str(), true);
      mqtt.publish((prefix + "chip_temp").c_str(), String(currentSample.temp, 1).c_str(), true);
      mqtt.publish((prefix + "fps").c_str(), String(currentSample.fps).c_str(), true);
      mqtt.publish((prefix + "imu_roll").c_str(), String(currentSample.imu_euler_x, 1).c_str(), true);
      mqtt.publish((prefix + "imu_pitch").c_str(), String(currentSample.imu_euler_y, 1).c_str(), true);
      mqtt.publish((prefix + "imu_yaw").c_str(), String(currentSample.imu_euler_z, 1).c_str(), true);
      mqtt.publish((prefix + "imu_mag_x").c_str(), String(currentSample.imu_mag_x, 1).c_str(), true);
      mqtt.publish((prefix + "imu_mag_y").c_str(), String(currentSample.imu_mag_y, 1).c_str(), true);
      mqtt.publish((prefix + "imu_mag_z").c_str(), String(currentSample.imu_mag_z, 1).c_str(), true);
      mqtt.publish((prefix + "imu_temp").c_str(), String(currentSample.imu_temp, 1).c_str(), true);
      lastMqttTelemetry = now;
  }
}


//-----------------------------------------------------------------------------SETUP---------------------------------------------------------------------------------------------------------//
void setup() {
  initSdMutex();

  #ifdef DEBUG_SERIAL
    Serial.begin(115200);
    Serial.setDebugOutput(true);
  #endif

  setUpPinModes();

  wifiPrefs.begin("wifi", false);      
  camPrefs.begin("camsettings", false);
  keymapPrefs.begin("keymap", false); 
  joymapPrefs.begin("joymap", false);      
  imuPrefs.begin("telemetry", false);
  uiPrefs.begin("ui", false);
  oledPrefs.begin("oled", false);
  camEnablePrefs.begin("enabled", false);
  

  darkMode        = uiPrefs.getBool("darkMode", false);
  horScreen       = uiPrefs.getBool("Switch", false);
  holdBucket      = uiPrefs.getBool("HoldBucket", false);
  holdAux         = uiPrefs.getBool("HoldAux", false);
  tlmEnabled      = uiPrefs.getBool("RecordTelemetry", false);
  sSndEnabled     = uiPrefs.getBool("SystemSounds", true);
  sSndVolume      = uiPrefs.getInt("SystemVolume", 15);


  initInputMapsIfEmpty();

  i2cStart();

  spiStart();

  //cleanOrphanTempUploads();  // Clean orphan temp files NOW (no outer SdLock!)

  playSystemSound("/web/pcm/boot.wav");

  pixelStart();

  serverStart();
  DBG_PRINTF("After serverStart: heap=%u\n", ESP.getFreeHeap());

  startNtpSync();

  otaUpdate();

  // startCamera();

  // startCameraServer();

  // applySavedCamSettings();

  loadCameraPrefs();
  if (cameraEnabled) enableCamera();


  initBNO055();

  // Load playlist from your media folder
  if (loadPlaylistFromIndex(mediaFolders[0])) {
    DBG_PRINTF("[PLAYLIST] Loaded playlist from %s\n", mediaFolders[0]);
  } else {
    DBG_PRINTF("[PLAYLIST] Failed to load playlist from %s\n", mediaFolders[0]);
  }

  #if USE_BLUEPAD32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableNewBluetoothConnections(true);
    DBG_PRINTF("After Bluepad32: heap=%u\n", ESP.getFreeHeap());
  #endif

  // Begin MQTT
  mqtt.setBufferSize(512);
  mqttBegin();

  DBG_PRINTF("<<< End of setup: heap=%u, PSRAM=%u\n", ESP.getFreeHeap(), ESP.getFreePsram());

}



//------------------------------------------------------------------------------LOOP---------------------------------------------------------------------------------------------------------//

void loop() {

  #ifdef DEBUG_SERIAL
    //Serial commands if pins assigned to serial is free, check pin assignment globals
    handleSerialCommands();
  #endif

  //telemetry data
  sendBatteryTelemetryIfIdle();
  sendImuTelemetry();
  sendFpsTelemetry();
  if (tlmEnabled) flushTelemetryBufferToSD_Auto(); //Logging

  // Car controls
  wsCarInput.cleanupClients();

  // Animations!
  handleAnimationTimers();

  // Wi-Fi state machine
  handleWifiStateMachine();

  // ... remaining logic ...
  audio.loop();
 
  processReindexTask();
  pollTimeValid();
  streamMicToWebSocket();
  sendDeviceMediaProgress();

  #if USE_BLUEPAD32

    BP32.update();

    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            handleControllerInput(ctl);
        }
    }

  #endif

    static unsigned long lastGearFrame = 0;

    if (WiFi.status() == WL_CONNECTED) {
      if (millis() - lastGearFrame > 200) {
        animateGears();
        lastGearFrame = millis();
      }
    }


    // Handle MQTT reconnect requested by config change
    if (mqttNeedsReconnect) {
        mqttDisconnect();
        mqttBegin();
        mqttNeedsReconnect = false;
    }

    // --- MQTT handler (auto-reconnect, discovery, telemetry) ---
    if (mqttCfg.enable) {
        // 1. Auto-reconnect if needed
        if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
            static unsigned long lastAttempt = 0;
            if (millis() - lastAttempt > 5000) {
                lastAttempt = millis();
                mqttBegin();
                mqttDiscoveryPublished = false; // reset discovery flag after reconnect attempt
            }
        }

        // 2. After connect, publish discovery if needed
        if (mqtt.connected() && !mqttDiscoveryPublished) {
            publishMqttDiscovery();
            mqttDiscoveryPublished = true;
            DBG_PRINTLN("[MQTT] Discovery published!");
        }

        // 3. While connected, push telemetry and loop
        if (mqtt.connected()) {
            publishPeriodicTelemetry();
            mqtt.loop();
        }
    }

    // System sound playback poll
    if (isSystemSoundPlaying && !audio.isRunning()) {
        onSystemSoundFinished();
    }

  delay(1);
}



