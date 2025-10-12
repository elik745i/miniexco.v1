# 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
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
                   TO DO:
                    b. implement BMP180 barometric pressure sensor readings in UI
                    c. Start working on android APK and API for it.
         v2.0.78: Added 3D Models into the folder
                  Reworked WiFi connection logics           
         v2.0.77: Finetuning IP speakup and AP/STA modes
                  Troubleshooting WiFi down and watchdog reboot
         v2.0.76: Fixed NPT Time blocking the loop
                  Added IP speaking Capability
                  Added animation for AP mode
                  Fine tuning the code.
         v2.0.75: Implemented wifi connection priorities in saved multible networks.

         v2.0.74: Implement adaptive stream quality settings toggled through cam settings
                  Tweaked IMU chick check to be only in setup, otherwise playing audio hickups
                  Added Wifi AP deletion from prefs, otherwise keeps connecting to first one.
                  Tweaked frontend with show password eye
                  Tweaked front and backend with delete saved wifi AP
 
         v2.0.73: Made IMU chip optional not to block code from execution when its not present.
                  Added throttle to camera quality settings to help with heap allocation
                  Untie lights and media playing from websocket connection quality to avoid hickups
                  Pause streaming and playing media when loading settings modal, then on close resume

         v2.0.72: Cleaning Code from unused references
                  Implemented watchdog on crient disconnect Server Stuck/Dead
                  Videofeed for home assistant discovery added over MDNS
                  Settings → Devices & Services → Add Integration → search “MJPEG IP Camera”
                  camera:
                    - platform: mjpeg
                      name: MiniExco Rover
                      mjpeg_url: http://miniexco-s3-v1-01.local:81/stream
                      still_image_url: http://miniexco-s3-v1-01.local/capture
                  AP password change from UI

         v2.0.71: Serialize SD access
                  Fix telemetry kept logging
                  Added online radio channels to media panel
                  
         v2.0.70: Tweak Controls tab
                  Telemetry recording logics tweaks
                  Added Bluepad32 switch in frontpage

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
 *      - Fix large file uploads
 *      - OLED battery icon artifacts
 *      - 3D Model in overlay moving with tilt/turn and flash lights and animate
 *
 ***************************************************************/

/* Bluepad32 documentation: https://bluepad32.readthedocs.io/en/latest/
                            https://github.com/ricardoquesada/bluepad32
*/

// ============================================================================
// Feature switches (override with -D in build flags if you like)
// ----------------------------------------------------------------------------
// USE_BLUEPAD32 - Controlled by frontend Settings in Control tab switch - update v2.0.70
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

//-----------------------------------------------------------------------GLOBALS---------------------------------------------------------------------


//-------------------------------------------------------------------GLOBAL FLAGS------------------------------------------------------------------



//-----------------------------------------------------------------------LIBs----------------------------------------------------------------------
  // --- Core / Arduino ---
# 133 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 134 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 135 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 136 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 137 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // --- NVS / Preferences ---
# 140 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // --- Filesystems / Storage ---
# 143 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 144 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 145 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

# 147 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 148 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // Optional fast FS stats (used only if present)




  // --- Async web stack ---
# 156 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 157 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 158 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 159 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 160 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // --- Networking helpers ---
# 163 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 164 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 165 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 166 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // --- MQTT ---
# 169 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // --- Camera / Display / Sensors ---
# 172 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 173 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 174 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 175 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 176 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 177 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

  // auto-throttle quality/size if heap dips

  extern "C" void requestCameraParams(framesize_t fs, int quality);
  extern volatile int frameCount;

  // --- Audio / I2S / LEDC ---
# 185 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2



  // --- System / FreeRTOS / PSRAM ---
# 190 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 191 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 192 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2

# 194 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 195 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 196 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2


  //#if USE_BLUEPAD32
# 200 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
  //#endif

  // --- STL / utilities ---
# 204 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 205 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
# 206 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 2
  
# 206 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
 //#include <iostream>
  //#include <sstream>


//--------------------------------------------------------------------FIRMWARE INFO----------------------------------------------------------------






  // AP credentials (global)

  String g_apPassword = "12345678" /* 8..63 chars; change to your default*/; // mutable so you can change it from UI later

//---------------------------------------------------------------------Preferences-----------------------------------------------------------------

  Preferences preferences, prefs, wifiPrefs, camPrefs, keymapPrefs, joymapPrefs, imuPrefs, uiPrefs, oledPrefs, camEnablePrefs;

//---------------------------------------------------------------------NPT Tıme--------------------------------------------------------------------

  int myTimezone = +3; // For US Eastern (UTC-5)
  long gmtOffset_sec = myTimezone * 3600;
  bool timeIsValid = false;
  unsigned long lastTimeCheck = 0;


//----------------------------------------------------------------Globals for MQTT-----------------------------------------------------------------
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

  String getMqttPrefix() {
    loadMqttConfig(); // Ensure latest config
    String p = mqttCfg.topic_prefix;
    while (p.endsWith("/")) p.remove(p.length()-1);
    if (p.length()) p += "/";
    return p;
  }



//-----------------------------------------------------Globals for system sounds to play on event--------------------------------------------------
  unsigned long lastLowBatteryBeep = 0;
  bool lowBatteryAlertActive = false;
  static bool wasCharging = false; //for charging
  static bool chargingCompletePlayed = false; // For chargeComplete.wav
  static bool endChargingPlayed = false; // For endCharging.wav
  static bool wasFullyCharged = false; // For 100% detection
  bool sirenPlaying = false;

  volatile bool isSystemSoundPlaying = false;
  String lastPlayedFile = "";
  unsigned long lastPlayedTime = 0;

  // >>> KEEP ONLY ONE of these; delete any duplicate definition elsewhere <<<
  unsigned long soundRepeatDelay = 1000; // or 250 if you prefer; just one

  // >>> Queue size: 24 is fine; delete any later #ifndef/#define that redefines it <<<

  String systemSoundQueue[24];
  uint8_t queueHead = 0, queueTail = 0;

  // ---- Minimal exclusivity (NEW) ----
  volatile bool ss_exclusive = false; // while true, any new non-IP enqueues are dropped
  volatile bool ss_allow = false; // temporarily true only while we enqueue the IP sequence


// -------------------------------------------------------------- Globals for recording -----------------------------------------------------------
  volatile bool videoRecording = false;
  volatile bool videoTaskActive = false;
  TaskHandle_t videoTaskHandle = nullptr;

//---------------------------------------------------------Telemetry Logging Global Variables------------------------------------------------------

  bool imuPresent = false;
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


//------------------------------------------------------------------Bluepad32 Globals--------------------------------------------------------------

  //#if USE_BLUEPAD32






    ControllerPtr myControllers[
# 340 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                               4
# 340 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                                ];
  //#endif


//--------------------------------------------------------------------Audio Globals----------------------------------------------------------------

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
  bool isPaused = false;
  bool playbackStarted = false;


  // ---------------- System-sound scheduler ----------------
  static bool ss_apSpeechPending = false;
  static uint32_t ss_apSpeechAtMs = 0;
  static bool ss_staSpeechPending = false;
  static IPAddress ss_staIpToSpeak;

//--------------------------------------------------------------------Reindex globals--------------------------------------------------------------

  int reindexTotal = 0;
  bool reindexCounting = false; // true while counting files
  bool reindexReadyToIndex = false; // true when ready to start indexing

  volatile bool pendingReindex = false;
  String reindexPath = "/";
  int reindexCount = 0;

//----------------------------------------------------------------------File Upload----------------------------------------------------------------
  namespace sdweb { // keeps the symbol local to this file
    struct UploadCtx {
      String uploadPath; // final path
      String tmpPath; // temp path
      File uploadFile; // open temp handle
      String error; // non-empty => failed
      size_t bytesWritten = 0;
      size_t bytesSinceYield = 0; // for periodic flush/yield

      // Optional integrity check
      bool verifySha = false;
      mbedtls_sha256_context sha;
      String expectedShaHex; // 64-hex (lowercase)
      uint8_t digest[32]; // computed
    };

  }
  using sdweb::UploadCtx;

  volatile bool isSdUploadInProgress = false; //used in app_httpd.cpp
  volatile bool isStreaming = true; //used in app_httpd.cpp

//----------------------------------------------------------------------Serial Debug---------------------------------------------------------------
  /* As pins 19, 43 and 44 is used to control DRV8833 when debut is enabled, you will see serial but will be unable to control motors */
# 426 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
//-----------------------------------------------------------Reboot Watchdog on Webserver Disconnect-----------------------------------------------

  // --- WebServer (WebSocket) watchdog ---
  static volatile uint32_t lastWebActivityMs = 0; // last time we saw a WS connect/data/pong
  static volatile uint32_t lastWsDisconnectMs = 0; // last time a WS disconnected
  static volatile uint16_t wsActiveClients = 0; // current WS client count
  static bool hadAnyClientSinceBoot = false; // becomes true after first WS connect

  // Tunables
  static const uint32_t WS_REBOOT_TIMEOUT_MS = 5000; // 5 seconds with no WS clients
  static const uint32_t WS_REBOOT_GRACE_MS = 30000; // 30s after boot, don't reboot

  // If you already have these, keep them. Otherwise remove the 'extern' lines.
  extern volatile bool isSdUploadInProgress;
  extern volatile bool isStreaming; // set to true while /stream is serving frames


//-------------------------------------------------------------------------I2S SPK & MIC-------------------------------------------------------------
  volatile bool micStreamActive = false;

  Adafruit_SSD1306 display(128, 64, &Wire, -1);


  // --- I2S config for MSM261S mic ---







  // --- I2S config for NS4168 speaker ---
# 468 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
  // ===== MIC STREAM STATE =====
  static int16_t* micBuf = nullptr;
  static size_t micBufBytes = 1024 * sizeof(int16_t); // bytes per read
  static uint32_t micLastSendMs = 0;


  // Track current I2S state
  enum I2SMode { I2S_NONE, I2S_MIC, I2S_SPEAKER };
  I2SMode currentI2SMode = I2S_NONE;

//----------------------------------------------------------------------CONTROL PINS---------------------------------------------------------------
  // Channel numbers for ledcWrite/ledcSetup







  // PWM channel mapping summary
# 506 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
//-------------------------------------------------------------------------CAMERA------------------------------------------------------------------

  bool cameraEnabled = false;
  bool cameraInitialized = false;
# 533 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
  unsigned long lastCamCheck = 0;
  const unsigned long camCheckInterval = 3000; // 3 seconds

  //--------------Auto throttle stream quality-------------------------

  volatile bool allowFsAuto = false; // UI: auto_res
  volatile bool adaptiveQEnabled = true; // UI: adaptive_q

  static int bestQualityCap = 12; // best allowed (lowest number)
  static int minQualityFloor = 38; // worst allowed (highest number) safety floor
  static framesize_t startFS = FRAMESIZE_VGA; // your preferred FS from prefs
  static framesize_t minFS = FRAMESIZE_QQVGA; // absolute minimum FS you accept
  static framesize_t maxFS = FRAMESIZE_SVGA; // absolute maximum FS you want to use

  // ---- Targets / thresholds ----
  static const int TARGET_FPS = 15;
  static const int FPS_LO_HYST = TARGET_FPS - 2; // hysteresis window
  static const int FPS_HI_HYST = TARGET_FPS + 3;

  static const size_t HEAP_SOFT_FLOOR = 180 * 1024; // below this: start easing quality
  static const size_t HEAP_HARD_FLOOR = 120 * 1024; // below this: drop framesize or degrade more
  static const size_t HEAP_MIN_FREE_ABS = 80 * 1024; // absolute floor = emergency degrade

  static const int RSSI_WEAK = -70; // dBm
  static const int RSSI_BAD = -78; // dBm

  // ---- Step sizes / cadence ----
  static const int QUALITY_STEP = 2; // increase number = more compression
  static const int QUALITY_RECOVER_STEP= 1; // decrease number = better quality
  extern const uint32_t ADAPT_PERIOD_MS = 1000; // run controller every second

  // ---- State ----
  static int curQuality = 20; // will be seeded from sensor at start
  static framesize_t curFS = FRAMESIZE_VGA; // will be seeded from sensor at start
  static int lastFrameCount = 0;
  extern uint32_t lastTickMs = 0;

  // Helpers: next lower/higher framesize within your allowed band
  framesize_t stepDownFS(framesize_t fs) { // “bigger pixels” → fewer bytes
    if (fs <= minFS) return minFS;
    return (framesize_t)((int)fs - 1);
  }
  framesize_t stepUpFS(framesize_t fs) { // “smaller pixels” → more detail
    if (fs >= maxFS) return maxFS;
    return (framesize_t)((int)fs + 1);
  }

  // Bound a value into [lo, hi]
  int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

//-------------------------------------------------------------------SD_|CARD Pin defines----------------------------------------------------------






  // ---- SD mutex (recursive) ----
  static SemaphoreHandle_t sdMutex = nullptr;

  static inline void initSdMutex() {
    if (!sdMutex) sdMutex = xQueueCreateMutex( ( ( uint8_t ) 4U ) );
  }

  struct SdLock {
    SdLock() { xQueueTakeMutexRecursive( ( sdMutex ), ( ( TickType_t ) 0xffffffffUL ) ); }
    ~SdLock() { xQueueGiveMutexRecursive( ( sdMutex ) ); }
  };

  // --- Global SD stream gate (serialize SD sessions) ---
  static SemaphoreHandle_t g_sdStreamGate;
  static bool g_gateHeldByAudio = false; // true while audio (mp3/wav) owns SD exclusively
  static bool g_mediaPausedBySystem = false; // we paused mp3 to play a system beep

  struct SdGateGuard {
    bool held = false;
    SdGateGuard(bool take_now = true) {
      if (take_now) take();
    }
    void take() {
      if (!held) {
        xQueueSemaphoreTake( ( g_sdStreamGate ), ( ( TickType_t ) 0xffffffffUL ) );
        held = true;
      }
    }
    void give() {
      if (held) {
        xQueueGenericSend( ( QueueHandle_t ) ( g_sdStreamGate ), 
# 620 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
       __null
# 620 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
       , ( ( TickType_t ) 0U ), ( ( BaseType_t ) 0 ) );
        held = false;
      }
    }
    ~SdGateGuard(){ give(); }
  };

//---------------------------------------------------------------------------I2C-------------------------------------------------------------------



  Adafruit_BNO055 bno055 = Adafruit_BNO055(55, 0x28); // 0x28 is default addr

  volatile int frameCount = 0; // For FPS counting, used in loop()

  //-----------------------Neopixel Globals------------------------------



  Adafruit_NeoPixel pixels(12, 21, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /*|< Transmit as G,R,B*/ + 0x0000 /*|< 800 KHz data transmission*/);

  bool beaconOn = false;
  bool emergencyOn = false;
  bool leftSignalActive = false;
  bool rightSignalActive = false;

  uint8_t beaconPhase = 0;
  bool blinkState = false; // For emergency and turn signals
  unsigned long lastAnimUpdate = 0;
  const unsigned long beaconInterval = 180;
  const unsigned long blinkInterval = 400;

//-----------------------------------------------------------------------WiFi Globals--------------------------------------------------------------

enum WifiState { WIFI_AP_LOBBY, WIFI_STA_WAIT, WIFI_STA_OK };
extern WifiState wifiState;
WifiState wifiState = WIFI_AP_LOBBY;

String wifiSSID = "";
String wifiPassword = "";

unsigned long wifiLastScanAt = 0;
unsigned long wifiConnectSince = 0;

// scan / connect cadences
const uint32_t SCAN_PERIOD_MS = 15000; // how often to scan while in AP lobby
const uint32_t CONNECT_TIMEOUT = 8000; // how long to wait for WL_CONNECTED per attempt
int currentCandidateIndex = -1; // -1 means "start from preferred", then walk the list

// optional: keep AP password you already have
extern String g_apPassword;

// Non-blocking auto-detach tracking for servos
unsigned long auxDetachTime = 0;
bool auxAttached = false;

unsigned long bucketDetachTime = 0;
bool bucketAttached = false;

// --- Path drawing type (was lost during edits) ---
struct PathPoint { float x; float y; };

bool shouldReboot = false; // used in upload lambda
bool otaValid = false; // used in upload lambda

int wifiRetryCount = 5;
bool wifiConnecting = false;
unsigned long wifiConnectStartTime = 0;

//----------------------------------------------------------------------SERVO GLOBALS--------------------------------------------------------------




  int lastBucketValue = 140; // Set to your safe init value
  int lastAuxValue = 150;


// ----------------------------------------------------------------------OLED Globals--------------------------------------------------------------






  // For OLED or UI feedback
  unsigned long wifiOledLastMsg = 0;
  String wifiOledLastStep = "";

  int batteryPercentDisplay = 0;

  bool isCharging = false;

  int wifiSignalStrength = 0;

  String lastWsClientIP = "";
  unsigned long lastWsConnectTime = 0;

//----------------------------------------------------------------------SENSOR Globals-------------------------------------------------------------



//----------------------------------------------------------------------CONTROL Globals------------------------------------------------------------
# 740 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
  //Use a 10k + 4.7k divider — safe and very common. Let me know if yours is different.
  unsigned long lastTelemetrySend = 0;

  const float MAX_BATTERY_VOLTAGE = 8.4; // 2S Li-ion full charge
  const float MIN_BATTERY_VOLTAGE = 6.0; // 2S safe cutoff

  // ---- Actions we support everywhere ----
  enum Action : uint8_t {
    ACT_NONE = 0,
    ACT_FORWARD, ACT_BACKWARD, ACT_LEFT, ACT_RIGHT, ACT_STOP,
    ACT_ARM_UP, ACT_ARM_DOWN, ACT_BUCKET_UP, ACT_BUCKET_DOWN,
    ACT_AUX_UP, ACT_AUX_DOWN, ACT_LIGHT_TOGGLE, ACT_BEACON_TOGGLE,
    ACT_EMERGENCY_TOGGLE, ACT_HORN
  };


  static Action actionFromName(const String& name);
  static Action actionForKeyToken(String keyToken);
  static Action actionForJoyButtonCached(const String& btnName);
  static void dispatchAction(Action a, bool pressed);





  // ---- Keyboard defaults (what the UI shows by default too) ----
  static const struct { const char* action; const char* def; } KEYMAP_DEFAULTS[] = {
    {"forward", "w"},
    {"backward", "s"},
    {"left", "a"},
    {"right", "d"},
    {"stop", " "}, // space
    {"arm_up", " "},
    {"arm_down", " "},
    {"bucket_up", "e"},
    {"bucket_down", "q"},
    {"aux_up", "r"},
    {"aux_down", "f"},
    {"light_toggle", "l"},
    {"beacon", "b"},
    {"emergency","x"},
    {"horn", "h"},
  };
  static const size_t KEYMAP_N = sizeof(KEYMAP_DEFAULTS)/sizeof(KEYMAP_DEFAULTS[0]);

  // ---- Joypad defaults (Bluepad32-style names) ----
  // Keep simple & digital to start; you can extend with axes later.
  static const struct { const char* action; const char* defBtn; } JOYMAP_DEFAULTS[] = {
    {"forward", "DPAD_UP"},
    {"backward", "DPAD_DOWN"},
    {"left", "DPAD_LEFT"},
    {"right", "DPAD_RIGHT"},
    {"stop", "BTN_BACK"}, // “select/back” as stop
    {"arm_up", "L_STICK_UP"}, // ⟵ changed
    {"arm_down", "L_STICK_DOWN"}, // ⟵ changed 
    {"bucket_up", "R1"},
    {"bucket_down", "L1"},
    {"aux_up", "R2_CLICK"}, // treat R2/L2 as “click” if your lib offers it
    {"aux_down", "L2_CLICK"},
    {"light_toggle", "X"},
    {"beacon", "Y"},
    {"emergency","B"},
    {"horn", "A"},
  };
  static const size_t JOYMAP_N = sizeof(JOYMAP_DEFAULTS)/sizeof(JOYMAP_DEFAULTS[0]);

  // Normalize UI tokens to stored form
  static String normKeyToken(String s) {
    // handle literal single space BEFORE trimming
    if (s.length() == 1 && s[0] == ' ') return String(" ");

    s.trim();
    if (s.equalsIgnoreCase("Space")) return String(" ");
    if (s.startsWith("Arrow")) { s.toLowerCase(); return s; } // ArrowUp -> arrowup
    s.toLowerCase(); // letters/digits -> lowercase
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
    {"emergency","emergency"}, // UI "emergency"
    {"emergency","emergency_toggle"}, // legacy alias accepted too
    {"horn","horn"},
  };
  static const size_t ACTION_ALIASES_N = sizeof(ACTION_ALIASES)/sizeof(ACTION_ALIASES[0]);

  static String aliasToFw(const String& key) {
    for (size_t i=0;i<ACTION_ALIASES_N;++i) {
      if (key.equalsIgnoreCase(ACTION_ALIASES[i].alias)) return String(ACTION_ALIASES[i].fw);
      if (key.equalsIgnoreCase(ACTION_ALIASES[i].fw)) return String(ACTION_ALIASES[i].fw);
    }
    return key;
  }

//-------------------------------------------------------------------UI Settings variables---------------------------------------------------------

  bool darkMode = false;
  bool horScreen = false;
  bool holdBucket = false;
  bool holdAux = false;
  bool tlmEnabled = false;
  bool sSndEnabled = false;
  bool bluepadEnabled = false;
  int sSndVolume = 15;


  // global constants
  //const char* ap_ssid = "MiniExco_Setup";

  bool light = false;


  AsyncWebServer server(80);
  AsyncWebSocket wsCarInput("/CarInput");


  // --- PSRAM allocator for ESP32 Arduino ---
  // This must be defined ONCE, above all vector declarations that use it.

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


//-----------------------------------------------------------------------PSRAM GLOBALS-------------------------------------------------------------
  // -----------------------------------------------------
  // PSRAM-based or regular vector declarations for globals
  // Place this block after your struct/class definitions (TelemetrySample, PathPoint)
  // and after psram_allocator is defined
  // -----------------------------------------------------


  // Use psram_allocator for all large or expandable vectors to store them in external PSRAM.
  // This saves valuable internal RAM for stack/heap.
  // You can add any additional vectors here to move them to PSRAM.

  std::vector<String, psram_allocator<String>> playlist; // Stores filenames or track names for media
  std::vector<int, psram_allocator<int>> folderIndex; // Index mapping for folders
  std::vector<PathPoint, psram_allocator<PathPoint>> pathPoints; // List of path points received from frontend UI
  std::vector<TelemetrySample, psram_allocator<TelemetrySample>> telemetryBuffer; // Buffer for telemetry logging
# 905 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
  int currentTrack = 0;
  int currentVolume = 15; // Default volume (0–21)

  String currentlyPlayingFile = "";
  unsigned long lastMediaProgressSend = 0;





//-----------------------------------------------------------------MDNS HELPERS TO SANITIZE ID-----------------------------------------------------

  // Global sanitized hostname to use everywhere (STA/AP/mDNS)
  String g_mdnsHost;

  // Make a valid mDNS/DHCP hostname from S3_ID (lowercase, [a-z0-9-], <=63)
  static String makeMdnsHostFromId(const char* id) {
    String s = id;
    s.toLowerCase();
    for (size_t i = 0; i < s.length(); ++i) {
      char c = s[i];
      bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (c == '-');
      if (!ok) s.setCharAt(i, '-');
    }
    while (s.length() && s[0] == '-') s.remove(0, 1);
    while (s.length() && s[s.length()-1] == '-') s.remove(s.length()-1, 1);
    while (s.indexOf("--") != -1) s.replace("--", "-");
    if (s.isEmpty()) s = "esp32";
    if (s.length() > 63) s = s.substring(0, 63);
    return s;
  }

  // Call this early (before/around WiFi.begin) to set DHCP hostname
  static void initHostnames() {
    g_mdnsHost = makeMdnsHostFromId("MINIEXCO_S3_V1_02"); // e.g. "miniexco-s3-v1-01"
    WiFi.setHostname(g_mdnsHost.c_str());
  }

  // Start mDNS after you are connected (STA) or after AP is up (AP)
  static bool startMdns() {
    if (MDNS.begin(g_mdnsHost.c_str())) {
      MDNS.setInstanceName("MINIEXCO_S3_V1_02"); // pretty name
      MDNS.addService("http", "tcp", 80); // UI
      MDNS.addServiceTxt("http", "tcp", "path", "/");
      MDNS.addService("http", "tcp", 81); // stream
      MDNS.addServiceTxt("http", "tcp", "path", "/stream");
      return true;
    }
    return false;
  }
//---------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------FUNCTIONS-----------------------------------------------------------------


//----------------------------------------------------------------------Robot Cam Settings----------------------------------------------------------

  void startCameraServer();

  bool startCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = 7;
    config.pin_d1 = 5;
    config.pin_d2 = 4;
    config.pin_d3 = 6;
    config.pin_d4 = 8;
    config.pin_d5 = 42;
    config.pin_d6 = 48;
    config.pin_d7 = 47;
    config.pin_xclk = 33 /* ✅ From MCLK*/;
    config.pin_pclk = 41;
    config.pin_vsync = 35;
    config.pin_href = 34;
    config.pin_sccb_sda = 37 /* ✅ From schematic SDA*/;
    config.pin_sccb_scl = 36 /* ✅ From schematic SCK*/;
    config.pin_pwdn = -1;
    config.pin_reset = -1;

    config.xclk_freq_hz = 10000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    if (config.pixel_format == PIXFORMAT_JPEG) {
      if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 1;
        //config.grab_mode    = CAMERA_GRAB_LATEST;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

      } else {
        config.frame_size = FRAMESIZE_SVGA;
        //config.fb_location  = CAMERA_FB_IN_DRAM;
        config.fb_location = CAMERA_FB_IN_PSRAM;
      }
    } else {
      config.frame_size = FRAMESIZE_240X240;

      config.fb_count = 1;

    }

    esp_err_t err = esp_camera_init(&config);

    do { if (1) Serial.printf("[HEAP][After camera init] Free: %u, MinFree: %u, MaxAlloc: %u\n", ESP.getFreeHeap(), heap_caps_get_minimum_free_size((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/), heap_caps_get_largest_free_block((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/)); } while(0)


                                                                ;
    do { if (1) Serial.printf("[PSRAM][After camera init] Free: %u, MinFree: %u, MaxAlloc: %u\n", heap_caps_get_free_size((1<<10) /*|< Memory must be in SPI RAM*/), heap_caps_get_minimum_free_size((1<<10) /*|< Memory must be in SPI RAM*/), heap_caps_get_largest_free_block((1<<10) /*|< Memory must be in SPI RAM*/)); } while(0)


                                                                  ;

    if (err != 0 /*!< esp_err_t value indicating success (no error) */) {
      do { if (1) Serial.printf("Camera init failed with error 0x%x\n", err); } while(0);
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
      int res = camPrefs.getInt("res", FRAMESIZE_VGA);
      int quality = camPrefs.getInt("quality", 10);
      int contrast = camPrefs.getInt("contrast", 0);
      int brightness = camPrefs.getInt("brightness", 0);
      int saturation = camPrefs.getInt("saturation", 0);
      int gray = camPrefs.getInt("gray", 0);
      int hmirror = camPrefs.getInt("hmirror", 0);
      int vflip = camPrefs.getInt("vflip", 0);
      int special_effect = camPrefs.getInt("special_effect", 0);
      int wb_mode = camPrefs.getInt("wb_mode", 0);
      int awb = camPrefs.getInt("awb", 1);
      int agc = camPrefs.getInt("agc", 1);
      int agc_gain = camPrefs.getInt("agc_gain", 0);
      int aec = camPrefs.getInt("aec", 1);
      int aec_value = camPrefs.getInt("aec_value", 300);
      int aec2 = camPrefs.getInt("aec2", 0);
      int dcw = camPrefs.getInt("dcw", 1);
      int bpc = camPrefs.getInt("bpc", 0);
      int wpc = camPrefs.getInt("wpc", 1);
      int raw_gma = camPrefs.getInt("raw_gma", 1);
      int lenc = camPrefs.getInt("lenc", 1);
      int gainceiling= camPrefs.getInt("gainceiling", 0);
      int colorbar = camPrefs.getInt("colorbar", 0);

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
          int sharpness = camPrefs.getInt("sharpness", 2);
          int denoise = camPrefs.getInt("denoise", 0);
          int compression= camPrefs.getInt("compression", 12);

          if (s->set_sharpness) s->set_sharpness(s, sharpness);
          if (s->set_denoise) s->set_denoise(s, denoise);
          if (s->set_quality) s->set_quality(s, compression); // Can be optional
      }

      // === OV5640-specific ===
      if (s->id.PID == OV5640_PID) {
          int sharpness = camPrefs.getInt("sharpness", 33); // 0..255
          int denoise = camPrefs.getInt("denoise", 0); // 0,1
          int brightness = camPrefs.getInt("brightness", 0); // -4..4
          int saturation = camPrefs.getInt("saturation", 0); // -4..4
          int contrast = camPrefs.getInt("contrast", 0); // -4..4
          // int hue        = camPrefs.getInt("hue", 0);        // NOT USED

          if (s->set_sharpness) s->set_sharpness(s, sharpness);
          if (s->set_denoise) s->set_denoise(s, denoise);
          if (s->set_brightness) s->set_brightness(s, brightness);
          if (s->set_saturation) s->set_saturation(s, saturation);
          if (s->set_contrast) s->set_contrast(s, contrast);
          // No set_hue for OV5640 or any camera
      }


      // === Add other models as needed ===
      // For example: OV3660, etc.
      // if (s->id.PID == OV3660_PID) { ... }

      // -- Optionally, print sensor info for debug --
      do { if (1) Serial.printf("Camera PID: 0x%04X\n", s->id.PID); } while(0);
  }

  // --- Enable/disable functions ---
  bool enableCamera() {
    saveCameraPrefs(true);
    if (!cameraInitialized) {
      if (!startCamera()) {
        do { if (1) Serial.println("enableCamera(): startCamera failed"); } while(0);
        return false;
      }
      cameraInitialized = true;
      startCameraServer(); // assumes idempotent
      applySavedCamSettings();
      initAdaptiveCameraCaps();
      do { if (1) Serial.println("Camera enabled."); } while(0);
    }
    return true;
  }

  bool disableCamera() {
    if (cameraInitialized) {
      esp_camera_deinit();
      cameraInitialized = false;
      do { if (1) Serial.println("Camera deinitialized."); } while(0);
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

  //----------------------Auto throttle stream quality functions----------------------------


  // Initialize from prefs and actual sensor
  void initAdaptiveCameraCaps() {
    // Use the same prefs your UI writes
    bestQualityCap = clampInt(camPrefs.getInt("quality", 12), 10, 40);
    startFS = (framesize_t)clampInt(camPrefs.getInt("res", (int)FRAMESIZE_VGA),
                                          (int)minFS, (int)maxFS);
    // new optional UI flags
    allowFsAuto = camPrefs.getBool("auto_res", false);
    adaptiveQEnabled = camPrefs.getBool("adaptive_q", true);

    // Seed from current sensor so we don’t fight a fresh applySavedCamSettings()
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
      curQuality = s->status.quality;
      curFS = (framesize_t)s->status.framesize;
    }

    // Snap to your prefs and publish them via the stream-safe hook
    curQuality = clampInt(curQuality, bestQualityCap, minQualityFloor);
    curFS = startFS;
    requestCameraParams(curFS, curQuality); // stream loop applies between frames
  }

  // Core policy: compute new target (quality, framesize)
  void adaptCameraOnce() {
    // compute FPS
    int frames = frameCount;
    int fps = frames - lastFrameCount;
    lastFrameCount = frames;

    // sample health metrics
    size_t freeHeap = heap_caps_get_free_size((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/);
    size_t minEver = heap_caps_get_minimum_free_size((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/);
    int rssi = WiFi.isConnected() ? WiFi.RSSI() : -100;

    bool needRelief = (freeHeap < HEAP_SOFT_FLOOR) || (rssi <= RSSI_WEAK) || (fps < FPS_LO_HYST);
    bool severe = (freeHeap < HEAP_HARD_FLOOR) || (rssi <= RSSI_BAD)
                      || (minEver < HEAP_MIN_FREE_ABS) || (fps < TARGET_FPS - 4);
    bool canRecover = (freeHeap > HEAP_SOFT_FLOOR + 40*1024) && (rssi > RSSI_WEAK) && (fps > FPS_HI_HYST);

    int newQ = curQuality;
    framesize_t newFS = curFS;

    // If adaptive quality is OFF, don’t auto-tune quality (only obey manual changes)
    // If auto-res is OFF, never change framesize unless emergency RAM floor
    bool canTouchQuality = adaptiveQEnabled;
    bool canTouchFS = allowFsAuto;

    if (needRelief) {
      if (canTouchQuality) newQ += severe ? QUALITY_STEP * 2 : QUALITY_STEP;
      if (severe && (canTouchFS || freeHeap < HEAP_MIN_FREE_ABS)) {
        newFS = stepDownFS(newFS);
      }
    } else if (canRecover) {
      if (canTouchQuality) newQ -= QUALITY_RECOVER_STEP;
      if (canTouchFS && newFS < startFS &&
          fps > TARGET_FPS + 6 && // FPS well above target (≈15)
          rssi > RSSI_WEAK + 5 && // Wi-Fi is strong, not marginal
          freeHeap > HEAP_SOFT_FLOOR + 64*1024) { // at least +64k heap above soft floor
        newFS = stepUpFS(newFS);
      }
    }

    // Never exceed user's best/“max” quality or your safety floor
    newQ = clampInt(newQ, bestQualityCap, minQualityFloor);

    // Never go beyond the user’s selected resolution upward
    if (newFS > startFS) newFS = startFS;

    if (newQ != curQuality || newFS != curFS) {
      curQuality = newQ;
      curFS = newFS;
      requestCameraParams(curFS, curQuality); // applied cleanly between frames
    }
  }

  void runAdaptiveCamera() {
    uint32_t now = millis();
    if (now - lastTickMs >= ADAPT_PERIOD_MS) {
      lastTickMs = now;
      adaptCameraOnce();
    }
  }

  inline void adaptiveKickNow() {
    // ensures runAdaptiveCamera() triggers adaptCameraOnce() at next iteration
    lastTickMs = (millis() > ADAPT_PERIOD_MS) ? (millis() - ADAPT_PERIOD_MS) : 0;
  }

//------------------------------------------------------------------Telemetry Logging Functions-----------------------------------------------------

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
      telemetryBuffer.clear(); // drop any queued samples
      currentTelemetryFile = ""; // forget target file so we re-pick when re-enabled
    }
  }


  void resetCurrentSample() {
    currentSample.timestamp = 0;
    currentSample.voltage = 
# 1280 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1280 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.temp = 
# 1281 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1281 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.charger = 
# 1282 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1282 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_euler_x = 
# 1283 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1283 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_euler_y = 
# 1284 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1284 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_euler_z = 
# 1285 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1285 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_mag_x = 
# 1286 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1286 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_mag_y = 
# 1287 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1287 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_mag_z = 
# 1288 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1288 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.imu_temp = 
# 1289 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1289 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
    currentSample.fps = 
# 1290 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                (__builtin_nanf(""))
# 1290 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                   ;
  }

  void flushTelemetryBufferToSD_Auto() {
    static unsigned long lastFlushTime = 0;
    if (!tlmEnabled) return; // Only log when enabled
    if (!sdPresent()) return; // <- don't even try to open anything if card is ejected

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
      SdLock lock; // 🔒 Mutex lock for SD access

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
          File f = SD.open(fname, "w");
          if (f) {
            f.println(TELEMETRY_CSV_HEADER);
            f.close();
          }
          currentTelemetryFile = fname;
          break;
        } else {
          File f = SD.open(fname, "r");
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
      File f = SD.open(currentTelemetryFile, "a");
      if (!f) {
        do { if (1) Serial.printf("Failed to open %s for appending!\n", currentTelemetryFile.c_str()); } while(0);
        // If the SD was pulled, stop hammering the card. We’ll resume when user re-enables.
        if (!sdPresent()) {
          setTelemetryEnabled(false); // auto-disable to avoid repeated errors
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
        File nf = SD.open(nextFile, "w");
        if (nf) {
          nf.println(TELEMETRY_CSV_HEADER);
          nf.close();
        }
        currentTelemetryFile = nextFile;
        f = SD.open(currentTelemetryFile, "a");
        if (!f) {
          do { if (1) Serial.printf("Failed to open %s for appending!\n", currentTelemetryFile.c_str()); } while(0);
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
          f.print(s.timestamp); f.print(",");
          f.print(s.voltage, 2); f.print(",");
          f.print(s.temp, 1); f.print(",");
          f.print(s.charger, 2); f.print(",");
          f.print(s.imu_euler_x, 2); f.print(",");
          f.print(s.imu_euler_y, 2); f.print(",");
          f.print(s.imu_euler_z, 2); f.print(",");
          f.print(s.imu_mag_x, 2); f.print(",");
          f.print(s.imu_mag_y, 2); f.print(",");
          f.print(s.imu_mag_z, 2); f.print(",");
          f.print(s.imu_temp, 1); f.print(",");
          f.println(s.fps, 1);
      }

      f.close();
    } // 🔓 Lock automatically released here

    telemetryBuffer.clear();
    resetCurrentSample();
  }


//-------------------------------------------------------------------------NPT Tine sync------------------------------------------------------------
  // Call this after WiFi connects
  void startNtpSync() {
    static bool sntpInit = false;

    // Non-blocking: just configure SNTP and return.
    // (Turkey is UTC+3 permanently; no DST.)
    const long gmtOffset_sec = 3 * 3600;
    configTime(gmtOffset_sec, 0,
              "pool.ntp.org",
              "time.google.com",
              "time.windows.com");

    if (!sntpInit) {
      sntp_set_sync_interval(60UL * 60UL * 1000UL); // resync every 1h
      sntp_set_time_sync_notification_cb([](struct timeval*) {
        // no-op; ensures FS timestamps get updated when time changes
      });
      sntpInit = true;
    }
  }

  // Non blocking NPT time sync
  void pollTimeValid() {
    static uint32_t lastPollMs = 0;
    static uint8_t backoffIdx = 0;
    // gentle backoff so we don't spam when offline
    const uint16_t backoffMs[] = {1000, 2000, 4000, 8000, 16000, 30000};

    if (timeIsValid) return;

    // Only try when we actually have internet
    if (WiFi.status() != WL_CONNECTED) return; // or: if (wifiState != WIFI_STA_CONNECTED) return;

    uint32_t now = millis();
    uint16_t interval = backoffMs[backoffIdx];
    if (now - lastPollMs < interval) return;
    lastPollMs = now;

    struct tm tm;
    // IMPORTANT: short timeout to keep this NON-BLOCKING
    if (getLocalTime(&tm, 50)) { // 50 ms max
      if ((tm.tm_year + 1900) >= 2023) {
        timeIsValid = true;
        do { if (1) Serial.print("NTP time is valid: "); } while(0);
        do { if (1) Serial.println(asctime(&tm)); } while(0); // asctime() already has a newline

        //  (optional) set callback once; harmless if repeated
        sntp_set_time_sync_notification_cb([](struct timeval *tv) {
          // No-op; ensures FATFS has timestamps
        });

        backoffIdx = 0; // reset backoff
      }
    } else {
      // didn't get time yet; increase backoff up to max
      if (backoffIdx < (sizeof(backoffMs) / sizeof(backoffMs[0]) - 1)) backoffIdx++;
    }
  }

  void pumpTimeSyncTick() {
    static bool sntpConfigured = false;

    // Only do NTP work when STA is actually connected.
    if (wifiState == WIFI_STA_OK || WiFi.status() == WL_CONNECTED) {
      if (!sntpConfigured) {
        startNtpSync(); // just config, returns immediately
        sntpConfigured = true;
        do { if (1) Serial.println("[NTP] configured (non-blocking)"); } while(0);
      }

      // This is non-blocking (uses getLocalTime(&tm, 50) + backoff).
      pollTimeValid();

    } else {
      // In AP / disconnected: do nothing (no NTP); reset so we re-config next time.
      if (sntpConfigured) {
        sntpConfigured = false;
        // Optional: if you want to forget "valid" time when leaving STA:
        // timeIsValid = false;
        do { if (1) Serial.println("[NTP] paused (not STA-connected)"); } while(0);
      }
    }
  }


//--------------------------------------------------------------------------Lighting----------------------------------------------------------------
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
          for (int i = 0; i < 12; i++)
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


//----------------------------------------------------------------------------OLED------------------------------------------------------------------
  /*screen is 128x64 I2C SSD1306 (“2-color” means 8 rows yellow, 56 blue; common for these displays).*/

  void displayMessage(const String& line1 = "", const String& line2 = "", const String& line3 = "") {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(1 /*|< Draw 'on' pixels*/); // <--- CRITICAL LINE!

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
    display.setTextColor(1 /*|< Draw 'on' pixels*/);
    display.fillRect(0, 0, 128, 16, 0 /*|< Draw 'off' pixels*/); // Clear top
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
    display.setTextColor(1 /*|< Draw 'on' pixels*/);
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

    display.drawRect(0, 35, 128, 10, 1 /*|< Draw 'on' pixels*/);
    int barWidth = map(percent, 0, 100, 0, 124);
    display.fillRect(2, 37, barWidth, 6, 1 /*|< Draw 'on' pixels*/);

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
    float toothWidth = 3.1415926535897932384626433832795 / teeth;
    float outerR = r;
    float innerR = r - 3;

    for (int i = 0; i < teeth; i++) {
      float a = angle + i * 2 * 3.1415926535897932384626433832795 / teeth;

      float ax = cx + cos(a) * innerR;
      float ay = cy + sin(a) * innerR;
      float bx = cx + cos(a + toothWidth / 2) * outerR;
      float by = cy + sin(a + toothWidth / 2) * outerR;
      float cx2 = cx + cos(a + toothWidth) * innerR;
      float cy2 = cy + sin(a + toothWidth) * innerR;

      // Draw tooth triangle
      display.drawLine(ax, ay, bx, by, 1 /*|< Draw 'on' pixels*/);
      display.drawLine(bx, by, cx2, cy2, 1 /*|< Draw 'on' pixels*/);
    }

    // Optional: draw gear hub
    display.drawCircle(cx, cy, 2, 1 /*|< Draw 'on' pixels*/);
  }

  void drawBatteryBar(int x, int y, int width, int height, int percent, bool charging) {
    display.drawRect(x, y, width, height, 1 /*|< Draw 'on' pixels*/);
    display.drawRect(x + width, y + height / 3, 2, height / 3, 1 /*|< Draw 'on' pixels*/); // tip

    int fillHeight = map(percent, 0, 100, 0, height - 2);
    display.fillRect(x + 1, y + height - 1 - fillHeight, width - 2, fillHeight, 1 /*|< Draw 'on' pixels*/);

    if (charging) {
      // Simple animated lightning bolt
      static bool flash = false;
      flash = !flash;

      if (flash) {
        display.setCursor(x + 1, y - 8);
        display.setTextSize(1);
        display.print("⚡"); // Alternatively: use custom shape if ⚡ doesn't render
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
        display.fillRect(bx, by, barWidth, barHeight, 1 /*|< Draw 'on' pixels*/);
      } else {
        display.drawRect(bx, by, barWidth, barHeight, 1 /*|< Draw 'on' pixels*/);
      }
    }
  }

  void drawWebSocketStatus() {
    if (lastWsClientIP != "" && millis() - lastWsConnectTime < 15000) {
      String wsText = "WS: " + lastWsClientIP;
      //playSystemSound("/web/pcm/click.wav");
      display.setTextSize(1);
      display.setTextColor(1 /*|< Draw 'on' pixels*/);

      int16_t x1, y1;
      uint16_t w, h;
      display.getTextBounds(wsText, 0, 0, &x1, &y1, &w, &h);

      if (w < 128) {
        display.setCursor((128 - w) / 2, 56); // bottom center
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
    bool showIP = oledPrefs.getBool("showIP", true);
    bool showBattery = oledPrefs.getBool("showBattery", true);
    bool showWiFi = oledPrefs.getBool("showWiFi", true);

    display.clearDisplay();

    // Show IP at top center (if enabled)
    if (showIP) {
      String ipStr = (WiFi.status() == WL_CONNECTED)
                    ? WiFi.localIP().toString()
                    : "WiFi";
      drawCenteredIP(ipStr);
    }

    // AP mode indicator (top-left)
    if (WiFi.getMode() == WIFI_MODE_AP && WiFi.status() != WL_CONNECTED) {
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
      drawGear(x2, y1, r2, teeth2, -angle1 * ((float)teeth1 / teeth2) + 3.1415926535897932384626433832795 / teeth2);
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
        File f = SD.open(filename, "r");
        if (f) {
          // ---- Allocate buffer dynamically in PSRAM if enabled ----
          uint8_t* buffer = nullptr;

            buffer = (uint8_t*)ps_malloc(2048);



          if (buffer) {
            f.read(buffer, 1024);
            f.close();
            display.drawBitmap(0, 0, buffer, 128, 64, 1 /*|< Draw 'on' pixels*/);
            frame = (frame + 1) % 30;
            free(buffer); // Always free after use!
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
    display.setTextColor(1 /*|< Draw 'on' pixels*/);
    display.setCursor(x, y);
    display.print("AP");
  }

  void animateAP() {
    static uint8_t phase = 0;
    display.clearDisplay();

    int cx = 20, cy = 32;
    display.fillCircle(cx, cy, 3, 1 /*|< Draw 'on' pixels*/); // AP center

    for (int i = 0; i < 3; i++) {
      uint8_t r = 8 + i * 6 + (phase % 6) / 2; // slow ripple
      display.drawCircle(cx, cy, r, 1 /*|< Draw 'on' pixels*/);
    }

    display.setTextSize(1);
    display.setTextColor(1 /*|< Draw 'on' pixels*/);
    display.setCursor(50, 28);
    display.print("AP MODE");
    display.display();

    phase++;
  }

//-----------------------------------------------------------------------------I2C------------------------------------------------------------------

  void i2cStart(){

    Wire.begin(15 /* Use your actual I2C SDA pin for ESP32-S3!*/, 14 /* Use your actual I2C SCL pin for ESP32-S3!*/); // I2C pins
    if (!display.begin(0x02 /*|< Gen. display voltage from 3.3V*/, 0x3C)) {
      do { if (1) Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("❌ OLED init failed"))))); } while(0);
    } else {
      displayMessage("", "🔋 MiniExco Booting", "Please wait");
    }

  }

  bool initBNO055() {
    Wire.begin(15 /* Use your actual I2C SDA pin for ESP32-S3!*/, 14 /* Use your actual I2C SCL pin for ESP32-S3!*/);
    for (int i = 0; i < 5; ++i) {
      if (bno055.begin()) {
        imuPresent = true;
        bno055.setExtCrystalUse(true);
        return true;
      }
      delay(100);
    }
    imuPresent = false;
    do { if (1) Serial.println("❌ BNO055 not detected; continuing without IMU."); } while(0);
    return false;
  }

  void spiStart() {
    SPI.begin(11, 12, 3, 2);

    bool ok = false;

    // --- Prefer new-style config if available (Arduino-ESP32 2.x/3.x) ---
    // SDFSConfig lets us set MaxOpenFiles; header is provided by SD.h in new cores.
# 2030 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
    // --- Fallback: older core signature supports max_files as 5th arg ---
    // SD.begin(cs, spi, freq, mountpoint, max_files)
    {
      ok = SD.begin(2, SPI, 40000000U, "/sd", 24); // <-- raise to 24
    }


    if (!ok) {
      do { if (1) Serial.println("SD Card initialization failed!"); } while(0);
      return;
    }

    do { if (1) Serial.println("SD Card initialized."); } while(0);
    // SD.setTimeCallback(getFatTime); // if you use timestamps

    // Helpers to make dirs safely
    auto ensureDir = [](const char* p) {
      SdLock lock;
      if (!SD.exists(p)) {
        SD.mkdir(p);
        do { if (1) Serial.printf("Created %s folder on SD card.\n", p); } while(0);
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

//----------------------------------------------------------------Helpers for webserver file handling-----------------------------------------------

  static String mimeFor(const String& p_) {
    String p = p_;
    if (p.endsWith(".gz")) p.remove(p.length()-3);

    if (p.endsWith(".html")||p.endsWith(".htm")) return "text/html";
    if (p.endsWith(".css")) return "text/css";
    if (p.endsWith(".js")) return "application/javascript";
    if (p.endsWith(".json")) return "application/json";
    if (p.endsWith(".svg")) return "image/svg+xml";
    if (p.endsWith(".png")) return "image/png";
    if (p.endsWith(".webp")) return "image/webp";
    if (p.endsWith(".avif")) return "image/avif"; // if you try AVIF later
    if (p.endsWith(".jpg")||p.endsWith(".jpeg")) return "image/jpeg";
    if (p.endsWith(".ico")) return "image/x-icon";
    if (p.endsWith(".csv")||p.endsWith(".txt")) return "text/plain";
    return "application/octet-stream";
  }

  static void sendStaticFromWeb(AsyncWebServerRequest* req, const String& urlPath) {
    String fsPath = urlPath.startsWith("/web/") ? urlPath : ("/web" + urlPath);
    if (fsPath.endsWith("/")) fsPath += "index.html";

    String real = fsPath;
    { SdLock lk;
      if (!SD.exists(real)) {
        String gz = fsPath + ".gz";
        if (SD.exists(gz)) real = gz;
        else { req->send(404, "text/plain", "File Not Found"); return; }
      }
    }

    String baseMime = mimeFor(fsPath);
    String mime = real.endsWith(".gz") ? baseMime : mimeFor(real);

    size_t size = 0;
    { SdLock lk; File f = SD.open(real, "r"); if (!f) { req->send(404,"text/plain","File Not Found"); return; } size = f.size(); f.close(); }

    SdGateGuard gate; // <--- serialize whole send
    auto* resp = req->beginResponseStream(mime, size);
    if (real.endsWith(".gz")) { resp->addHeader("Content-Encoding","gzip"); resp->setContentType(baseMime); }
    resp->addHeader("Cache-Control", "public, max-age=86400");

    const size_t BUFSZ = 4096;
    std::unique_ptr<uint8_t[]> buf(new uint8_t[BUFSZ]);

    { SdLock lk; File f = SD.open(real, "r");
      while (true) {
        size_t n = f.read(buf.get(), BUFSZ);
        if (!n) break;
        lk.~SdLock(); resp->write(buf.get(), n); yield(); new (&lk) SdLock;
      }
      f.close();
    }
    req->send(resp);
  }


  static void sendFileFromSD(AsyncWebServerRequest* req, const String& path) {
    { SdLock lk; if (!SD.exists(path)) { req->send(404,"text/plain","File not found"); return; } }

    String mime = mimeFor(path);
    size_t total = 0;
    { SdLock lk; File f = SD.open(path, "r"); if (!f) { req->send(404,"text/plain","File not found"); return; } total = f.size(); f.close(); }

    SdGateGuard gate; // <--- serialize the send
    auto* resp = req->beginResponseStream(mime, total);
    resp->addHeader("Accept-Ranges", "bytes");
    resp->addHeader("Cache-Control", "public, max-age=86400");

    const size_t BUFSZ = 4096;
    std::unique_ptr<uint8_t[]> buf(new uint8_t[BUFSZ]);

    { SdLock lk; File f = SD.open(path, "r");
      while (true) {
        size_t n = f.read(buf.get(), BUFSZ);
        if (!n) break;
        lk.~SdLock(); resp->write(buf.get(), n); yield(); new (&lk) SdLock;
      }
      f.close();
    }
    req->send(resp);
  }

  static void sendFileFromSDWithMime(AsyncWebServerRequest* req,
                                    const String& path,
                                    const String& mime,
                                    bool asAttachment = false)
  {
    { SdLock lk; if (!SD.exists(path)) { req->send(404, "text/plain", "File not found"); return; } }

    size_t total = 0;
    { SdLock lk; File f = SD.open(path, "r"); if (!f) { req->send(404, "text/plain", "File not found"); return; } total = f.size(); f.close(); }

    SdGateGuard gate; // serialize whole send (no overlap with audio or other sends)

    auto* resp = req->beginResponseStream(mime, total);
    resp->addHeader("Cache-Control", "public, max-age=86400");
    if (asAttachment) resp->addHeader("Content-Disposition", "attachment");

    const size_t BUFSZ = 4096;
    std::unique_ptr<uint8_t[]> buf(new uint8_t[BUFSZ]);

    { SdLock lk; File f = SD.open(path, "r");
      while (true) {
        size_t n = f.read(buf.get(), BUFSZ);
        if (!n) break;
        lk.~SdLock(); // release lock during network write
        resp->write(buf.get(), n);
        yield();
        new (&lk) SdLock; // re-acquire for next SD read
      }
      f.close();
    }

    req->send(resp);
  }


//-----------------------------------------------------------------------SD File Management---------------------------------------------------------

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
              do { if (1) Serial.println("Failed to open dir for count!"); } while(0);
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
          do { if (1) Serial.printf("Counted %d files for indexing in %s\n", reindexTotal, reindexPath.c_str()); } while(0);
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
          idx = SD.open(reindexPath + "/.index", "w");
          if (!dir || !dir.isDirectory() || !idx) {
              pendingReindex = false;
              do { if (1) Serial.println("Failed to start reindex!"); } while(0);
              return;
          }
          f = dir.openNextFile();
          reindexCount = 0;
          reindexReadyToIndex = false; // Reset so doesn't re-enter
          do { if (1) Serial.printf("Begin background indexing for %s\n", reindexPath.c_str()); } while(0);
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
          do { if (1) Serial.printf("Index for %s finished, %d files\n", reindexPath.c_str(), reindexCount); } while(0);
      }
  }

  // Utility: read a batch from .index file
  void readSdIndexBatch(const String& idxPath, int start, int count, JsonArray& arr, bool showSystem /*= false*/) {
    if (start < 0) start = 0;
    if (count < 1) count = 1;

    SdLock lock; // safe with recursive mutex
    File idx = SD.open(idxPath, "r");
    if (!idx) return;

    delay(0); // initial yield after open

    int idxLine = 0;
    int added = 0;
    uint32_t tick = 0;
    String line;
    bool isEmptyMarker = false;

    while (idx.available()) {
      line = idx.readStringUntil('\n');
      line.trim();

      if (line == "__EMPTY__") { // fast exit if index marks empty dir
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

      if (comma1 < 0 || comma2 < 0) { // malformed line
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
          name.endsWith(".bak") ||
          name.endsWith(".meta") ||
          name.startsWith(".") ||
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
      obj["name"] = name;
      obj["isFolder"] = isFolder;
      if (!isFolder) obj["size"] = size;
      obj["type"] = isFolder ? "folder" : "default";
      if (date > 0) obj["date"] = (uint32_t)date * 1000; // seconds -> ms

      added++;

      if (((++tick) & 0x7F) == 0) delay(0); // periodic yield to feed WDT
    }

    idx.close();
    delay(0); // final cooperative yield

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

    vTaskDelay(( ( TickType_t ) ( ( ( TickType_t ) ( ms ) * ( TickType_t ) ( 
# 2458 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
              1000 
# 2458 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
              ) ) / ( TickType_t ) 1000U ) ));



  }

  void initSdGate() {
    g_sdStreamGate = xQueueGenericCreate( ( UBaseType_t ) 1, ( ( uint8_t ) 0U ), ( ( uint8_t ) 3U ) );
    xQueueGenericSend( ( QueueHandle_t ) ( g_sdStreamGate ), 
# 2466 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
   __null
# 2466 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
   , ( ( TickType_t ) 0U ), ( ( BaseType_t ) 0 ) ); // gate is free at boot
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
      f.close(); // close before any yield

      if (isDir) {
        if (name != "." && name != "..") {
          // Recurse into subdir
          removeOrphanTempsInDir(path + "/" + name);
        }
      } else {
        if (name.endsWith(".upload.tmp")) {
          String doomed = path + "/" + name;
          { SdLock lock; SD.remove(doomed); } // lock only around SD op
          do { if (1) Serial.printf("[CLEAN] removed orphan temp: %s\n", doomed.c_str()); } while(0);
        }
      }

      // Give the scheduler a breath after each entry (no SD lock held)

      vTaskDelay(1);



    }

    dir.close();
  }

  void auditPcmAssets() {
    const char* req[] = {"0.wav","1.wav","2.wav","3.wav","4.wav","5.wav","6.wav","7.wav","8.wav","9.wav","dot.wav","apaudio.wav","connected.wav"};
    for (auto f : req) {
      String p = String("/web/pcm/") + f;
      if (!SD.exists(p)) {
        String pu = p; pu.replace(".wav",".WAV");
        if (!SD.exists(pu)) do { if (1) Serial.printf("[PCM MISSING] %s (or %s)\n", p.c_str(), pu.c_str()); } while(0);
      }
    }
  }

//-------------------------------------------------------------------------System Sound-------------------------------------------------------------

  void playSystemSound(const char* filename) {
      if (!sSndEnabled || !filename || !filename[0]) return;

      // NEW: drop any enqueues while exclusive, except the ones we are explicitly allowing now
      if (ss_exclusive && !ss_allow) return;

      unsigned long now = millis();
      // Ignore if file is same and within debounce period
      if (lastPlayedFile == filename && (now - lastPlayedTime < soundRepeatDelay)) return;

      // If something is playing, enqueue the request if there's space
      if (isSystemSoundPlaying) {
          // Add to queue only if not a duplicate in the queue
          for (uint8_t i = queueHead; i != queueTail; i = (i + 1) % 24) {
              if (systemSoundQueue[i] == filename) return; // Already in queue
          }
          uint8_t nextTail = (queueTail + 1) % 24;
          if (nextTail != queueHead) { // Queue not full
              systemSoundQueue[queueTail] = filename;
              queueTail = nextTail;
          }
          return;
      }

      // Not playing, start playing this one
      sSndVolume = ((sSndVolume)<(0)?(0):((sSndVolume)>(21)?(21):(sSndVolume)));
      audio.setVolume(sSndVolume);
      isSystemSoundPlaying = true;
      lastPlayedFile = filename;
      lastPlayedTime = now;
      playWavFileOnSpeaker(filename); // This should be non-blocking
  }

  void onSystemSoundFinished() {
    isSystemSoundPlaying = false;

    // Release exclusive gate held for the beep
    if (g_gateHeldByAudio) {
      g_gateHeldByAudio = false;
      xQueueGenericSend( ( QueueHandle_t ) ( g_sdStreamGate ), 
# 2567 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
     __null
# 2567 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
     , ( ( TickType_t ) 0U ), ( ( BaseType_t ) 0 ) );
    }

    // Chain next system sound if queued (they’ll preempt again)
    if (queueHead != queueTail) {
      String nextFile = systemSoundQueue[queueHead];
      queueHead = (queueHead + 1) % 24;
      playSystemSound(nextFile.c_str());
      return;
    }

    // If we paused media to play the system sound, resume it now
    if (g_mediaPausedBySystem) {
      g_mediaPausedBySystem = false;
      resumeAudio(); // your resume starts track from the beginning; acceptable
    }
    if (ss_exclusive && (queueHead == queueTail) && !audio.isRunning() && !isSystemSoundPlaying) {
      ss_exclusive = false;
    }
  }

  // Speak an IP address by queueing WAV digits + a dot sound
  static inline void _enqueueDigit(char d) {
    String f = "/web/pcm/";
    f += d;
    f += ".wav";
    playSystemSound(f.c_str());
  }

  static inline void _enqueueDot() {
    playSystemSound("/web/pcm/dot.wav"); // add a very short “dot” (or “space.wav” if preferred)
  }

  void speakNumber(uint16_t n) {
    String s = String(n);
    for (uint16_t i = 0; i < s.length(); i++) _enqueueDigit(s[i]);
  }

  void speakIPAddress(const IPAddress& ip) {
    // Temporarily disable de-bounce to allow repeated digits (e.g., "1.wav" twice)
    unsigned long _prevDelay = soundRepeatDelay;
    soundRepeatDelay = 0;

    for (int i = 0; i < 4; i++) {
      speakNumber(ip[i]); // enqueues each digit of the octet
      if (i < 3) _enqueueDot();
    }

    // Restore original setting
    soundRepeatDelay = _prevDelay;
  }


  inline bool soundIsPlaying() {
    // Your code already has isSystemSoundPlaying; double-guard with audio.isRunning()
    return isSystemSoundPlaying || audio.isRunning();
  }

  void scheduleApSpeechSoon(uint32_t delayMs = 700) {
    if (ss_exclusive || ss_apSpeechPending) return; // NEW: prevent re-arming
    ss_apSpeechPending = true;
    ss_apSpeechAtMs = millis() + delayMs;
  }

  void speakStaIpOrDefer(const IPAddress& ip) {
    // If something is playing, defer until queue is idle
    if (soundIsPlaying()) {
      ss_staSpeechPending = true;
      ss_staIpToSpeak = ip;
      return;
    }
    playSystemSound("/web/pcm/connected.wav");
    speakIPAddress(ip);
  }

  void pumpSystemSoundScheduler(uint32_t now) {
    // Fire pending AP speech once loop is alive and queue is idle
    if (ss_apSpeechPending && now >= ss_apSpeechAtMs && !soundIsPlaying() && !ss_exclusive) {
      ss_exclusive = true;
      ss_allow = true;
      queueHead = queueTail = 0;

      playSystemSound("/web/pcm/apaudio.wav");

      unsigned long _prevDelay = soundRepeatDelay;
      soundRepeatDelay = 0;
      speakIPAddress(WiFi.softAPIP());
      soundRepeatDelay = _prevDelay;

      ss_allow = false;
      ss_apSpeechPending = false;
    }

    // Fire deferred STA speech when queue is idle
    if (ss_staSpeechPending && !soundIsPlaying() && !ss_exclusive) {
      ss_exclusive = true;
      ss_allow = true;
      queueHead = queueTail = 0;

      playSystemSound("/web/pcm/connected.wav");

      unsigned long _prevDelay = soundRepeatDelay;
      soundRepeatDelay = 0;
      speakIPAddress(ss_staIpToSpeak);
      soundRepeatDelay = _prevDelay;

      ss_allow = false;
      ss_staSpeechPending = false;
    }
  }



//----------------------------------------------------------------------WiFi Stack Management-------------------------------------------------------

  void WIFI_LOG_MODE(wifi_mode_t t, const char* where){
    Serial.printf("[WIFI] mode(%d) at %s (cur=%d) t=%lu\n", (int)t, where, (int)WiFi.getMode(), millis());
    if (WiFi.getMode()!=t) WiFi.mode(t);
  }
  void WIFI_LOG_DISC(bool erase, bool off, const char* where){
    Serial.printf("[WIFI] disconnect(erase=%d, off=%d) at %s t=%lu\n",
                  erase, off, where, millis());
    WiFi.disconnect(erase, off);
  }

  static void startApLobby() {
    // AP+STA so UI stays reachable while we hunt for Wi-Fi
    if (WiFi.getMode() != WIFI_MODE_APSTA) WiFi.mode(WIFI_MODE_APSTA);

    if (g_apPassword.length() >= 8 && g_apPassword.length() <= 63)
      WiFi.softAP("MINIEXCO_S3_V1_02", g_apPassword.c_str());
    else
      WiFi.softAP("MINIEXCO_S3_V1_02");

    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    WiFi.setSleep(true);

    showWiFiStep(String("AP:\n") + WiFi.softAPIP().toString());
    wifiState = WIFI_AP_LOBBY;
    wifiLastScanAt = 0; // force immediate scan
    currentCandidateIndex = -1; // restart candidate iteration
  }

  static void stopApIfRunning() {
    if (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) {
      WiFi.softAPdisconnect(true);
    }
  }

  static void switchToStaOnly() {
    stopApIfRunning();
    if (WiFi.getMode() != WIFI_MODE_STA) WiFi.mode(WIFI_MODE_STA);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // safe for Bluepad32
    WiFi.setSleep(true);
  }

  static bool hasAnySavedNetworks() {
    String list = wifiPrefs.getString("networks", "");
    if (list.length()) {
      for (int i = 0; i < (int)list.length(); ++i) {
        char c = list[i];
        if (c != ',' && c != ' ' && c != '\t' && c != '\r' && c != '\n') return true;
      }
    }
    if (wifiPrefs.getString("preferred_ssid", "").length()) return true;
    return false;
  }

  // Build the ordered list: preferred first (deduped), then others
  static void loadOrderedSavedList(std::vector<String>& ssidsOut) {
    ssidsOut.clear();
    // preferred first
    String pref = normalizedSSID(wifiPrefs.getString("preferred_ssid", ""));
    if (pref.length()) ssidsOut.push_back(pref);

    // then the rest (dedup)
    String list = wifiPrefs.getString("networks", "");
    int last = 0;
    while (true) {
      int next = list.indexOf(',', last);
      String item = normalizedSSID((next == -1) ? list.substring(last) : list.substring(last, next));
      if (item.length()) {
        bool dup = false;
        for (auto &s : ssidsOut) { if (s == item) { dup = true; break; } }
        if (!dup) ssidsOut.push_back(item);
      }
      if (next == -1) break;
      last = next + 1;
    }
  }

  static void beginConnectTo(const String& ssid) {
    String pass = wifiPrefs.getString(("wifi_" + ssid).c_str(), "");
    bool aRt = wifiPrefs.getBool(("aRt_" + ssid).c_str(), true);
    if (!aRt) return; // skip SSIDs user disabled

    // keep AP up while we attempt (we are in AP_LOBBY)
    if (WiFi.getMode() != WIFI_MODE_APSTA) WiFi.mode(WIFI_MODE_APSTA);

    showWiFiStep("Connecting:\n" + ssid);
    wifiSSID = ssid;
    wifiPassword = pass;
    WiFi.disconnect(true, false); // clean slate, keep radio on
    delay(50);
    WiFi.begin(ssid.c_str(), pass.c_str());
    wifiConnectSince = millis();
    wifiState = WIFI_STA_WAIT;
  }

  void handleWifiSimple() {
    // 1) If we are connected, ensure STA-only and monitor for drop
    if (WiFi.status() == WL_CONNECTED) {
      if (wifiState != WIFI_STA_OK) {
        switchToStaOnly();
        showWiFiStep("Connected:\n" + WiFi.localIP().toString(), true);
        playSystemSound("/web/pcm/connected.wav");
        speakIPAddress(WiFi.localIP());
        wifiState = WIFI_STA_OK;
      }
      return; // nothing else to do while happily connected
    }

    // 2) Not connected → ensure AP lobby is running
    if (wifiState == WIFI_STA_OK && WiFi.status() != WL_CONNECTED) {
      // lost link → go back to AP lobby and retry forever
      startApLobby();
    }

    if (wifiState == WIFI_AP_LOBBY) {
      // No saved networks? Just stay AP.
      if (!hasAnySavedNetworks()) return;

      // Periodic async scan
      if (millis() - wifiLastScanAt >= SCAN_PERIOD_MS || wifiLastScanAt == 0) {
        wifiLastScanAt = millis();
        showWiFiStep("Scanning for\nknown Wi-Fi...");
        WiFi.scanDelete();
        WiFi.scanNetworks(true); // async
        return;
      }

      int sc = WiFi.scanComplete();
      if (sc >= 0) {
        // Build candidate list once per scan
        std::vector<String> ssids;
        loadOrderedSavedList(ssids);

        // Iterate candidates, pick the first present in results
        for (size_t i = 0; i < ssids.size(); ++i) {
          String s = ssids[i];
          bool present = false;
          for (int j = 0; j < sc; ++j) if (WiFi.SSID(j) == s) { present = true; break; }
          if (present) {
            beginConnectTo(s);
            WiFi.scanDelete();
            return;
          }
        }
        // Nothing found in this scan; keep AP and wait for next period
        WiFi.scanDelete();
        showWiFiStep("No known Wi-Fi\nStill in AP");
      }
      // sc == -1 → scan running; sc == -2 → none yet (next tick will start)
      return;
    }

    if (wifiState == WIFI_STA_WAIT) {
      // wait for connect or timeout
      if (WiFi.status() == WL_CONNECTED) {
        // the connected branch at top will handle transition next tick
        return;
      }
      if (millis() - wifiConnectSince >= CONNECT_TIMEOUT) {
        // timeout → back to lobby (AP kept running the whole time)
        wifiState = WIFI_AP_LOBBY;
        showWiFiStep("Timeout.\nBack to AP");
      }
      return;
    }
  }

  // 👇 Helper: Stop server + WebSocket cleanly before Wi-Fi switch
  void stopWebServerAndWS() {
      wsCarInput.closeAll(); // Close all websocket clients
      server.end(); // Fully stop the HTTP server
      do { if (1) Serial.println("✅ WebServer + WebSocket stopped."); } while(0);
  }

  void webServerReboot() {
      Serial.println("[WebServer] Forcibly restarting AsyncWebServer stack...");
      stopWebServerAndWS(); // Your function to stop/end server and WS
      delay(100);
      serverStart(); // Or startWebServer(), whatever re-adds handlers and begins server
      Serial.println("[WebServer] AsyncWebServer stack restarted.");
  }

  bool connectToWiFiWithRetries(const String& ssid, const String& password, int retries) {

    wifi_mode_t m = WiFi.getMode();
    if (m != WIFI_MODE_STA){
      //WiFi.mode(WIFI_STA);

      WIFI_LOG_MODE(WIFI_MODE_STA, "site2880");
    }

    WiFi.begin(ssid.c_str(), password.c_str());

    for (int attempt = 1; attempt <= retries; attempt++) {
      int progress = map(attempt - 1, 0, retries, 0, 100);
      showWiFiProgress(progress);

      do { if (1) Serial.printf("🔁 Attempt %d to connect to WiFi SSID: %s\n", attempt, ssid.c_str()); } while(0);
      unsigned long startAttemptTime = millis();

      // Keep attempt short; yield often
      while (millis() - startAttemptTime < 1200) {
        if (WiFi.status() == WL_CONNECTED) {
          do { if (1) Serial.println("✅ Connected to WiFi!"); } while(0);
          displayMessage("", "✅ WiFi Connected\n" + WiFi.localIP().toString());
          delay(120);
          showCheckmark();
          return true;
        }
        delay(100);
        yield();
      }

      // Prepare next try
      WIFI_LOG_DISC(true, false, "site2866");
      delay(80);
      WiFi.begin(ssid.c_str(), password.c_str());
    }

    do { if (1) Serial.println("❌ Failed to connect after retries."); } while(0);
    return false;
  }

  void handleWiFiSetup(AsyncWebServerRequest *request) {
    if (SD.exists("/WiFiPages.html")) {
      sendFileFromSDWithMime(request, "/WiFiPages.html", "text/html");
    } else {
      request->send(404, "text/plain", "WiFiPages.html not found on SD card");
    }
  }

  void handleListWiFi(AsyncWebServerRequest *request) {
    int n = WiFi.scanComplete();

    if (n == -2) {
      // No scan yet, start it
      WiFi.scanNetworks(true);
      request->send(202, "application/json", "[]"); // Accept request, but scanning now
      return;
    }

    if (n == -1) {
      // Scan ongoing
      request->send(202, "application/json", "[]"); // Accept request, still scanning
      return;
    }

    // Scan complete, return results
    String json = "[";
    for (int i = 0; i < n; ++i) {
      if (i) json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";

    WiFi.scanDelete(); // Free memory
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
        sendFileFromSDWithMime(request, "/wifi_success.html", "text/html");
      } else {
        request->send(200, "text/html",
          "<html><body><h1>✅ Wi-Fi Credentials Saved</h1><p>Rebooting and trying to connect...</p></body></html>");
      }

      delay(3000);
      ESP.restart(); // ← Optional: remove if you want to stay up and let the loop connect
    } else {
      request->send(400, "text/html", "<html><body><h1>Missing SSID or Password</h1></body></html>");
    }
  }

//---------------------------------------------------------------------AP Password Function---------------------------------------------------------

  bool setApPassword(const String& pw) {
    if (pw.length() && (pw.length() < 8 || pw.length() > 63)) return false; // reject invalid
    g_apPassword = pw; // allow empty to make AP open
    uiPrefs.putString("ap_pass", g_apPassword); // persist
    return true;
  }


//------------------------------------------------------------------------Robot Controls------------------------------------------------------------

  void rightMotorForward(uint8_t pwm = 255) {
    ledcWrite(0, pwm);
    ledcWrite(1, 0);
  }
  void rightMotorBackward(uint8_t pwm = 255) {
    ledcWrite(0, 0);
    ledcWrite(1, pwm);
  }
  void rightMotorStop() {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
  void leftMotorForward(uint8_t pwm = 255) {
    ledcWrite(2, pwm);
    ledcWrite(3, 0);
  }
  void leftMotorBackward(uint8_t pwm = 255) {
    ledcWrite(2, 0);
    ledcWrite(3, pwm);
  }
  void leftMotorStop() {
    ledcWrite(2, 0);
    ledcWrite(3, 0);
  }
  void armMotorUp(uint8_t pwm = 255) {
    ledcWrite(4, pwm);
    ledcWrite(5, 0);
  }
  void armMotorDown(uint8_t pwm = 255) {
    ledcWrite(4, 0);
    ledcWrite(5, pwm);
  }
  void armMotorStop() {
    ledcWrite(4, 0);
    ledcWrite(5, 0);
  }

  void moveCar(int inputValue, int pwm = 255) {
    if (!horScreen) { // Normal orientation
      switch (inputValue) {
        case 1:
          rightMotorForward(pwm);
          leftMotorForward(pwm);
          break;
        case 2:
          rightMotorBackward(pwm);
          leftMotorBackward(pwm);
          break;
        case 3:
          rightMotorForward(pwm);
          leftMotorBackward(pwm);
          break;
        case 4:
          rightMotorBackward(pwm);
          leftMotorForward(pwm);
          break;
        case 0:
          rightMotorStop();
          leftMotorStop();
          armMotorStop();
          break;
        case 5:
          armMotorUp(pwm);
          break;
        case 6:
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
        case 1: // Rotate UP -> RIGHT
          rightMotorBackward(pwm);
          leftMotorForward(pwm);
          break;
        case 2: // Rotate DOWN -> LEFT
          rightMotorForward(pwm);
          leftMotorBackward(pwm);
          break;
        case 3: // Rotate LEFT -> DOWN
          rightMotorBackward(pwm);
          leftMotorBackward(pwm);
          break;
        case 4: // Rotate RIGHT -> UP
          rightMotorForward(pwm);
          leftMotorForward(pwm);
          break;
        case 0:
          rightMotorStop();
          leftMotorStop();
          armMotorStop();
          break;
        case 5:
          armMotorUp(pwm);
          break;
        case 6:
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
    angle = ((angle)<(0)?(0):((angle)>(180)?(180):(angle)));
    int pulse_us = map(angle, 0, 180, 500, 2500);
    uint32_t duty = ((uint64_t)pulse_us << 14) / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
  }

  void bucketTilt(int bucketServoValue) {
    if (!bucketAttached) {
      // Use low-speed timer explicitly
      ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_2, // 🔄 CHANGED from 0 → 2
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
      };

      ledc_timer_config(&timer);

      ledc_channel_config_t ch = {
        .gpio_num = 0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)6,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0
      };
      ledc_channel_config(&ch);
      bucketAttached = true;
      do { if (1) Serial.println("🪣 Bucket PWM configured (low-speed)"); } while(0);
    }

    writeServo((ledc_channel_t)6, bucketServoValue);

    lastBucketValue = bucketServoValue;
    preferences.putInt("bucketAngle", lastBucketValue);
    if (!holdBucket) bucketDetachTime = millis() + 300;
  }

  void auxControl(int auxServoValue) {
    if (!auxAttached) {
      ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_3, // 🔄 CHANGED from 1 → 3
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
      };

      ledc_timer_config(&timer);

      ledc_channel_config_t ch = {
        .gpio_num = 20,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = (ledc_channel_t)7,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_3, // use a separate timer from bucket
        .duty = 0,
        .hpoint = 0
      };
      ledc_channel_config(&ch);
      auxAttached = true;
      do { if (1) Serial.println("🔧 AUX PWM configured (low-speed)"); } while(0);
    }

    writeServo((ledc_channel_t)7, auxServoValue);

    lastAuxValue = auxServoValue;
    preferences.putInt("auxAngle", lastAuxValue);
    if (!holdAux) auxDetachTime = millis() + 300;
  }

  void setUpPinModes() {
    const int pwmFreq = 1000;
    const int pwmRes = 8;

    // --- RIGHT motor ---
    pinMode(16, 0x03);
    pinMode(17, 0x03);
    ledcSetup(0, pwmFreq, pwmRes);
    ledcAttachPin(16, 0);
    ledcSetup(1, pwmFreq, pwmRes);
    ledcAttachPin(17, 1);

    // --- LEFT motor ---
    pinMode(18, 0x03);
    pinMode(-1, 0x03);
    ledcSetup(2, pwmFreq, pwmRes);
    ledcAttachPin(18, 2);
    ledcSetup(3, pwmFreq, pwmRes);
    ledcAttachPin(-1, 3);

    // --- ARM motor ---
    pinMode(-1, 0x03);
    pinMode(-1, 0x03);
    ledcSetup(4, pwmFreq, pwmRes);
    ledcAttachPin(-1, 4);
    ledcSetup(5, pwmFreq, pwmRes);
    ledcAttachPin(-1, 5);

    // --- Force stop all motors at start ---
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, 0);
    ledcWrite(5, 0);

    // Optionally also drive pins LOW in case FETs "leak":
    digitalWrite(16, 0x0);
    digitalWrite(17, 0x0);
    digitalWrite(18, 0x0);
    digitalWrite(-1, 0x0);
    digitalWrite(-1, 0x0);
    digitalWrite(-1, 0x0);
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
      else armMotorDown(pwm);
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
      do { if (1) Serial.println("Lights ON"); } while(0);
      playSystemSound("/web/pcm/hit.wav");
    } else {
      //digitalWrite(lightPin1, LOW);
      //digitalWrite(lightPin2, LOW);
      light = false;
      do { if (1) Serial.println("Lights OFF"); } while(0);
      playSystemSound("/web/pcm/click.wav");
    }
  }

  void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT: {
        do { if (1) Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str()); } while(0);
        lastWsClientIP = client->remoteIP().toString();
        lastWsConnectTime = millis();

        // >>> ADDED: watchdog book-keeping
        wsActiveClients = server->count();
        hadAnyClientSinceBoot = true;
        lastWebActivityMs = millis();
        // <<<

        client->text("HoldBucket," + String(holdBucket ? 1 : 0));
        client->text("HoldAux," + String(holdAux ? 1 : 0));
        client->text("Switch," + String(horScreen ? 1 : 0));
        client->text("DarkMode," + String(darkMode ? 1 : 0));
        client->text("RecordTelemetry," + String(tlmEnabled ? 1 : 0));
        client->text("SystemSounds," + String(sSndEnabled ? 1 : 0));
        client->text("GamepadEnabled," + String(bluepadEnabled ? 1 : 0));
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
      } break;

      case WS_EVT_DISCONNECT: {
        do { if (1) Serial.printf("WebSocket client #%u disconnected\n", client->id()); } while(0);
        lastWsClientIP = ""; // Clear
        moveCar(0);

        // >>> ADDED: watchdog book-keeping
        wsActiveClients = server->count();
        lastWsDisconnectMs = millis();
        // <<<
      } break;

      // >>> ADDED: count any traffic/pongs as activity
      case WS_EVT_PONG: {
        lastWebActivityMs = millis();
      } break;

      case WS_EVT_ERROR: {
        lastWebActivityMs = millis();
        do { if (1) Serial.printf("WebSocket error on client #%u\n", client->id()); } while(0);
      } break;
      // <<<

      case WS_EVT_DATA: {
        // >>> ADDED: any data counts as activity
        lastWebActivityMs = millis();
        // <<<

        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

          // ---------- JSON branch for key events ----------
          if (len > 0 && ((const char*)data)[0] == '{') {
            StaticJsonDocument<128> jd;
            DeserializationError derr = deserializeJson(jd, (const char*)data, len);
            if (!derr) {
              const char* t = jd["t"] | "";
              if (strcmp(t, "key") == 0) {
                const char* code = jd["code"] | "";
                const char* st = jd["state"] | "down";
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
          char* buf = (char*)
# 3364 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                            __builtin_alloca(
# 3364 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                            len + 1
# 3364 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                            )
# 3364 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                           ; // stack buffer to avoid heap fragmentation
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
            do { if (1) Serial.printf("[WS] MEDIA_PLAY: %s\n", value1.c_str()); } while(0);
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

          else if (key == "RADIO_PLAY") {
            const char* urlc = value1.c_str(); // value1 is std::string
            do { if (1) Serial.printf("[AUDIO] RADIO_PLAY %s\n", urlc); } while(0);

            disableMic();
            stopAudio();
            xQueueSemaphoreTake( ( g_sdStreamGate ), ( ( TickType_t ) 0xffffffffUL ) );
            g_gateHeldByAudio = true;

            enableSpeaker();
            audio.connecttohost(urlc); // ESP32-audioI2S expects const char*
            playbackStarted = true;
            isPaused = false;

            wsCarInput.textAll(String("MEDIA_DEVICE_PLAYING,") + urlc);
          }


          else if (key == "MEDIA_STOP") { stopAudio(); wsCarInput.textAll("MEDIA_DEVICE_STOPPED"); isPaused = false; }
          else if (key == "MEDIA_PAUSE") { pauseAudio(); wsCarInput.textAll("MEDIA_DEVICE_PAUSED"); }
          else if (key == "MEDIA_RESUME") { resumeAudio(); wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + playlist[currentTrack]); }
          else if (key == "MEDIA_PREV") { prevTrack(); wsCarInput.textAll("MEDIA_DEVICE_PLAYING," + playlist[currentTrack]); }
          else if (key == "MEDIA_LOOP_ON") { loopMode = true; }
          else if (key == "MEDIA_LOOP_OFF") { loopMode = false; }
          else if (key == "MEDIA_SHUFFLE_ON") { shuffleMode = true; }
          else if (key == "MEDIA_SHUFFLE_OFF") { shuffleMode = false; }

          // --- MIC STREAM COMMANDS ---
          if (key == "START_MIC_STREAM") { micStreamActive = true; client->text("MIC_STREAM_ON"); }
          else if (key == "STOP_MIC_STREAM") { micStreamActive = false; client->text("MIC_STREAM_OFF"); }

          // --- Turn signals via Slider ---
          if (key == "Slider") {
            String side = value1.c_str();
            int val = atoi(value2.c_str());
            if (side == "Left") {
              if (val > 5 && !leftSignalActive) { leftSignalActive = true; updatePixels(); wsCarInput.textAll("TURN_LEFT,1"); }
              else if (val <= 5 && leftSignalActive) { leftSignalActive = false; updatePixels(); wsCarInput.textAll("TURN_LEFT,0"); }
            } else if (side == "Right") {
              if (val > 5 && !rightSignalActive) { rightSignalActive = true; updatePixels(); wsCarInput.textAll("TURN_RIGHT,1"); }
              else if (val <= 5 && rightSignalActive) { rightSignalActive = false; updatePixels(); wsCarInput.textAll("TURN_RIGHT,0"); }
            }
          }

          if (key == "Beacon") { beaconOn = !beaconOn; updatePixels(); }
          if (key == "Emergency") { emergencyOn = !emergencyOn; updatePixels(); }

          if (key == "Motor") {
            int pwm = atoi(value2.c_str());
            controlMotorByDirection(value1, pwm);
          }

          do { if (1) Serial.printf("Key [%s] Value1[%s] Value2[%s]\n", key.c_str(), value1.c_str(), value2.c_str()); } while(0);
          int valueInt = atoi(value1.c_str());

          if (key == "MoveCar") {
            int pwm = (value2.empty() ? 255 : atoi(value2.c_str()));
            moveCar(valueInt, pwm);
            do { if (1) Serial.printf("[WS] moveCar called with valueInt=%d pwm=%d\n", valueInt, pwm); } while(0);
          }
          else if (key == "AUX") auxControl(valueInt);
          else if (key == "Bucket") bucketTilt(valueInt);
          else if (key == "Light") {
            lightControl(); updatePixels();
            wsCarInput.textAll("Light," + String(light ? 1 : 0));
          }
          else if (key == "Switch") { horScreen = (valueInt != 0); uiPrefs.putBool("Switch", horScreen); }
          else if (key == "HoldBucket") { holdBucket = (valueInt != 0); uiPrefs.putBool("HoldBucket", holdBucket); }
          else if (key == "HoldAux") { holdAux = (valueInt != 0); uiPrefs.putBool("HoldAux", holdAux); }
          else if (key == "DarkMode") { darkMode = (valueInt != 0); uiPrefs.putBool("darkMode", darkMode); }
          else if (key == "RecordTelemetry") { setTelemetryEnabled(valueInt != 0); }
          else if (key == "SystemSounds") { int v = atoi(value1.c_str()); sSndEnabled = (v != 0); uiPrefs.putBool("SystemSounds", sSndEnabled); }
          else if (key == "GamepadEnabled") { int v = atoi(value1.c_str()); bluepadEnabled = (v != 0); uiPrefs.putBool("BluepadEnabled", bluepadEnabled); }
          else if (key == "SystemVolume") { int v = atoi(value1.c_str()); sSndVolume = v; uiPrefs.putInt("SystemVolume", sSndVolume); }

        } // if (frame ok)
        break; // <-- IMPORTANT: end of WS_EVT_DATA case
      } // <-- end of case block
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

  static inline void joymapLoadToJson(JsonObject root) {
    for (size_t i=0;i<JOYMAP_N;++i) {
      const char* k = JOYMAP_DEFAULTS[i].action;
      const char* d = JOYMAP_DEFAULTS[i].defBtn;
      root[k] = normJoyBtn( joymapPrefs.getString(k, d) ); // canonicalize
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
    if (k=="arrowup") return "arrowup";
    if (k=="arrowdown") return "arrowdown";
    if (k=="arrowleft") return "arrowleft";
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


  // ---------- Central dispatcher (call your actual robot functions here) ----------
  // NOTE: Replace the TODOs with your real control functions.
  static void dispatchAction(Action a, bool pressed) {
    // Movement: act on press; release to stop if needed
    switch (a) {
      case ACT_FORWARD: /* TODO: on press go forward; on release stop/neutral */ break;
      case ACT_BACKWARD: /* TODO */ break;
      case ACT_LEFT: /* TODO */ break;
      case ACT_RIGHT: /* TODO */ break;
      case ACT_STOP: if (pressed) {/* TODO: stop all motion */} break;

      case ACT_BUCKET_UP: if (pressed) {/* TODO: bucketTilt(+step) or start hold */} break;
      case ACT_BUCKET_DOWN: if (pressed) {/* TODO: bucketTilt(-step) or start hold */} break;
      case ACT_AUX_UP: if (pressed) {/* TODO: auxControl(+step) */} break;
      case ACT_AUX_DOWN: if (pressed) {/* TODO: auxControl(-step) */} break;

      case ACT_LIGHT_TOGGLE: if (pressed) {/* TODO: toggle light */} break;
      case ACT_BEACON_TOGGLE: if (pressed) {/* TODO */} break;
      case ACT_EMERGENCY_TOGGLE: if (pressed) {/* TODO */} break;
      case ACT_HORN: /* on press: start horn; on release: stop horn */ break;
      default: break;
    }
  }

  //--------------------------Joystick helpers------------------------------------------

  // --- Joystick button normalizer (accept RB/LB/RT/LT etc., return canonical tokens) ---
  static String normJoyBtn(String s) {
    s.trim(); s.toUpperCase();

    // Face
    if (s == "A" || s == "BTN_A") return "A";
    if (s == "B" || s == "BTN_B") return "B";
    if (s == "X" || s == "BTN_X") return "X";
    if (s == "Y" || s == "BTN_Y") return "Y";

    // D-Pad
    if (s == "UP" || s == "DPAD-UP" || s == "DUP") return "DPAD_UP";
    if (s == "DOWN" || s == "DPAD-DOWN" || s == "DDOWN") return "DPAD_DOWN";
    if (s == "LEFT" || s == "DPAD-LEFT" || s == "DLEFT") return "DPAD_LEFT";
    if (s == "RIGHT" || s == "DPAD-RIGHT" || s == "DRIGHT") return "DPAD_RIGHT";

    // Bumpers
    if (s == "RB" || s == "R1" || s == "BUTTON_R1") return "R1";
    if (s == "LB" || s == "L1" || s == "BUTTON_L1") return "L1";

    // Triggers-as-click (digital)
    if (s == "RT" || s == "RT_CLICK" || s == "TRIGGER_RIGHT") return "R2_CLICK";
    if (s == "LT" || s == "LT_CLICK" || s == "TRIGGER_LEFT") return "L2_CLICK";

    // Stick clicks
    if (s == "LS" || s == "L3" || s == "L_STICK") return "L3";
    if (s == "RS" || s == "R3" || s == "R_STICK") return "R3";

    // Menu
    if (s == "START" || s == "BTN_START") return "BTN_START";
    if (s == "BACK" || s == "SELECT" || s == "BTN_BACK") return "BTN_BACK";

    // Digital stick directions (if you use them)
    if (s.startsWith("LS_") || s.startsWith("RS_")) return s; // already canonical

    return s; // return uppercased as-is if unknown
  }


  // --- Joy button -> Action cache ---
  struct JoyPair { String btn; Action act; };
  static JoyPair g_joyCache[32];
  static size_t g_joyCacheN = 0;

  static void rebuildJoyActionCache() {
    g_joyCacheN = 0;
    for (size_t i=0; i<JOYMAP_N && g_joyCacheN<32; ++i) {
      const char* fw = JOYMAP_DEFAULTS[i].action;
      String btn = normJoyBtn( joymapPrefs.getString(fw, JOYMAP_DEFAULTS[i].defBtn) );
      if (!btn.length()) continue; // allow unassigned
      Action a = actionFromName(String(fw)); // your existing helper
      if (a != ACT_NONE) g_joyCache[g_joyCacheN++] = { btn, a };
    }
    do { if (1) Serial.printf("[JOYMAP] cache rebuilt: %u entries\n", (unsigned)g_joyCacheN); } while(0);
  }

  static Action actionForJoyButtonCached(const String& btnName) {
    String key = normJoyBtn(btnName);
    for (size_t i=0; i<g_joyCacheN; ++i)
      if (g_joyCache[i].btn.equalsIgnoreCase(key)) return g_joyCache[i].act;
    return ACT_NONE;
  }



//----------------------------------------------------------------------Bluepad32 Functions---------------------------------------------------------

  //#if USE_BLUEPAD32

    void onConnectedController(ControllerPtr ctl) {
        for (int i = 0; i < 
# 3660 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                           4
# 3660 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                            ; i++) {
            if (myControllers[i] == nullptr) {
                myControllers[i] = ctl;
                do { if (1) Serial.printf("Controller connected! Index=%d\n", i); } while(0);
                break;
            }
        }
    }

    void onDisconnectedController(ControllerPtr ctl) {
        for (int i = 0; i < 
# 3670 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                           4
# 3670 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                            ; i++) {
            if (myControllers[i] == ctl) {
                myControllers[i] = nullptr;
                do { if (1) Serial.printf("Controller disconnected! Index=%d\n", i); } while(0);
                break;
            }
        }
    }

    // Call mapped action on button edge using the joy cache
    static inline void joyEdgeDispatch(const char* token, bool now, bool& last) {
      if (now == last) return;
      last = now;
      Action a = actionForJoyButtonCached(String(token)); // token examples: "R1", "L2_CLICK", "A", "X", ...
      if (a != ACT_NONE) {
        dispatchAction(a, now); // 'now' == true on press, false on release
      }
    }


    void handleControllerInput(ControllerPtr ctl) {
      if (!ctl) return;

      // --- RIGHT STICK: DRIVE (arcade) ---
      int rx = ctl->axisRY();
      int ry = ctl->axisRX();
      const int deadzone = 100;

      if (abs(rx) < deadzone) rx = 0;
      if (abs(ry) < deadzone) ry = 0;

      int speed = map(ry, -511, 511, -255, 255); // Forward +, reverse -
      int turn = map(rx, -511, 511, -255, 255); // Left -, right +

      int leftPWM = ((speed + turn)<(-255)?(-255):((speed + turn)>(255)?(255):(speed + turn)));
      int rightPWM = ((speed - turn)<(-255)?(-255):((speed - turn)>(255)?(255):(speed - turn)));

      static int lastLeftPWM = 0, lastRightPWM = 0;
      if (leftPWM != lastLeftPWM || rightPWM != lastRightPWM) {
        if (leftPWM == 0 && rightPWM == 0) {
          rightMotorStop();
          leftMotorStop();
        } else {
          if (leftPWM > 0) leftMotorForward( leftPWM);
          else leftMotorBackward(-leftPWM);
          if (rightPWM > 0) rightMotorForward( rightPWM);
          else rightMotorBackward(-rightPWM);
        }
        lastLeftPWM = leftPWM;
        lastRightPWM = rightPWM;
      }

      // --- LEFT STICK (Y only): ARM (analog) ---
      int ly = -ctl->axisY(); // Up = positive
      int armPWM = (abs(ly) > deadzone) ? map(ly, -511, 511, -255, 255) : 0;

      static int lastArmPWM = 0;
      if (armPWM != lastArmPWM) {
        if (armPWM == 0) {
          armMotorStop();
        } else if (armPWM > 0) {
          armMotorUp( armPWM);
        } else {
          armMotorDown(-armPWM);
        }
        lastArmPWM = armPWM;
      }

      // --- BUTTONS -> mapped actions (IMMEDIATE via joy cache) ---
      // We treat L2 / R2 as "click" digital tokens for mapping purposes.
      static bool pR1=false, pR2=false, pL1=false, pL2=false;
      static bool pA=false, pB=false, pX=false, pY=false;

      bool r1 = ctl->r1();
      bool r2 = ctl->r2();
      bool l1 = ctl->l1();
      bool l2 = ctl->l2();

      bool a = ctl->a();
      bool b = ctl->b();
      bool x = ctl->x();
      bool y = ctl->y();

      // Shoulder / triggers
      joyEdgeDispatch("R1", r1, pR1);
      joyEdgeDispatch("R2_CLICK", r2, pR2);
      joyEdgeDispatch("L1", l1, pL1);
      joyEdgeDispatch("L2_CLICK", l2, pL2);

      // Face buttons
      joyEdgeDispatch("A", a, pA);
      joyEdgeDispatch("B", b, pB);
      joyEdgeDispatch("X", x, pX);
      joyEdgeDispatch("Y", y, pY);

      // --- (Optional) add START/BACK if your controller API exposes them ---
      // Example (uncomment if ctl->start() / ctl->back() exist):
      /*
      static bool pStart=false, pBack=false;
      bool start = ctl->start();
      bool back  = ctl->back();
      joyEdgeDispatch("BTN_START", start, pStart);
      joyEdgeDispatch("BTN_BACK",  back,  pBack);
      */

      // --- (Optional) D-Pad mapping ---
      // If your API gives discrete booleans (e.g. ctl->dpadUp()), wire them like below:
      /*
      static bool pDU=false, pDD=false, pDL=false, pDR=false;
      joyEdgeDispatch("DPAD_UP",    ctl->dpadUp(),    pDU);
      joyEdgeDispatch("DPAD_DOWN",  ctl->dpadDown(),  pDD);
      joyEdgeDispatch("DPAD_LEFT",  ctl->dpadLeft(),  pDL);
      joyEdgeDispatch("DPAD_RIGHT", ctl->dpadRight(), pDR);
      */
      // If you only have an enum-based dpad(), add a small switch that sets four booleans and call joyEdgeDispatch for each.
    }


  //#endif

//-----------------------------------------------------------------------Robot I2S Audio------------------------------------------------------------

  void stopAudio() {
    audio.stopSong();
    digitalWrite(46 /* Optional: amp enable*/, 0x0);
    playbackStarted = false;

    // Release exclusive SD if we own it
    if (g_gateHeldByAudio) {
      g_gateHeldByAudio = false;
      xQueueGenericSend( ( QueueHandle_t ) ( g_sdStreamGate ), 
# 3800 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
     __null
# 3800 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
     , ( ( TickType_t ) 0U ), ( ( BaseType_t ) 0 ) );
    }
  }


    // Call once when enabling mic (after i2s_driver_install)
  void setupI2SMic() {
    static const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = (1<<1) /*|< Accept a Level 1 interrupt vector (lowest priority)*/,
      .dma_buf_count = 4,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
    };
    static const i2s_pin_config_t pin_config = {
      .bck_io_num = 39,
      .ws_io_num = 40,
      .data_out_num = -1,
      .data_in_num = 38
    };

    i2s_driver_install(I2S_NUM_1 /* instead of I2S_NUM_0*/, &i2s_config, 0, 
# 3827 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                                   __null
# 3827 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                                       );
    i2s_set_pin(I2S_NUM_1 /* instead of I2S_NUM_0*/, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_1 /* instead of I2S_NUM_0*/);

    if (!micBuf) {

      micBuf = (int16_t*)ps_malloc(micBufBytes);



    }
    micLastSendMs = millis();
  }

  void enableMic() {
    if (currentI2SMode == I2S_MIC) { do { if (1) Serial.println("[I2S] Mic already enabled."); } while(0); return; }
    do { if (1) Serial.println("[I2S] Switching to MIC: stopping audio, uninstalling I2S, and initializing mic..."); } while(0);
    stopAudio();
    esp_err_t res = i2s_driver_uninstall(I2S_NUM_1 /* instead of I2S_NUM_0*/);
    if (res == 0 /*!< esp_err_t value indicating success (no error) */) do { if (1) Serial.println("[I2S] I2S driver uninstalled successfully (for MIC)."); } while(0);
    else if (res == 0x103 /*!< Invalid state */) do { if (1) Serial.println("[I2S] I2S driver not installed, nothing to uninstall."); } while(0);
    else do { if (1) Serial.printf("[I2S] I2S driver uninstall for MIC returned code: %d\n", res); } while(0);

    setupI2SMic();
    currentI2SMode = I2S_MIC;
    do { if (1) Serial.println("[I2S] MIC enabled and ready."); } while(0);
  }

  void disableMic() {
    if (currentI2SMode == I2S_MIC) {
      do { if (1) Serial.println("[I2S] Disabling MIC: uninstalling I2S driver..."); } while(0);
      esp_err_t res = i2s_driver_uninstall(I2S_NUM_1 /* instead of I2S_NUM_0*/);
      if (res == 0 /*!< esp_err_t value indicating success (no error) */) do { if (1) Serial.println("[I2S] I2S driver uninstalled successfully (MIC off)."); } while(0);
      else do { if (1) Serial.printf("[I2S] I2S driver uninstall for MIC returned code: %d\n", res); } while(0);
      currentI2SMode = I2S_NONE;
    } else {
      do { if (1) Serial.println("[I2S] MIC not enabled, nothing to disable."); } while(0);
    }
    if (micBuf) { free(micBuf); micBuf = nullptr; }
  }

  void streamMicToWebSocket() {
    if (soundIsPlaying()) return;
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
    esp_err_t res = i2s_read(I2S_NUM_1 /* instead of I2S_NUM_0*/,
                            (void*)micBuf,
                            micBufBytes,
                            &bytesRead,
                            10 / ( ( TickType_t ) 1000 / ( 
# 3892 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                1000 
# 3892 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                ) ));

    if (res == 0 /*!< esp_err_t value indicating success (no error) */ && bytesRead > 0) {
      // Send as binary; clients that don't care (UI) ignore non-text frames
      wsCarInput.binaryAll((uint8_t*)micBuf, bytesRead);
      micLastSendMs = now;
    }

    // Cooperative yield
    delay(0);
  }


  void playWavFileOnSpeaker(const String& filename) {
    do { if (1) Serial.printf("[AUDIO] playWavFileOnSpeaker: %s\n", filename.c_str()); } while(0);
    { SdLock lk; if (!SD.exists(filename.c_str())) { do { if (1) Serial.printf("[ERROR] File not found on SD: %s\n", filename.c_str()); } while(0); return; } }

    // If long media is playing, pause it and mark that we’ll need to resume
    if (audio.isRunning() && !isSystemSoundPlaying) {
      pauseAudio(); // you already restart from start; good enough
      g_mediaPausedBySystem = true; // remember to resume afterwards
    }

    // Take exclusive SD gate for this short system sound
    xQueueSemaphoreTake( ( g_sdStreamGate ), ( ( TickType_t ) 0xffffffffUL ) );
    g_gateHeldByAudio = true;

    disableMic();
    stopAudio();
    delay(60);
    enableSpeaker();
    audio.setVolume(sSndVolume); // you already do this in queue path
    audio.connecttoFS(SD, filename.c_str());

    currentlyPlayingFile = filename;
    lastMediaProgressSend = 0;
    delay(30);

    if (!audio.isRunning()) do { if (1) Serial.println("[AUDIO] Playback did not start!"); } while(0);
    else do { if (1) Serial.println("[AUDIO] Playback started OK."); } while(0);
    isSystemSoundPlaying = true;
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
      pinMode(46 /* Optional: amp enable*/, 0x03);
      digitalWrite(46 /* Optional: amp enable*/, 0x1); // Enable NS4168 PA

      if (currentI2SMode == I2S_SPEAKER) {
          do { if (1) Serial.println("[I2S] Speaker already enabled."); } while(0);
          return;
      }
      do { if (1) Serial.println("[I2S] Switching to SPEAKER: stopping audio, uninstalling I2S, and initializing speaker..."); } while(0);

      // Always stop audio regardless of state
      audio.stopSong();

      // Only uninstall if previously MIC or SPEAKER was initialized
      if (currentI2SMode != I2S_NONE) {
          esp_err_t res = i2s_driver_uninstall(I2S_NUM_1 /* instead of I2S_NUM_0*/);
          delay(20); // Allow I2S peripheral to settle
          if (res == 0 /*!< esp_err_t value indicating success (no error) */) {
              do { if (1) Serial.println("[I2S] I2S driver uninstalled successfully (for SPEAKER)."); } while(0);
          } else {
              do { if (1) Serial.printf("[I2S] I2S driver uninstall for SPEAKER returned code: %d\n", res); } while(0);
          }
      }

      // Re-setup speaker I2S pinout and volume every time
      audio.setPinout(10, 45, 9); // (10, 45, 9)
      audio.setVolume(15); // Use your tested value (or 21 for max)
      currentI2SMode = I2S_SPEAKER;
      do { if (1) Serial.println("[I2S] SPEAKER enabled and ready."); } while(0);
  }

  void sendDeviceMediaProgress() {
      // Only send if playing a file
      if (currentlyPlayingFile.length() > 0 && audio.isRunning()) {
          unsigned long now = millis();
          if (now - lastMediaProgressSend > 500) { // Send progress update every 0.5s
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
      do { if (1) Serial.println("[AUDIO] Paused (returns to start on resume)"); } while(0);
    }
    playbackStarted = false;
  }

  void resumeAudio() {
    if (!isPaused || playlist.empty()) return;
    do { if (1) Serial.printf("[AUDIO] Resuming %s from start\n", playlist[currentTrack].c_str()); } while(0);

    // Take exclusive SD for long media playback
    xQueueSemaphoreTake( ( g_sdStreamGate ), ( ( TickType_t ) 0xffffffffUL ) );
    g_gateHeldByAudio = true;

    enableSpeaker();
    audio.connecttoFS(SD, playlist[currentTrack].c_str());
    playbackStarted = true;
    isPaused = false;
  }


  void playCurrentTrack() {
      if (playlist.empty() || currentTrack < 0 || currentTrack >= playlist.size()) {
          do { if (1) Serial.println("[playCurrentTrack] Playlist empty or currentTrack invalid!"); } while(0);
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
    do { if (1) Serial.printf("[VOLUME] %d\n", currentVolume); } while(0);
  }

  void randomTrack() {
    if (playlist.empty()) return;
    currentTrack = random(0, playlist.size());
    do { if (1) Serial.printf("[RANDOM] %s (Volume: %d)\n", playlist[currentTrack].c_str(), currentVolume); } while(0);
    playCurrentTrack();
  }

  void nextFolder() {
    if (folderIndex.size() <= 1) {
      do { if (1) Serial.println("Only one folder in list."); } while(0);
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
    do { if (1) Serial.printf("[NEXTFOLDER] Now playing from folder %s\n", mediaFolders[folder]); } while(0);
    playCurrentTrack();
  }

  void resetESP() {
    do { if (1) Serial.println("Resetting ESP32..."); } while(0);
    delay(200);
    ESP.restart();
  }


//-------------------------------------------------------------------------PSRAM VERSION------------------------------------------------------------

  bool loadPlaylistFromIndex(const char* folder) {
      SdLock lock; // 🔒 protect SD card access

      playlist.clear();
      String indexPath = String(folder) + "/.index";
      File idx = SD.open(indexPath);
      if (!idx) {
          do { if (1) Serial.printf("[ERROR] Can't open index: %s\n", indexPath.c_str()); } while(0);
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


//-------------------------------------------------------------------------Firmware OTA-------------------------------------------------------------

  void otaUpdate(){
      // OTA (unchanged)
    ElegantOTA.begin(&server); // Works with AsyncWebServer
    do { if (1) Serial.println("OTA ready: http://<device_ip>/update"); } while(0);
    ElegantOTA.onEnd([](bool success) {
        do { if (1) Serial.println("ElegantOTA finished, restarting..."); } while(0);
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

//----------------------------------------------------------------------MEDIA CAPTURE FUNCTIONS-----------------------------------------------------

  // Utility: Get timestamp filename


  std::vector<std::vector<uint8_t, psram_allocator<uint8_t>>, psram_allocator<std::vector<uint8_t, psram_allocator<uint8_t>>>> frameBuffer;





  String getMediaTimestamp(const char* prefix, const char* ext) {
      struct timeval tv;
      gettimeofday(&tv, 
# 4235 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                       __null
# 4235 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                           );
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
      SdLock lk; // lock only while opening
      f = SD.open(filePath, "w");
    }
    if (!f) {
      videoRecording = false;
      videoTaskActive = false;
      vTaskDelete(
# 4264 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                 __null
# 4264 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                     );
      return;
    }

    videoRecording = true;
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
          f.write((const uint8_t*)header, hdrLen);
          f.write(fb->buf, fb->len);
          // Optionally flush every few frames if you want extra safety
          // if ((millis() & 0x1F) == 0) f.flush();
        }

        esp_camera_fb_return(fb);
      }

      // Be cooperative; don't block the system
      vTaskDelay(( ( TickType_t ) ( ( ( TickType_t ) ( 90 ) * ( TickType_t ) ( 
# 4298 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                1000 
# 4298 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                ) ) / ( TickType_t ) 1000U ) )); // ~10–11 fps
    }

    {
      SdLock lk; // lock while closing
      f.close();
    }

    videoRecording = false;
    videoTaskActive = false;

    // Safe to play the WAV now: SD is free and we’re outside the handler
    playSystemSound("/web/pcm/vreccomplete.wav");

    vTaskDelete(
# 4312 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
               __null
# 4312 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                   );
  }

//---------------------------------------------------------------DEBUG Helpers for get/set_keymap handlers------------------------------------------





  // Print current stored keymap from Preferences
  static void dumpKeymapToSerial(const char* tag) {
    do { if (1) Serial.printf("[KEYMAP] %s\n", tag ? tag : ""); } while(0);
    for (size_t i = 0; i < KEYMAP_N; ++i) {
      const char* a = KEYMAP_DEFAULTS[i].action;
      String cur = keymapPrefs.getString(a, KEYMAP_DEFAULTS[i].def);
      do { if (1) Serial.printf("  %-16s = '%s'\n", a, cur.c_str()); } while(0);
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


//---------------------------------------------------------------------------ServerStart------------------------------------------------------------

  void serverStart() {
    // Optional: start from a clean Wi-Fi state
    if (WiFi.getMode() != WIFI_MODE_NULL) WiFi.mode(WIFI_MODE_NULL);
    WiFi.persistent(false);

    // ---- Always boot into AP lobby (AP+STA) ----
    startApLobby(); // shows AP IP on OLED, sets wifiState = WIFI_AP_LOBBY
    scheduleApSpeechSoon(700); // optional voice line "AP mode" + IP

    do { if (1) Serial.println("🟠 Starting AP mode for setup / UI..."); } while(0);
    do { if (1) Serial.print("AP SSID: "); } while(0); do { if (1) Serial.println("MINIEXCO_S3_V1_02"); } while(0);
    do { if (1) Serial.print("AP IP:   "); } while(0); do { if (1) Serial.println(WiFi.softAPIP()); } while(0);

    // ---- SD card static file serving setup ----
    server.on("/", HTTP_GET, handleRoot);

    // Favicon
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
      if (SD.exists("/favicon.ico")) request->send(SD, "/favicon.ico", "image/x-icon");
      else request->send(404, "text/plain", "favicon.ico not found on SD card");
    });

    // Wi-Fi pages / endpoints (keep your route names)
    server.on("/setup", HTTP_GET, handleWiFiSetup);
    server.on("/savewifi", HTTP_POST, handleSaveWiFi);
    server.on("/listwifi", HTTP_GET, handleListWiFi);

    // Captive-portal helpers
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){ request->redirect("/setup"); });

    // Try-connect endpoint (kept, useful for manual tests)
    // /wifi_try_connect (purely reports status now)
    server.on("/wifi_try_connect", HTTP_POST, [](AsyncWebServerRequest *request){
      auto *resp = new AsyncJsonResponse();
      JsonObject root = resp->getRoot();
      if (request->hasParam("ssid", true)) {
        String ssid = normalizedSSID(request->getParam("ssid", true)->value());
        bool ok = (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid);
        root["ok"] = ok;
        root["ssid"] = ssid;
        if (ok) { root["ip"] = WiFi.localIP().toString(); resp->setCode(200); }
        else { root["error"] = "not_connected"; resp->setCode(503); }
      } else {
        root["ok"] = false; root["error"] = "missing_ssid";
        resp->setCode(400);
      }
      resp->setLength();
      request->send(resp);
    });

    // /wifi_set_priority (set preferred only; the loop will try it on next scan)
    server.on("/wifi_set_priority", HTTP_POST, [](AsyncWebServerRequest* req){
      if (!req->hasParam("ssid", true)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"ssid required\"}");
        return;
      }
      String ssid = normalizedSSID(req->getParam("ssid", true)->value());
      wifiPrefs.putString("preferred_ssid", ssid);
      req->send(200, "application/json", "{\"ok\":true}");
    });

    // /delete_saved_wifi (no WIFI_LOG_DISC here)
    server.on("/delete_saved_wifi", HTTP_POST, [](AsyncWebServerRequest* req){
      if (!req->hasParam("ssid", true)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"ssid required\"}");
        return;
      }
      String ssid = normalizedSSID(req->getParam("ssid", true)->value());

      wifiPrefs.remove(("wifi_" + ssid).c_str());
      wifiPrefs.remove(("retry_" + ssid).c_str());
      wifiPrefs.remove(("aRt_" + ssid).c_str());

      String list = wifiPrefs.getString("networks", "");
      String newList; int last = 0;
      while (true) {
        int next = list.indexOf(',', last);
        String item = normalizedSSID((next == -1) ? list.substring(last) : list.substring(last, next));
        if (item.length() && item != ssid) {
          if (newList.length()) newList += ",";
          newList += item;
        }
        if (next == -1) break;
        last = next + 1;
      }
      wifiPrefs.putString("networks", newList);

      // If currently connected to that SSID, disconnect once (non-blocking)
      if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid) WiFi.disconnect(true, false);

      req->send(200, "application/json", "{\"ok\":true}");
    });


    // Report AP password state
    server.on("/get_ap_password", HTTP_GET, [](AsyncWebServerRequest* req){
      bool isOpen = (g_apPassword.length() == 0);
      String payload = String("{\"ok\":true,\"open\":") + (isOpen ? "true" : "false") +
                      ",\"min\":8,\"max\":63}";
      req->send(200, "application/json", payload);
    });

    // Set AP password (empty => OPEN AP)
    server.on("/set_ap_password", HTTP_POST, [](AsyncWebServerRequest* req){
      if (!req->hasParam("ap_pass", true)) {
        return req->send(400, "application/json", "{\"ok\":false,\"err\":\"missing\"}");
      }
      String pw = req->getParam("ap_pass", true)->value();
      if (!setApPassword(pw)) {
        return req->send(400, "application/json", "{\"ok\":false,\"err\":\"len\"}");
      }
      req->send(200, "application/json", "{\"ok\":true}");
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
        root[k] = v; // normalized ("arrowup", " ")
      }

      // Log the JSON we are about to send
      String out; serializeJson(root, out);
      do { if (1) Serial.printf("[/get_keymap] response: %s\n", out.c_str()); } while(0);

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
          do { if (1) Serial.printf("[/set_keymap] payload: %s\n", in.c_str()); } while(0);

          // Log all fields we received and flag unknowns
          for (JsonPair kv : obj) {
            String k = kv.key().c_str();
            String v = kv.value().as<String>();
            do { if (1) Serial.printf("    recv field '%s' = '%s'%s\n", k.c_str(), v.c_str(), isKnownActionField(k) ? "" : "  (UNKNOWN)"); } while(0)
                                                                               ;
          }

          size_t updated = 0;

          // Accept both firmware snake_case and UI camelCase aliases
          for (size_t i = 0; i < ACTION_ALIASES_N; ++i) {
            const char* fw = ACTION_ALIASES[i].fw;
            const char* alias = ACTION_ALIASES[i].alias;

            String raw; const char* used = nullptr;
            if (obj.containsKey(fw)) { raw = obj[fw].as<String>(); used = fw; }
            else if (alias && obj.containsKey(alias)) { raw = obj[alias].as<String>(); used = alias; }
            else continue;

            String norm = normKeyToken(raw); // "ArrowUp"->"arrowup", "Space"->" ", "E"->"e"
            do { if (1) Serial.printf("    apply %-16s from '%s': '%s' -> '%s'\n", fw, used, raw.c_str(), norm.c_str()); } while(0)
                                                          ;

            keymapPrefs.putString(fw, norm);
            ++updated;
          }

          if (updated == 0) {
            do { if (1) Serial.printf("[/set_keymap] WARNING: no fields matched known actions (updated=0)\n"); } while(0);
          }

          // Dump what is now stored after writes
          dumpKeymapToSerial("after set_keymap");

          root["status"] = "ok";
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
          auto* out = new AsyncJsonResponse();
          JsonObject root = out->getRoot().to<JsonObject>();

          if (!json.is<JsonObject>()) {
            root["ok"] = false; root["error"]="object json expected";
            out->setCode(400); out->setLength(); request->send(out); return;
          }

          JsonObject obj = json.as<JsonObject>();
          size_t updated = 0;

          for (JsonPair kv : obj) {
            String fw = aliasToFw(String(kv.key().c_str())); // accept UI alias or fw key
            // Only accept known action keys from JOYMAP_DEFAULTS
            bool known = false;
            for (size_t i=0;i<JOYMAP_N;++i)
              if (fw.equalsIgnoreCase(JOYMAP_DEFAULTS[i].action)) { known = true; break; }
            if (!known) continue;

            String btn = normJoyBtn(kv.value().as<String>());
            joymapPrefs.putString(fw.c_str(), btn);
            ++updated;
            do { if (1) Serial.printf("[set_joymap] %-16s <- '%s'\n", fw.c_str(), btn.c_str()); } while(0);
          }

          rebuildJoyActionCache(); // <-- take effect immediately

          root["ok"] = true;
          root["updated"] = (int)updated;
          out->setCode(200); out->setLength(); request->send(out);
        });

      h->setMethod(HTTP_POST);
      h->setMaxContentLength(1024);
      server.addHandler(h);
    }

    // POST /reset_joymap
    server.on("/reset_joymap", HTTP_POST, [](AsyncWebServerRequest* request){
      joymapResetDefaults();
      rebuildJoyActionCache(); // <-- keep runtime in sync
      auto* ok = new AsyncJsonResponse(); ok->getRoot()["ok"]=true;
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
          if (!Update.begin(0xFFFFFFFF)) {
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
      resp->getRoot()["current"] = "v2.0.78";
      resp->setCode(200);
      resp->setLength();
      request->send(resp);
    });

    // ---------- /list_saved_wifi (GET) ----------
    // Returns: [ { ssid, password, retry, autoReconnect, preferred }, ... ]
    server.on("/list_saved_wifi", HTTP_GET, [](AsyncWebServerRequest *request){
      auto *resp = new AsyncJsonResponse();
      JsonArray arr = resp->getRoot().to<JsonArray>();

      const String pref = wifiPrefs.getString("preferred_ssid", "");
      bool prefExistsInList = false;

      String savedList = wifiPrefs.getString("networks", "");
      int last = 0;
      while (true) {
        int next = savedList.indexOf(',', last);
        String ssid = (next == -1) ? savedList.substring(last) : savedList.substring(last, next);
        ssid = normalizedSSID(ssid);

        if (ssid.length()) {
          String wifiKey = "wifi_" + ssid;
          String pass = wifiPrefs.getString(wifiKey.c_str(), "");
          int retry = wifiPrefs.getInt(("retry_" + ssid).c_str(), 5);
          bool autoRec = wifiPrefs.getBool(("aRt_" + ssid).c_str(), false);

          JsonObject it = arr.createNestedObject();
          it["ssid"] = ssid;
          it["password"] = pass;
          it["retry"] = retry;
          it["autoReconnect"] = autoRec;

          bool isPreferred = (ssid == pref);
          it["preferred"] = isPreferred; // <-- NEW

          if (isPreferred) prefExistsInList = true;
        }

        if (next == -1) break;
        last = next + 1;
      }

      // If a preferred SSID is set but no longer saved, clear the preference
      if (pref.length() && !prefExistsInList) {
        wifiPrefs.remove("preferred_ssid");
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

      do { if (1) Serial.println("🔄 Switching to saved Wi-Fi: " + ssid); } while(0);
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

          // SAVE: persist ints & bools to camPrefs.
          // (OK to omit camPrefs.begin/end here if you've already begun("camera") in setup())
          for (JsonPair kv : obj) {
            const char* key = kv.key().c_str();
            JsonVariant v = kv.value();
            // normalize any type to int-ish for booleans too
            if (strcmp(key, "auto_res") == 0) {
              camPrefs.putBool("auto_res", v.as<int>() != 0);
            } else if (strcmp(key, "adaptive_q") == 0) {
              camPrefs.putBool("adaptive_q", v.as<int>() != 0);
            } else if (v.is<int>()) {
              camPrefs.putInt(key, v.as<int>());
            }
            // (add putString if you ever send strings)
          }

          // APPLY: use stream-safe update for res/quality; direct setters for others
          sensor_t *s = esp_camera_sensor_get();
          if (s) {
            auto getI = [&](const char* k, int def=0){ return obj.containsKey(k) ? obj[k].as<int>() : def; };

            if (obj.containsKey("res") || obj.containsKey("quality")) {
              framesize_t fs = (framesize_t)(obj.containsKey("res") ? obj["res"].as<int>() : s->status.framesize);
              int q = obj.containsKey("quality") ? obj["quality"].as<int>() : s->status.quality;
              requestCameraParams(fs, q); // <-- stream-safe apply
            }

            // DO NOT set framesize/quality directly here (avoid double-applying)
            if (obj.containsKey("contrast")) s->set_contrast(s, getI("contrast"));
            if (obj.containsKey("brightness")) s->set_brightness(s, getI("brightness"));
            if (obj.containsKey("saturation")) s->set_saturation(s, getI("saturation"));
            if (obj.containsKey("gray")) s->set_special_effect(s, getI("gray") ? 2 : 0);
            if (obj.containsKey("hmirror")) s->set_hmirror(s, getI("hmirror"));
            if (obj.containsKey("vflip")) s->set_vflip(s, getI("vflip"));
            if (obj.containsKey("awb")) s->set_whitebal(s, getI("awb"));
            if (obj.containsKey("wb_mode")) s->set_wb_mode(s, getI("wb_mode"));
            if (obj.containsKey("aec")) s->set_exposure_ctrl(s, getI("aec"));
            if (obj.containsKey("ae_level")) s->set_ae_level(s, getI("ae_level"));
            if (obj.containsKey("aec_value")) s->set_aec_value(s, getI("aec_value"));
            if (obj.containsKey("agc")) s->set_gain_ctrl(s, getI("agc"));
            if (obj.containsKey("agc_gain")) s->set_agc_gain(s, getI("agc_gain"));
            if (obj.containsKey("gainceiling")) s->set_gainceiling(s, (gainceiling_t)getI("gainceiling"));
            if (obj.containsKey("awb_gain")) s->set_awb_gain(s, getI("awb_gain"));
            if (obj.containsKey("colorbar")) s->set_colorbar(s, getI("colorbar"));
            if (obj.containsKey("lenc")) s->set_lenc(s, getI("lenc"));
            if (obj.containsKey("bpc")) s->set_bpc(s, getI("bpc"));
            if (obj.containsKey("wpc")) s->set_wpc(s, getI("wpc"));
            if (obj.containsKey("dcw")) s->set_dcw(s, getI("dcw"));
            if (obj.containsKey("raw_gma")) s->set_raw_gma(s, getI("raw_gma"));
            if (obj.containsKey("special_effect")) s->set_special_effect(s, getI("special_effect"));
          }

          // Set LIVE flags immediately (so toggles take effect now), then kick
          if (obj.containsKey("auto_res")) allowFsAuto = obj["auto_res"].as<int>() != 0;
          if (obj.containsKey("adaptive_q")) adaptiveQEnabled = obj["adaptive_q"].as<int>() != 0;
          adaptiveKickNow();

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
          default: modelName = "Unknown"; break;
        }
        root["model"] = modelName;
        root["res"] = (int)s->status.framesize;
        root["quality"] = (int)s->status.quality;
        root["contrast"] = (int)s->status.contrast;
        root["brightness"] = (int)s->status.brightness;
        root["saturation"] = (int)s->status.saturation;
        root["gray"] = (int)(s->status.special_effect == 2 ? 1 : 0);
        root["hmirror"] = (int)s->status.hmirror;
        root["vflip"] = (int)s->status.vflip;
        root["awb"] = (int)s->status.awb;
        root["wb_mode"] = (int)s->status.wb_mode;
        root["aec"] = (int)s->status.aec;
        root["ae_level"] = (int)s->status.ae_level;
        root["aec_value"] = (int)s->status.aec_value;
        root["agc"] = (int)s->status.agc;
        root["agc_gain"] = (int)s->status.agc_gain;
        root["gainceiling"] = (int)s->status.gainceiling;
        root["awb_gain"] = (int)s->status.awb_gain;
        root["colorbar"] = (int)s->status.colorbar;
        root["lenc"] = (int)s->status.lenc;
        root["bpc"] = (int)s->status.bpc;
        root["wpc"] = (int)s->status.wpc;
        root["dcw"] = (int)s->status.dcw;
        root["raw_gma"] = (int)s->status.raw_gma;
        root["special_effect"] = (int)s->status.special_effect;
      } else {
        root["model"] = modelName;
      }

      // UI / app flags (assumes these globals exist)
      root["darkMode"] = (int)(darkMode ? 1 : 0);
      root["holdBucket"] = (int)(holdBucket ? 1 : 0);
      root["holdAux"] = (int)(holdAux ? 1 : 0);
      root["horizontalScreen"] = (int)(horScreen ? 1 : 0);
      root["RecordTelemetry"] = (int)(tlmEnabled ? 1 : 0);
      root["SystemSounds"] = (int)(sSndEnabled ? 1 : 0);
      root["BluepadEnabled"] = (int)(bluepadEnabled ? 1 : 0);
      root["SystemVolume"] = (int)sSndVolume;

      // NEW: controller toggles from prefs
      root["auto_res"] = (int)camPrefs.getBool("auto_res", 0); // default OFF
      root["adaptive_q"] = (int)camPrefs.getBool("adaptive_q", 1); // default ON

      String payload;
      serializeJson(doc, payload);

      AsyncWebServerResponse *resp = request->beginResponse(200, "application/json", payload);
      resp->addHeader("Cache-Control", "no-store");
      request->send(resp);
    });


    // ---------- /list_sd_files (GET -> JSON, chunked) ----------
    server.on("/list_sd_files", HTTP_GET, [](AsyncWebServerRequest *request) {
      do { if (1) Serial.println("📁 /list_sd_files requested (INDEX)"); } while(0);

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
          do { if (1) Serial.println("[Auto-clear] pendingReindex reset by file presence"); } while(0);
        }
      }

      // -- If folder doesn't exist: return []
      {
        SdLock lk;
        if (!SD.exists(path)) {
          auto *resp = new AsyncJsonResponse(false); // array
          resp->getRoot().to<JsonArray>(); // []
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
          auto *resp = new AsyncJsonResponse(false); // array
          resp->getRoot().to<JsonArray>(); // []
          resp->setCode(200);
          resp->setLength();
          request->send(resp);
          return;
        }

        if (!pendingReindex) {
          reindexPath = path;
          pendingReindex = true;
          reindexCount = 0;
        }

        auto *resp = new AsyncJsonResponse(); // object
        resp->getRoot()["status"] = "reindexing";
        resp->setCode(202);
        resp->setLength();
        request->send(resp);
        return;
      }

      // -- Index exists: read a page
      auto *resp = new AsyncJsonResponse(false); // array
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
      if (request->hasParam("path", true)) path = urlDecode(request->getParam("path", true)->value());
      else if (request->hasParam("path", false)) path = urlDecode(request->getParam("path", false)->value());
      if (!path.startsWith("/")) path = "/" + path;
      while (path.indexOf("//") >= 0) path.replace("//","/");
      if (path.endsWith("/") && path.length() > 1) path.remove(path.length()-1);

      AsyncJsonResponse *resp = new AsyncJsonResponse();
      JsonObject root = resp->getRoot();

      // 404 if path doesn't exist
      {
        SdLock lock;
        if (!SD.exists(path)) {
          root["ok"] = false;
          root["error"] = "not_found";
          root["path"] = path;
          resp->setCode(404);
          resp->setLength();
          request->send(resp);
          return;
        }
      }

      if (pendingReindex) {
        root["ok"] = false;
        root["error"] = "already_in_progress";
        root["path"] = path;
        resp->setCode(409);
        resp->setLength();
        request->send(resp);
        return;
      }

      // kick off background reindex (your loop/task should consume these)
      reindexPath = path;
      pendingReindex = true;
      reindexCount = 0; // how many files processed so far
      reindexTotal = 0; // optional: fill later when you count
      reindexCounting = true; // optional flag if you use it

      root["ok"] = true;
      root["status"] = "started";
      root["path"] = path;
      resp->setCode(202);
      resp->setLength(); // ← required
      request->send(resp);
    });

    // ---------- /sd_reindex_status (GET -> JSON) ----------
    server.on("/sd_reindex_status", HTTP_GET, [](AsyncWebServerRequest *request){
      AsyncJsonResponse *resp = new AsyncJsonResponse();
      JsonObject root = resp->getRoot();

      root["pending"] = pendingReindex;
      root["path"] = String(reindexPath);
      root["count"] = reindexCount;
      root["total"] = reindexTotal;
      root["counting"] = reindexCounting;

      resp->setCode(200);
      resp->setLength(); // ← required
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
          resp->getRoot()["path"] = path;
          resp->setCode(404);
          resp->setLength();
          request->send(resp);
          return;
        }
      }
      sendFileFromSDWithMime(request, path, mimeFor(path), /*asAttachment=*/true); // attachment
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
      String pathFile = recycleFile + ".path";
      String dst = "/" + name;

      if (SD.exists(pathFile)) {
        File f = SD.open(pathFile, "r");
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

      if (request->hasParam("path")) originalPath = urlDecode(request->getParam("path")->value());
      else if (request->hasArg("path")) originalPath = urlDecode(request->arg("path"));
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
        String ext = (dot >= 0) ? filename.substring(dot) : "";
        testRecyclePath = "/recycle/" + base + "_" + String(count++) + ext;
      }
      recyclePath = testRecyclePath;

      // Save metadata
      String pathMetaFile = recyclePath + ".path";
      { File meta = SD.open(pathMetaFile, "w"); if (meta) { meta.print(originalPath); meta.close(); } }

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
          resp->getRoot()["ok"] = false;
          resp->getRoot()["error"] = ctx ? ctx->error : "unknown";
          resp->setCode(500);
        } else {
          resp->getRoot()["ok"] = true;
          resp->getRoot()["path"] = ctx->uploadPath;
          resp->getRoot()["bytes"] = (int)ctx->bytesWritten;
          if (ctx->verifySha) {
            resp->getRoot()["sha256"] = toHexLower(ctx->digest, sizeof(ctx->digest));
            resp->getRoot()["sha_ok"] = true;
          }
          resp->setCode(200);
        }
        resp->setLength();
        request->send(resp);

        if (ctx && ctx->error.length() == 0) playSystemSound("/web/pcm/click.wav");
        else playSystemSound("/web/pcm/error.wav");

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
          if (request->hasParam("path", true)) uploadPath = urlDecode(request->getParam("path", true)->value());
          else if (request->hasParam("path", false)) uploadPath = urlDecode(request->getParam("path", false)->value());
          else uploadPath = "/" + filename;
          if (!uploadPath.startsWith("/")) uploadPath = "/" + uploadPath;
          ctx->uploadPath = uploadPath;

          // Optional sha256 (query/body)
          String shaHex;
          if (request->hasParam("sha256", true)) shaHex = request->getParam("sha256", true)->value();
          else if (request->hasParam("sha256", false)) shaHex = request->getParam("sha256", false)->value();
          shaHex = normHex64(shaHex);
          if (shaHex.length() == 64) {
            ctx->verifySha = true;
            ctx->expectedShaHex = shaHex;
            mbedtls_sha256_init(&ctx->sha);
            mbedtls_sha256_starts_ret(&ctx->sha, 0 /* 0 = SHA-256, 1 = SHA-224 */);
          }

          do { if (1) Serial.printf(">>> Starting upload: %s (verify=%d)\n", ctx->uploadPath.c_str(), ctx->verifySha); } while(0);

          // Ensure directory exists for final & temp
          { SdLock lock; ensureFolderExists(ctx->uploadPath); }

          // Build temp path in same directory, hidden-ish, unique
          String dir = dirName(ctx->uploadPath);
          String name = baseName(ctx->uploadPath);
          { SdLock lock;
            String tmpWant = dir + "/." + name + ".upload.tmp";
            ctx->tmpPath = SD.exists(tmpWant) ? uniqueInDir(dir, "." + name + ".upload.tmp") : tmpWant;
          }

          // Open temp for write. Do NOT touch the final yet.
          { SdLock lock; ctx->uploadFile = SD.open(ctx->tmpPath, "w"); }
          if (!ctx->uploadFile) { ctx->error = "open_tmp_failed"; return; }
        }

        // Stream chunk -> temp, with periodic flush + tiny yield, and update SHA if enabled
        // Robust, throttled streaming write (chunked + retries + reopen)
        if (ctx && ctx->uploadFile && len && ctx->error.length() == 0) {
          const size_t CHUNK = 4096; // write granularity
          const int RETRIES = 4; // attempts per write before escalate
          const size_t FLUSH_EVERY = 8192; // periodic flush/yield

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
                sdTinyYield(3 + attempts * 2); // backoff 3ms,5ms,7ms,9ms
              }
              attempts++;
            }

            // Last resort: reopen file and try again
            if (wrote == 0) {
              { SdLock lock; ctx->uploadFile.close(); }
              sdTinyYield(10);
              { SdLock lock; ctx->uploadFile = SD.open(ctx->tmpPath, "a"); }
              if (!ctx->uploadFile) {
                ctx->error = "sd_reopen_failed";
                do { if (1) Serial.printf("!!! SD reopen failed at index=%u\n", (unsigned)index); } while(0);
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
              do { if (1) Serial.printf("!!! SD write error at index=%u\n", (unsigned)index); } while(0);
              break;
            }

            // Hash exactly what we wrote
            if (ctx->verifySha) mbedtls_sha256_update_ret(&ctx->sha, data + off, wrote);

            off += wrote;
            ctx->bytesWritten += wrote;
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
          sdTinyYield(5); // final breather before sha/rename

          // Finalize SHA and verify (if requested)
          if (ctx && ctx->verifySha && ctx->error.length() == 0) {
            mbedtls_sha256_finish_ret(&ctx->sha, ctx->digest);
            mbedtls_sha256_free(&ctx->sha);
            String got = toHexLower(ctx->digest, sizeof(ctx->digest));
            if (got != ctx->expectedShaHex) {
              ctx->error = "sha256_mismatch";
              do { if (1) Serial.printf("!!! SHA256 mismatch: got=%s expected=%s\n", got.c_str(), ctx->expectedShaHex.c_str()); } while(0)
                                                                 ;
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
                do { if (1) Serial.printf("!!! Failed to move old file to recycle: %s\n", recPath.c_str()); } while(0);
              } else {
                do { if (1) Serial.printf("Moved old file to recycle: %s\n", recPath.c_str()); } while(0);
              }
            }

            // 2) Promote temp -> final if OK
            if (ctx->error.length() == 0) {
              if (!SD.rename(ctx->tmpPath, ctx->uploadPath)) {
                ctx->error = "rename_tmp_to_final_failed";
                do { if (1) Serial.printf("!!! Failed to promote temp to final: %s -> %s\n", ctx->tmpPath.c_str(), ctx->uploadPath.c_str()); } while(0)
                                                                        ;
              } else {
                do { if (1) Serial.printf("<<< Upload finished. Promoted %s -> %s (bytes=%u)\n", ctx->tmpPath.c_str(), ctx->uploadPath.c_str(), (unsigned)ctx->bytesWritten); } while(0)
                                                                                                     ;

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
        File file = SD.open(path, "w");
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
      auto updatePref = [&](const char* key, int value){ if (camPrefs.getInt(key, 
# 5658 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                                                                 (-0x7fffffff - 1)
# 5658 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                                                                          )!=value){ camPrefs.putInt(key,value); changed=true; } };

      if (request->hasParam("res")) { int v=request->getParam("res")->value().toInt(); if (camPrefs.getInt("camRes",FRAMESIZE_QVGA)!=v) camPrefs.putInt("camRes", v); }
      if (request->hasParam("fps")) { int v=request->getParam("fps")->value().toInt(); updatePref("camFps", v); s->set_quality(s, v); }
      if (request->hasParam("rot")) { int v=request->getParam("rot")->value().toInt(); updatePref("camRot", v); s->set_hmirror(s,(v==1||v==3)); s->set_vflip(s,(v==2||v==3)); }
      if (request->hasParam("sat")) { int v=request->getParam("sat")->value().toInt(); updatePref("camSat", v); s->set_saturation(s, v); }
      if (request->hasParam("gray")) { int v=request->getParam("gray")->value().toInt(); updatePref("camGray", v); s->set_special_effect(s, v?2:0); }
      if (request->hasParam("led")) { int v=request->getParam("led")->value().toInt(); updatePref("camLed", v); }
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

      // strip query (?v=123) before filesystem lookup
      int q = path.indexOf('?');
      if (q >= 0) path = path.substring(0, q);

      // 1) /telemetry/* served as CSV straight from SD
      if (path.startsWith("/telemetry/")) {
        SdLock lock;
        if (SD.exists(path)) {
          sendFileFromSDWithMime(request, path, "text/csv");
        } else {
          do { if (1) Serial.printf("[HTTP] 404 /telemetry %s\n", path.c_str()); } while(0);
          request->send(404, "text/plain", "File Not Found");
        }
        return;
      }

      // 2) /media/* served with correct content-type
      if (path.startsWith("/media/")) {
        sendFileFromSD(request, path);
        return;
      }

      // 3) everything else from /web (this covers /img/*.png, /modal.css, /modalScript.js, etc.)
      sendStaticFromWeb(request, path);
    });

    // ---------- /list_media_files (GET -> JSON, chunked) ----------
    server.on("/list_media_files", HTTP_GET, [](AsyncWebServerRequest *request){
      do { if (1) Serial.printf(">> Handler start: %s, heap=%u\n", __func__, ESP.getFreeHeap()); } while(0);

      // Pagination
      const int start = request->hasParam("start") ? request->getParam("start")->value().toInt() : 0;
      int count = request->hasParam("count") ? request->getParam("count")->value().toInt() : 40;
      if (count < 1 || count > 256) count = 40;

      // If any folder needs indexing -> trigger and return light JSON
      for (const auto &folder : mediaFolders) { // folder is const char*
        String idxPath = String(folder) + "/.index";
        idxPath.replace("//","/");
        bool hasIndex = false;
        { SdLock lk; hasIndex = SD.exists(idxPath); }
        if (!hasIndex) {
          { SdLock lk; if (!pendingReindex) { reindexPath = folder; pendingReindex = true; reindexCount = 0; } }
          auto *resp = new AsyncJsonResponse();
          auto root = resp->getRoot().to<JsonObject>();
          root["status"] = "reindexing";
          root["path"] = folder; // const char*
          resp->setCode(200);
          resp->setLength();
          request->send(resp);
          do { if (1) Serial.printf("<< Handler end (reindexing %s): heap=%u\n", folder, ESP.getFreeHeap()); } while(0);
          return;
        }
      }

      auto *resp = new AsyncJsonResponse(); // object root
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

        SdLock lk; // hold lock while reading index
        File idx = SD.open(idxPath, "r");
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

          if (((++tick) & 0x7F) == 0) delay(0); // cooperative yield
        }
        idx.close();
        // still under SdLock scope; it will release here
        delay(0); // tiny gap between folders
      }

      root["start"] = start;
      root["count"] = emitted;
      root["total"] = total;

      resp->setCode(200);
      resp->setLength(); // avoid chunked
      request->send(resp);

      do { if (1) Serial.printf("<< Handler end: %s, heap=%u\n", __func__, ESP.getFreeHeap()); } while(0);
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

      do { if (1) Serial.printf("[MEDIA] Switching to speaker, playing: %s\n", file.c_str()); } while(0);
      playWavFileOnSpeaker(file);

      root["ok"]=true; root["status"]="playing"; root["file"]=file;
      resp->setCode(200);
      resp->setLength();
      request->send(resp);
    });

    // ---------- /stop_playback (GET -> text) ----------
    server.on("/stop_playback", HTTP_GET, [](AsyncWebServerRequest *request){
      do { if (1) Serial.println("[AUDIO] stop_playback called"); } while(0);
      stopAudio(); // your function that stops speaker I2S
      request->send(200, "text/plain", "OK");
    });

    // ---------- /enable_mic (GET -> text) ----------
    server.on("/enable_mic", HTTP_GET, [](AsyncWebServerRequest *request){
      do { if (1) Serial.println("[MIC] enable_mic called"); } while(0);
      enableMic(); // your function
      request->send(200, "text/plain", "OK");
    });

    // ---------- /disable_mic (GET -> text) ----------
    server.on("/disable_mic", HTTP_GET, [](AsyncWebServerRequest *request){
      do { if (1) Serial.println("[MIC] disable_mic called (flagging stream stop)"); } while(0);
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
          int16_t audioBuffer[1024];
          esp_err_t res = i2s_read(I2S_NUM_1 /* instead of I2S_NUM_0*/, (void*)audioBuffer, sizeof(audioBuffer),
                                  &bytesRead, 30 / ( ( TickType_t ) 1000 / ( 
# 5886 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino" 3 4
                                                  1000 
# 5886 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\MiniExco_v_2_00_78.ino"
                                                  ) ));
          if (res != 0 /*!< esp_err_t value indicating success (no error) */ || bytesRead == 0) {
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
          String layout = obj.containsKey("layout") ? obj["layout"].as<String>() : "default";
          bool showIP = obj.containsKey("showIP") ? obj["showIP"].as<bool>() : true;
          bool showBattery = obj.containsKey("showBattery") ? obj["showBattery"].as<bool>() : true;
          bool showWiFi = obj.containsKey("showWiFi") ? obj["showWiFi"].as<bool>() : true;

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
          do { if (1) Serial.printf("📤 Uploading OLED animation file: %s\n", filename.c_str()); } while(0);
          SdLock lock;
          uploadFile = SD.open("/oled_anim.xbm", "w");
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
        err->getRoot()["status"] = "error";
        err->getRoot()["message"] = "Camera capture failed";
        err->setCode(500);
        err->setLength();
        request->send(err);
        return;
      }

      String filePath;
      bool okWrite = true;

      // 2) SD write (guarded, chunked, with yields)
      {
        SdLock lock; // 🔒 Only hold SD lock while actually touching SD
        String folder = "/media/capture/photo";
        if (!SD.exists(folder)) SD.mkdir(folder);

        // If your getMediaTimestamp already returns a full path, keep as-is:
        filePath = getMediaTimestamp("photo", "jpg"); // e.g. "/media/capture/photo/2025-08-13_12-34-56.jpg"
        File f = SD.open(filePath, "w");
        if (!f) {
          okWrite = false;
        } else {
          const uint8_t* p = fb->buf;
          size_t rem = fb->len;
          while (rem && okWrite) {
            size_t n = rem > 4096 ? 4096 : rem;
            size_t w = f.write(p, n);
            if (w != n) {
              okWrite = false;
              f.close();
              SD.remove(filePath); // delete partial file
              break;
            }
            p += w;
            rem -= w;
            vTaskDelay(1); // give time to lwIP/WiFi/etc.
          }
          if (okWrite) f.close();
        }
      } // 🔓 SD lock released here

      // 3) Return frame buffer to camera driver ASAP
      esp_camera_fb_return(fb);

      // 4) Respond
      if (!okWrite) {
        auto *err = new AsyncJsonResponse();
        err->getRoot()["status"] = "error";
        err->getRoot()["message"] = "SD write/open failed";
        err->setCode(500);
        err->setLength();
        request->send(err);
        return;
      }

      auto *resp = new AsyncJsonResponse();
      resp->getRoot()["status"] = "ok";
      resp->getRoot()["path"] = filePath;
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
        root["status"] = "error";
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
        if (duration < 1) duration = 1;
        if (duration > 60) duration = 60;
      }

      // Play sound BEFORE starting the task (keeps SD contention low)
      playSystemSound("/web/pcm/videorecord.wav");

      // Create task
      int* arg = new int(duration);
      BaseType_t ok =



          xTaskCreatePinnedToCore(recordVideoTask, "RecordVideo", 12288, arg, 1, &videoTaskHandle, 1);


      if (ok != ( ( ( BaseType_t ) 1 ) )) {
        delete arg;
        root["status"] = "error";
        root["message"] = "task_create_failed";
        resp->setCode(500);
        resp->setLength();
        request->send(resp);
        return;
      }

      // Pre-arm the guard so a 2nd start can’t slip in before the task sets flags
      videoTaskActive = true;

      root["status"] = "recording";
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
            f = SD.open(path, "r");
        }

        if (!f) {
            auto *err = new AsyncJsonResponse();
            err->getRoot()["error"] = "not_found";
            err->getRoot()["path"] = path;
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
                head = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ";
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
      do { if (1) Serial.println("[MQTT] /get_mqtt requested."); } while(0);
      loadMqttConfig();

      auto *resp = new AsyncJsonResponse(); // object
      JsonObject root = resp->getRoot().to<JsonObject>();
      root["enable"] = mqttCfg.enable;
      root["host"] = mqttCfg.host;
      root["port"] = mqttCfg.port;
      root["user"] = mqttCfg.user;
      root["pass"] = mqttCfg.pass;
      root["topic_prefix"] = mqttCfg.topic_prefix;

      resp->setCode(200);
      resp->setLength(); // ✅ avoid chunked
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
          cfg.enable = obj.containsKey("enable") ? obj["enable"].as<bool>() : false;
          cfg.host = obj.containsKey("host") ? obj["host"].as<String>() : "";
          cfg.port = obj.containsKey("port") ? obj["port"].as<uint16_t>() : 1883;
          cfg.user = obj.containsKey("user") ? obj["user"].as<String>() : "";
          cfg.pass = obj.containsKey("pass") ? obj["pass"].as<String>() : "";
          cfg.topic_prefix = obj.containsKey("topic_prefix") ? obj["topic_prefix"].as<String>() : "";

          saveMqttConfig(cfg);
          mqttNeedsReconnect = true;

          do { if (1) Serial.printf("[MQTT] Saved: enable=%d host='%s' port=%d user='%s' topic_prefix='%s'\n", cfg.enable, cfg.host.c_str(), cfg.port, cfg.user.c_str(), cfg.topic_prefix.c_str()); } while(0)
                                                                                               ;

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
      do { if (1) Serial.println("[MQTT] /mqtt_test requested."); } while(0);
      loadMqttConfig();

      // Quick validation
      if (mqttCfg.host.isEmpty() || mqttCfg.port == 0) {
        request->send(200, "text/plain", "❌ Invalid host/port");
        return;
      }

      // Keep this short & cooperative
      WiFiClient wifiClient;

        wifiClient.setTimeout(2000); // read timeout safeguard


      PubSubClient testClient(wifiClient);
      testClient.setServer(mqttCfg.host.c_str(), mqttCfg.port);
      testClient.setSocketTimeout(2); // seconds
      String clientId = "MiniExcoTest-" + String((uint32_t)millis());

      bool connected = false;
      // Try connect (keep it snappy)
      yield();
      if (mqttCfg.user.length() > 0) {
        do { if (1) Serial.printf("[MQTT] Connecting (user='%s')\n", mqttCfg.user.c_str()); } while(0);
        connected = testClient.connect(clientId.c_str(),
                                      mqttCfg.user.c_str(),
                                      mqttCfg.pass.c_str());
      } else {
        do { if (1) Serial.println("[MQTT] Connecting anonymously."); } while(0);
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
      root["connected"] = mqttConnected;
      if (!mqttConnected && mqttLastError.length()) {
        root["last_error"] = mqttLastError;
      }
      resp->setCode(200);
      resp->setLength(); // ✅
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

    // ---------- WS + server start ----------
    wsCarInput.onEvent(onCarInputWebSocketEvent);
    server.addHandler(&wsCarInput);
    server.begin();
    do { if (1) Serial.println("HTTP server started"); } while(0);
  }

//----------------------------------------------------------------------------Telemetry-------------------------------------------------------------

  // ----------- Battery, Charger, WiFi, Temp, Uptime Telemetry -----------
  void sendBatteryTelemetryIfIdle() {
      static unsigned long lastTelemetrySend = 0;
      static float filteredBatteryVoltage = 0;
      static float filteredChargerVoltage = 0;
      const float filterAlpha = 0.15; // 0.05–0.3, lower = smoother, slower

      if (!audio.isRunning() && millis() - lastTelemetrySend > 1000) {
          lastTelemetrySend = millis();

          int adcRaw = analogRead(13);
          int adcRawCharging = analogRead(1);

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
          int batteryPercent = (((int)(((filteredBatteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0))<(0)?(0):(((int)(((filteredBatteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0))>(100)?(100):((int)(((filteredBatteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0))));

          int rssi = WiFi.RSSI();
          int wifiQuality = ((2 * (rssi + 100))<(0)?(0):((2 * (rssi + 100))>(100)?(100):(2 * (rssi + 100))));

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
          currentSample.voltage = filteredBatteryVoltage;
          currentSample.charger = filteredChargerVoltage;
          currentSample.wifi = wifiQuality;
          currentSample.temp = chipTemp;

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
    if (!imuPresent) return;
    if (millis() - lastImuSend > 500) {
      lastImuSend = millis();
      imu::Vector<3> euler = bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> mag = bno055.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      float temp = bno055.getTemp();
      String imuMsg = "IMU," + String(euler.x(), 1) + "," + String(euler.y(), 1) + "," + String(euler.z(), 1)
                    + "," + String(mag.x(), 1) + "," + String(mag.y(), 1) + "," + String(mag.z(), 1)
                    + "," + String(temp, 1);
      wsCarInput.textAll(imuMsg);

      // ---- Fill currentSample for MQTT! ----
      currentSample.imu_euler_x = euler.x();
      currentSample.imu_euler_y = euler.y();
      currentSample.imu_euler_z = euler.z();
      currentSample.imu_mag_x = mag.x();
      currentSample.imu_mag_y = mag.y();
      currentSample.imu_mag_z = mag.z();
      currentSample.imu_temp = temp;
    }
  }

  // ----------- FPS Telemetry -----------
  void sendFpsTelemetry() {
    static unsigned long lastFpsSend = 0;
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

//----------------------------------------------------------------------Serial Commands and debug---------------------------------------------------

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
          do { if (1) Serial.printf("[SERIAL] Requested play: '%s'\n", filename.c_str()); } while(0);
          if (!SD.exists(filename.c_str())) {
            do { if (1) Serial.printf("[ERROR] File not found on SD: '%s'\n", filename.c_str()); } while(0);
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
          do { if (1) Serial.printf("✅ Command: %c %d\n", cmd, value); } while(0);
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
              do { if (1) Serial.println("[SERIAL] Triggering webServerReboot by command..."); } while(0);
              webServerReboot();
          }
          else if (cmd == "wifi" || cmd == "debug") {
            printWifiDebugInfo();
          }
          else if (cmd == "previous") prevTrack();
          else if (cmd == "play") playCurrentTrack();
          else if (cmd == "stop") {
            stopAudio();
            do { if (1) Serial.println("[STOP]"); } while(0);
          }
          else if (cmd == "random") randomTrack();
          else if (cmd == "nextfolder") nextFolder();
          else if (cmd == "reset" || cmd == "reboot") resetESP();
          else if (cmd == "+") setVolume(currentVolume + 1);
          else if (cmd == "-") setVolume(currentVolume - 1);
          else if (cmd == "list") {
            do { if (1) Serial.println("Current playlist:"); } while(0);
            for (size_t i = 0; i < playlist.size(); ++i) {
              do { if (1) Serial.printf("[%d] %s\n", i, playlist[i].c_str()); } while(0);
            }
          } else {
            do { if (1) Serial.printf("[UNKNOWN COMMAND] '%s'\n", input.c_str()); } while(0);
          }
        }
      }
    }
  }

  void printDebugInfo() {
    do { if (1) Serial.println(((reinterpret_cast<const __FlashStringHelper *>(("==== ESP32 Debug Info ===="))))); } while(0);
    do { if (1) Serial.printf("Free heap:      %u bytes\n", ESP.getFreeHeap()); } while(0);
    do { if (1) Serial.printf("Min free heap:  %u bytes\n", ESP.getMinFreeHeap()); } while(0);
    do { if (1) Serial.printf("Heap size:      %u bytes\n", ESP.getHeapSize()); } while(0);
    do { if (1) Serial.printf("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap()); } while(0);
    do { if (1) Serial.printf("Uptime:         %lu ms\n", millis()); } while(0);
    do { if (1) Serial.printf("Sketch size:    %u bytes\n", ESP.getSketchSize()); } while(0);
    do { if (1) Serial.printf("Flash chip size:%u bytes\n", ESP.getFlashChipSize()); } while(0);

      do { if (1) Serial.printf("ESP-IDF v%d.%d.%d\n", 4, 4, 7); } while(0);

      do { if (1) Serial.println("=========================="); } while(0);
  }

  void printWifiDebugInfo() {
    do { if (1) Serial.println("------ Debug Info ------"); } while(0);

    // General heap/PSRAM status
    do { if (1) Serial.printf("Free Heap:   %u bytes\n", ESP.getFreeHeap()); } while(0);

      do { if (1) Serial.printf("Free PSRAM:  %u bytes\n", ESP.getFreePsram()); } while(0);


    // Wi-Fi stack status
    wl_status_t status = WiFi.status();
    const char* wifiStatusStr =
      (status == WL_CONNECTED) ? "CONNECTED" :
      (status == WL_NO_SSID_AVAIL) ? "NO_SSID_AVAIL" :
      (status == WL_IDLE_STATUS) ? "IDLE_STATUS" :
      (status == WL_SCAN_COMPLETED) ? "SCAN_COMPLETED" :
      (status == WL_CONNECT_FAILED) ? "CONNECT_FAILED" :
      (status == WL_CONNECTION_LOST)? "CONNECTION_LOST" :
      (status == WL_DISCONNECTED) ? "DISCONNECTED" : "UNKNOWN";
    do { if (1) Serial.printf("WiFi.status(): %d (%s)\n", status, wifiStatusStr); } while(0);

    // Detailed Wi-Fi info
    do { if (1) Serial.printf("IP Address:   %s\n", WiFi.localIP().toString().c_str()); } while(0);
    do { if (1) Serial.printf("Gateway:      %s\n", WiFi.gatewayIP().toString().c_str()); } while(0);
    do { if (1) Serial.printf("Subnet Mask:  %s\n", WiFi.subnetMask().toString().c_str()); } while(0);
    do { if (1) Serial.printf("DNS 1:        %s\n", WiFi.dnsIP(0).toString().c_str()); } while(0);
    do { if (1) Serial.printf("DNS 2:        %s\n", WiFi.dnsIP(1).toString().c_str()); } while(0);
    do { if (1) Serial.printf("RSSI:         %d dBm\n", WiFi.RSSI()); } while(0);
    do { if (1) Serial.printf("Hostname:     %s\n", WiFi.getHostname()); } while(0);

    // Print AP details if connected
    if (status == WL_CONNECTED) {
      do { if (1) Serial.printf("SSID:         %s\n", WiFi.SSID().c_str()); } while(0);
      do { if (1) Serial.printf("BSSID:        %s\n", WiFi.BSSIDstr().c_str()); } while(0);
      do { if (1) Serial.printf("Channel:      %d\n", WiFi.channel()); } while(0);
      // No PHY mode on ESP32S3, skip
    }

    // WebSocket status
    do { if (1) Serial.printf("WebSocket clients: %u\n", wsCarInput.count()); } while(0);
    for (auto& c : wsCarInput.getClients()) {
        do { if (1) Serial.printf("  WS Client #%u, IP: %s, status: %s\n", c.id(), c.remoteIP().toString().c_str(), c.status() ? "Connected" : "Not Connected"); } while(0)
                                                                                              ;
    }


    do { if (1) Serial.println("------------------------"); } while(0);
  }


//------------------------------------------------------------------------------MQTT----------------------------------------------------------------

  String getDeviceBlock() {
    String ipStr = WiFi.localIP().toString();
    return
      "\"device\":{"
        "\"identifiers\":[\"" "MINIEXCO_S3_V1_02" "\"],"
        "\"connections\":[[\"ip\",\"" + ipStr + "\"]],"
        "\"manufacturer\":\"https://github.com/elik745i/miniexco.v1\","
        "\"model\":\"MiniExco S3\","
        "\"name\":\"MiniExco Robot\","
        "\"sw_version\":\"" "v2.0.78" "\""
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_battery\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_battvolt\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_chvolt\","
        + getDeviceBlock() +
      "}").c_str(), true);

    // ---- Charging status ----
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "charging/config").c_str(),
      ("{"
        "\"name\":\"Charging Status\","
        "\"state_topic\":\"" + prefix + "charging\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_charging\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_wifi\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_chiptemp\","
        + getDeviceBlock() +
      "}").c_str(), true);

    // ---- FPS ----
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "fps/config").c_str(),
      ("{"
        "\"name\":\"Camera FPS\","
        "\"state_topic\":\"" + prefix + "fps\","
        "\"unit_of_measurement\":\"fps\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_fps\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_uptime\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_roll\","
        + getDeviceBlock() +
      "}").c_str(), true);
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "imu_pitch/config").c_str(),
      ("{"
        "\"name\":\"IMU Pitch\","
        "\"state_topic\":\"" + prefix + "imu_pitch\","
        "\"unit_of_measurement\":\"°\","
        "\"icon\":\"mdi:axis-y-rotate-clockwise\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_pitch\","
        + getDeviceBlock() +
      "}").c_str(), true);
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "imu_yaw/config").c_str(),
      ("{"
        "\"name\":\"IMU Yaw\","
        "\"state_topic\":\"" + prefix + "imu_yaw\","
        "\"unit_of_measurement\":\"°\","
        "\"icon\":\"mdi:axis-z-rotate-clockwise\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_yaw\","
        + getDeviceBlock() +
      "}").c_str(), true);

    // ---- IMU: Magnetometer ----
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "imu_mag_x/config").c_str(),
      ("{"
        "\"name\":\"IMU Mag X\","
        "\"state_topic\":\"" + prefix + "imu_mag_x\","
        "\"icon\":\"mdi:magnet\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_mag_x\","
        + getDeviceBlock() +
      "}").c_str(), true);
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "imu_mag_y/config").c_str(),
      ("{"
        "\"name\":\"IMU Mag Y\","
        "\"state_topic\":\"" + prefix + "imu_mag_y\","
        "\"icon\":\"mdi:magnet\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_mag_y\","
        + getDeviceBlock() +
      "}").c_str(), true);
    mqtt.publish(
      ("homeassistant/sensor/" + prefix + "imu_mag_z/config").c_str(),
      ("{"
        "\"name\":\"IMU Mag Z\","
        "\"state_topic\":\"" + prefix + "imu_mag_z\","
        "\"icon\":\"mdi:magnet\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imu_mag_z\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_imutemp\","
        + getDeviceBlock() +
      "}").c_str(), true);

    // ---- Light Control (Switch) ----
    mqtt.publish(
      ("homeassistant/switch/" + prefix + "light/config").c_str(),
      ("{"
        "\"name\":\"Robot Light\","
        "\"command_topic\":\"" + prefix + "light/set\","
        "\"state_topic\":\"" + prefix + "light\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_light\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_beacon\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_emergency\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_armup\","
        "\"icon\":\"mdi:arrow-up-bold-box\","
        + getDeviceBlock() +
      "}").c_str(), true);
    mqtt.publish(
      ("homeassistant/button/" + prefix + "arm_down/config").c_str(),
      ("{"
        "\"name\":\"Arm Down\","
        "\"command_topic\":\"" + prefix + "arm_down/set\","
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_armdown\","
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
          "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_" + dirs[i] + "\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_buckettilt\","
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
        "\"unique_id\":\"" "MINIEXCO_S3_V1_02" "_auxtilt\","
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
      moveCar(1);
      mqtt.publish((prefix + "move_state").c_str(), "FORWARD", true);
    }
    else if (tpc == prefix + "backward/set") {
      moveCar(2);
      mqtt.publish((prefix + "move_state").c_str(), "BACKWARD", true);
    }
    else if (tpc == prefix + "left/set") {
      moveCar(3);
      mqtt.publish((prefix + "move_state").c_str(), "LEFT", true);
    }
    else if (tpc == prefix + "right/set") {
      moveCar(4);
      mqtt.publish((prefix + "move_state").c_str(), "RIGHT", true);
    }
    else if (tpc == prefix + "stop/set") {
      moveCar(0);
      mqtt.publish((prefix + "move_state").c_str(), "STOP", true);
    }

    // ---- Bucket Tilt ----
    else if (tpc == prefix + "bucket_tilt/set") {
      int angle = msg.toInt();
      angle = ((angle)<(0)?(0):((angle)>(180)?(180):(angle)));
      bucketTilt(angle);
      mqtt.publish((prefix + "bucket_tilt").c_str(), String(angle).c_str(), true);
    }

    // ---- AUX Tilt ----
    else if (tpc == prefix + "aux_tilt/set") {
      int angle = msg.toInt();
      angle = ((angle)<(0)?(0):((angle)>(180)?(180):(angle)));
      auxControl(angle);
      mqtt.publish((prefix + "aux_tilt").c_str(), String(angle).c_str(), true);
    }

    // ---- (Optional: add more control topics as needed) ----
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
      result = mqtt.connect("MINIEXCO_S3_V1_02", mqttCfg.user.c_str(), mqttCfg.pass.c_str());
    } else {
      result = mqtt.connect("MINIEXCO_S3_V1_02");
    }

    if (result) {
      do { if (1) Serial.println("✅ MQTT connected."); } while(0);
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
      do { if (1) Serial.print("❌ MQTT connect failed: "); } while(0);
      do { if (1) Serial.println(mqtt.state()); } while(0);
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


//-----------------------------------------------------------------------------SETUP--------------------------------------------------------------//


  void setup() {
    initSdMutex();


      Serial.begin(115200);
      Serial.setDebugOutput(true);


    setUpPinModes();
    initHostnames();

    // ---- Preferences ----
    wifiPrefs.begin("wifi", false);
    camPrefs.begin("camsettings", false);
    keymapPrefs.begin("keymap", false);
    joymapPrefs.begin("joymap", false);
    imuPrefs.begin("telemetry", false);
    uiPrefs.begin("ui", false);
    oledPrefs.begin("oled", false);
    camEnablePrefs.begin("enabled", false);

    // AP password sanity
    g_apPassword = uiPrefs.getString("ap_pass", "12345678" /* 8..63 chars; change to your default*/);
    if (g_apPassword.length() && (g_apPassword.length() < 8 || g_apPassword.length() > 63)) {
      g_apPassword = "12345678" /* 8..63 chars; change to your default*/;
    }

    // UI prefs (leave as you had)
    darkMode = uiPrefs.getBool("darkMode", false);
    horScreen = uiPrefs.getBool("Switch", false);
    holdBucket = uiPrefs.getBool("HoldBucket", false);
    holdAux = uiPrefs.getBool("HoldAux", false);
    tlmEnabled = uiPrefs.getBool("RecordTelemetry", false);
    sSndEnabled = uiPrefs.getBool("SystemSounds", true);
    sSndVolume = uiPrefs.getInt("SystemVolume", 15);
    bluepadEnabled = uiPrefs.getBool("BluepadEnabled", false);

    initInputMapsIfEmpty();
    rebuildJoyActionCache();

    i2cStart();
    spiStart();
    initSdGate();
    auditPcmAssets();
    pixelStart();

    // ---- Web / Wi-Fi ----
    WiFi.persistent(false);
    serverStart(); // ← boots AP lobby; no blocking STA attempts here

    do { if (1) Serial.printf("After serverStart: heap=%u\n", ESP.getFreeHeap()); } while(0);

    // ---- mDNS / NTP / OTA / camera, etc. ----
    bool mdnsOk = startMdns();
    if (mdnsOk) {
      do { if (1) Serial.printf("[mDNS] %s.local @ %s\n", g_mdnsHost.c_str(), WiFi.localIP().toString().c_str()); } while(0);
    } else {
      do { if (1) Serial.printf("[mDNS] FAILED for %s\n", g_mdnsHost.c_str()); } while(0);
    }
    startNtpSync();
    otaUpdate();

    loadCameraPrefs();
    if (cameraEnabled) enableCamera();

    initBNO055();

    if (loadPlaylistFromIndex(mediaFolders[0])) {
      do { if (1) Serial.printf("[PLAYLIST] Loaded playlist from %s\n", mediaFolders[0]); } while(0);
    } else {
      do { if (1) Serial.printf("[PLAYLIST] Failed to load playlist from %s\n", mediaFolders[0]); } while(0);
    }

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableNewBluetoothConnections(true);
    do { if (1) Serial.printf("After Bluepad32: heap=%u\n", ESP.getFreeHeap()); } while(0);

    mqtt.setBufferSize(512);
    mqttBegin();

    do { if (1) Serial.printf("<<< End of setup: heap=%u, PSRAM=%u\n", ESP.getFreeHeap(), ESP.getFreePsram()); } while(0);
    lastWebActivityMs = millis(); // watchdog timing for server disconnects
  }


//------------------------------------------------------------------------------LOOP--------------------------------------------------------------//

  void loop() {
    const uint32_t now = millis();

    // Audio
    audio.loop();
    pumpSystemSoundScheduler(now);


      handleSerialCommands();


    // Telemetry & logging
    sendBatteryTelemetryIfIdle();
    sendImuTelemetry();
    sendFpsTelemetry();
    if (tlmEnabled) flushTelemetryBufferToSD_Auto();

    // WS housekeeping
    wsCarInput.cleanupClients();

    // Animations (your existing)
    handleAnimationTimers();

    // ---- The only Wi-Fi brain you need ----
    handleWifiSimple();

    // Some lightweight UI flair (optional)
    static unsigned long lastGearFrame = 0;
    if (WiFi.status() == WL_CONNECTED) {
      if (now - lastGearFrame > 200) { animateGears(); lastGearFrame = now; }
    } else {
      static unsigned long lastApFrame = 0;
      if ((WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA) && (now - lastApFrame > 200)) {
        animateAP(); lastApFrame = now;
      }
    }

    // If you keep this helper, it will speak STA IP once on connect
    static bool spokeStaIpThisConnect = false;
    if (WiFi.status() == WL_CONNECTED) {
      if (!spokeStaIpThisConnect) {
        speakStaIpOrDefer(WiFi.localIP());
        spokeStaIpThisConnect = true;
      }
    } else {
      spokeStaIpThisConnect = false;
    }

    // Remainder of your app
    processReindexTask();
    pumpTimeSyncTick();
    streamMicToWebSocket();
    sendDeviceMediaProgress();

    if (bluepadEnabled) {
      BP32.update();
      for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) handleControllerInput(ctl);
      }
    }

    // MQTT reconnect / discovery / loop
    if (mqttCfg.enable) {
      static unsigned long lastAttempt = 0;
      if (!mqtt.connected() && WiFi.status() == WL_CONNECTED) {
        if (now - lastAttempt > 5000) { lastAttempt = now; mqttBegin(); mqttDiscoveryPublished = false; }
      }
      if (mqtt.connected() && !mqttDiscoveryPublished) { publishMqttDiscovery(); mqttDiscoveryPublished = true; }
      if (mqtt.connected()) { publishPeriodicTelemetry(); mqtt.loop(); }
    }


      // --- Watchdog block (use the same 'now') ---
      static uint32_t _lastWdTick = 0;
      if (now - _lastWdTick >= 250) {
        _lastWdTick = now;

        if (now >= WS_REBOOT_GRACE_MS) {
          if (hadAnyClientSinceBoot && wsActiveClients == 0) {
            uint32_t idleSince = (lastWsDisconnectMs > lastWebActivityMs)
                                  ? lastWsDisconnectMs : lastWebActivityMs;
            if (now - idleSince >= WS_REBOOT_TIMEOUT_MS) {
              resetESP();
            }
          }
        }
      }


    // Camera maintenance
    runAdaptiveCamera();

    delay(1);
  }
