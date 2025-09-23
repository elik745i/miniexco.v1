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
