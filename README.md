MiniExco Robot Firmware

Firmware for MiniExco Rover, running on ESP32-S3-SPK v1.0 (8 MB PSRAM / 16 MB flash).
Designed for autonomous robotics and media playback with integrated camera, audio, storage, and LED control.

🚀 Demo Preview

(Add screenshots, GIFs, or YouTube links here to showcase your robot in action)

🎥 Live MJPEG camera stream in web UI

🕹️ Web-based joystick + keyboard/gamepad control

📊 Real-time telemetry overlay (IMU, temperature, battery)

🔊 Media playback & online radio

🌈 WS2812B LED effects and signals

📱 Home Assistant integration

Example layout:

Web Interface	Camera Feed	Telemetry Overlay

	
	

👉 If you record a video demo, link it here (e.g. YouTube).

⚠️ Critical Hardware Requirement: Cooling

This project drives the ESP32-S3 at very high load (Wi-Fi + camera + SD + audio).

You must install both:

A metal heatsink on the ESP32-S3 chip (with thermal pad/adhesive)

A 25 × 25 mm (or similar) low-profile 5 V cooling fan

🚨 Without both passive and active cooling you risk:

Overheating, random resets, Wi-Fi/network failures

Severe instability or permanent damage

👉 ESP32-S3 has no built-in thermal protection.
Do not run without heatsink + fan.

Hardware Features

ESP32-S3-SPK v1.0 board

Dual microphones + DAC to mono speaker

microSD card slot for media and logging

OV2640 camera (MJPEG streaming, capture, video recording)

WS2812B addressable LEDs

Wi-Fi AP/STA modes with preference logic

Bluepad32 Bluetooth gamepad support (optional)

Circuit Diagrams & Tutorials

Circuit diagrams and wiring: [add link here]

Webserver files: [add link here]

Video tutorials: [add link here]

Getting Started
1. Clone the repository
git clone https://github.com/elik745i/miniexco.v1.git
cd miniexco.v1

2. Install toolchain

You can build using Arduino IDE or PlatformIO.

Arduino IDE

Install Arduino IDE

Add ESP32 board manager URL:

File → Preferences → Additional Board URLs →

https://espressif.github.io/arduino-esp32/package_esp32_index.json


Install ESP32 Board Package (2.0.14 or later recommended).

Select ESP32-S3-DevKitC-1 / ESP32-S3-SPK with PSRAM enabled.

Open MiniExco_v2_xx.ino and build/upload.

PlatformIO (VS Code)

Install VS Code
 + PlatformIO
.

Open this repo in VS Code.

Select the project environment (esp32-s3-spk).

Build → Upload firmware.

3. Connect and test

Open Serial Monitor at 115200 baud to see debug logs.

After boot, the ESP32 will:

Start in Wi-Fi AP mode if no saved networks are found.

Broadcast an SSID like MiniExco_xx.

Access the web interface at:

http://192.168.4.1              (AP mode default)
http://<device-ip>              (when connected to your router)
http://miniexco-s3-v1-02.local/ (via mDNS, no IP required 🎉)


(Tip: .local addresses work out of the box on macOS/Linux. On Windows, install Bonjour Print Services
 if mDNS isn’t available.)

4. Web Interface

The built-in frontend allows you to:

Control motors, servos, and LEDs

Adjust camera settings (resolution, FPS, stream quality)

View telemetry (IMU, temperature, battery)

Upload media files (audio, video) to the SD card

Configure Wi-Fi, OTA updates, and preferences

Home Assistant Integration

MiniExco Rover integrates directly into Home Assistant using the MJPEG IP Camera platform.

Add this to configuration.yaml:

camera:
  - platform: mjpeg
    name: MiniExco Rover
    mjpeg_url: http://miniexco-s3-v1-01.local:81/stream
    still_image_url: http://miniexco-s3-v1-01.local/capture


Replace miniexco-s3-v1-01.local with your device’s hostname or IP.

Restart Home Assistant → camera feed appears as an entity.

Build Options

Feature switches (can be overridden with -D in build flags):

USE_BLUEPAD32

0 → exclude BT gamepad support (saves flash/RAM)

1 → include Bluepad32 (requires BT + Bluepad32 library)

DEBUG_SERIAL

0 → silent (disable most DBG_PRINT calls)

1 → verbose serial logging

USE_PSRAM

0 → use internal heap only

1 → prefer PSRAM for large buffers (camera frames, audio, JSON, etc.)

Revision History

v2.0.78

Added 3D models into repo

Reworked Wi-Fi connection logic

v2.0.77

Improved AP/STA switching

Debugged Wi-Fi down and watchdog resets

v2.0.76

Fixed NTP blocking loop

Added IP speech output

Idle animation for AP mode

Code fine-tuning

v2.0.75

Implemented Wi-Fi priority handling for multiple saved networks

v2.0.74 → v2.0.56

See full changelog in source

TODO

Add BMP180 barometric pressure sensor readings in UI

Android APK + API integration

Path-following algorithms

Fix large file uploads

OLED battery icon artifacts

3D model overlay synced with IMU (tilt/turn, flashing lights, animation)

References

Bluepad32 documentation:

https://bluepad32.readthedocs.io/en/latest/

https://github.com/ricardoquesada/bluepad32
