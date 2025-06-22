# MINIEXCO v1 🪛🤖

A compact tabletop robo-excavator powered by dual ESP32 modules. Designed for fun, research, and robotics education — this project combines live streaming, sensor data, remote control, and path planning in a small footprint.

![miniexco](assets/miniexco_banner.jpg) <!-- Add an image later -->

---

## 🔧 Features

- **3x N20 Gear Motors** (100 RPM each):
  - 2 for tank-style drive (left/right track-style movement)
  - 1 for arm movement
- **2x MG90S Servos** for claw and bucket control
- **Tank-style movement** via 2 DRV8833 motor drivers
- **2S Battery Pack** recycled from used vape batteries
- **Wireless charging support** (auto-docking planned in future firmware)
- **ESP32-S3 Board** handles:
  - Web frontend interface (joystick, sliders, OTA updates)
  - Motor control (PWM, servo)
  - UDP/WebSocket communication
- **ESP32-CAM Board** handles:
  - Camera streaming
  - 9-axis IMU (BNO055)
  - OLED (124x64) screen for status, IP, and animations
- **Micro power switch** for manual ON/OFF
- **Fancy OLED animations and IP display**

---

## 📦 Hardware Requirements

| Component           | Quantity | Notes |
|---------------------|----------|-------|
| ESP32-S3 Dev Board  | 1        | Controls logic + serves UI |
| ESP32-CAM Board     | 1        | Camera + sensors |
| N20 Gear Motors (100 RPM) | 3 | 2 for drive, 1 for arm |
| MG90S Servos        | 2        | Claw/Bucket |
| DRV8833 Motor Driver| 2        | One per motor pair |
| 2S Li-Ion Battery Pack | 1    | Recycled cells from vapes work |
| Wireless Charging Module | 1  | Optional auto-docking |
| 124x64 OLED Display | 1        | I2C |
| BNO055 IMU Module   | 1        | Orientation, heading |
| Micro Power Switch  | 1        | Manual ON/OFF |

---

## 🌐 Features in Software

- Wi-Fi Manager with SSID/password storage
- OTA firmware update via browser modal
- Dynamic joystick and keyboard controls
- Camera IP auto-sync with video overlay
- Real-time telemetry: orientation, heading, temperature
- LED, emergency, and beacon control
- Path drawing overlay for future navigation
- Modular script loading to fit ESP32 memory constraints

---

## 🔄 Future Plans

- Auto-navigation back to wireless charging dock
- BLE control and MQTT integration
- Object detection + pathfinding
- Battery monitoring and smart power-saving

---

## 🧠 Getting Started

1. Flash `ESP32-S3` with [s3_firmware.ino](firmware/s3_firmware.ino)
2. Flash `ESP32-CAM` with [cam_firmware.ino](firmware/cam_firmware.ino)
3. Power on — OLED will show the current IP address
4. Connect to robot via browser and control using UI or keyboard
5. Check telemetry and camera stream from the same interface

---

## 🔁 OTA Update Support

Firmware update flow:
- The robot checks a `version.txt` file hosted in this repo
- If a newer version is found, it prompts for OTA update
- Downloaded from GitHub Releases

➡️ See: [`OTA_UPDATE.md`](OTA_UPDATE.md) for setup instructions

---

## 🧑‍💻 Author

Elnur Mehdiyev  
🇹🇷 🇦🇿 Engineer, maker, and robotics enthusiast  
📫 [LinkedIn](https://linkedin.com) | [miniexco.v1 GitHub](https://github.com/elik745i/miniexco.v1)

---

## 📜 License

MIT — free to use, modify, and share.

