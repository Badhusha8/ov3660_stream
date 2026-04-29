# ESP32-S3 OV3660 Live Camera Stream

Live MJPEG video streaming over Wi-Fi using ESP32-S3 DevKit N16R8 and OV3660 camera module, built with ESP-IDF.

## Hardware
- ESP32-S3 DevKit N16R8 (16MB Flash, 8MB PSRAM)
- OV3660 Camera Module

## Features
- Live MJPEG stream at `http://<device-ip>/`
- JPEG snapshot at `http://<device-ip>/snap`
- Auto Wi-Fi reconnect
- OV3660 sensor tuning (AWB, AEC, lens correction)
- 4-frame PSRAM pipeline for smooth streaming

## Setup
1. Clone the repo
2. Copy credentials template:
   cp main/wifi_config.h.example main/wifi_config.h
3. Edit wifi_config.h with your Wi-Fi credentials
4. Build and flash:
   idf.py set-target esp32s3
   idf.py build
   idf.py -p /dev/ttyACM0 flash monitor

## Pin Map (OV3660 → ESP32-S3)
| Signal | GPIO |
|--------|------|
| PWDN   | 38   |
| XCLK   | 15   |
| SIOD   | 4    |
| SIOC   | 5    |
| VSYNC  | 6    |
| HREF   | 7    |
| PCLK   | 13   |
| D0-D7  | 11,9,8,10,12,18,17,16 |

## Branch Structure
- `main` — stable working code
- `dev`  — active development

