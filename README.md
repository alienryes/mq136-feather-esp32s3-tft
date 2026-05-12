# MQ-136 H2S Sensor — Adafruit ESP32-S3 TFT Feather

CircuitPython firmware for the MQ-136 H2S sensor on the Adafruit ESP32-S3 TFT Feather.

Port of the [RP2350 version](https://github.com/alienryes/mq136-feather-rp2350).

## Key differences from RP2350 version

- Native WiFi — no AirLift co-processor required
- Colour 240x135 IPS TFT (ST7789) replaces external SH1107 OLED
- No physical calibration button — handled via Home Assistant MQTT command

## Hardware

- Adafruit ESP32-S3 TFT Feather 4MB Flash / 2MB PSRAM
- MQ-136 gas sensor (AOUT to A0)
