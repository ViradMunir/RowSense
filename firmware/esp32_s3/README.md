# ESP32-S3 Firmware

Real-time motion controller for RowSense. Bridges UART commands from the
Raspberry Pi 5 to PWM output, and exposes a BLE interface for manual
control during commissioning and emergency override.

**Status:** the Arduino sketch is being recovered and will be added here.

## What it does

- Reads newline-terminated commands from the Pi over UART @ 115200 baud
- Generates one PWM channel for the ESC (throttle) and one for the
  steering servo
- Accepts BLE commands from the companion mobile app for manual driving

## Serial protocol

Fully specified in the main [README](../../README.md#pi--esp32-serial-protocol).
In short:

| Line | Meaning |
|---|---|
| `AUTO` | Enter autonomous mode |
| `MANUAL` | Ignore incoming velocity commands |
| `CMD_VEL <linear.x> <angular.z>` | Velocity setpoint, sent at 50 Hz while in AUTO |

The Pi-side counterpart is `navigation/auto_manual_mode_switch.py`.

## Build

Arduino IDE with ESP32 board support. Target: ESP32-S3.