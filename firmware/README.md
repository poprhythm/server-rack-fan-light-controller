# Firmware

ESP32 firmware for the Server Rack Fan & Light Controller.

## Features

- Temperature-controlled fan speed (30-100%) using TMP36 sensor
- Dual fan control with tachometer feedback
- Door sensor monitoring with LED indicators
- MQTT integration for remote monitoring
- Temperature sensor validation with failsafe mode
- WiFi connectivity

## Quick Start

1. **Configure credentials:**
   ```bash
   cp secrets.h.example secrets.h
   # Edit secrets.h with your WiFi and MQTT broker details
   ```

2. **Upload to ESP32:**
   - Open `fan-light-controller.ino` in Arduino IDE
   - Select board: "ESP32 Dev Module"
   - Click Upload

3. **Monitor via Serial:**
   - Open Serial Monitor at 115200 baud
   - View temperature, fan speeds, and door status

4. **Calibrate temperature (if needed):**
   - ESP32 ADC can be inaccurate (±10-15°C)
   - Compare reading to known accurate thermometer
   - Adjust `TEMP_CALIBRATION_OFFSET` in firmware (line 49)
   - Default offset: 11.0°C (compensates for typical ADC error)

## MQTT Topics

All topics are published with retain flag:

- `fan_controller/temperature` - Temperature in °C
- `fan_controller/front_fan/rpm` - Front fan RPM
- `fan_controller/back_fan/rpm` - Back fan RPM
- `fan_controller/fan_speed_percent` - Fan speed 30-100%
- `fan_controller/front_door/open` - Front door status (true/false)
- `fan_controller/back_door/open` - Back door status (true/false)
- `fan_controller/sensor_status` - Sensor health (ok/failed)
- `fan_controller/status` - Device status (online/offline)

