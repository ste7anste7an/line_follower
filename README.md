# I²C Line-Following IR Sensor

**Firmware version:** `2.3`  
**I²C address:** `0x33`  
**Sensors:** `8` IR detectors  
**Frame length:** `13 bytes` (`NUM_SENSORS + 5`)

---

## Overview

This module is an **8-channel line-following IR sensor** running as an **I²C slave**.  
Key features:

- Raw and calibrated sensor output modes  
- Line position (0–255) and moving derivative  
- Line-shape detection (straight, T, Y, L-left, L-right, none)  
- EEPROM-backed calibration (save/load)  
- Neopixel visual feedback (one pixel per sensor + status pixel)  
- Optional emitter control (for Pololu QTR-style sensors)

Communication is performed by writing a single command (and optional params) to the device at address `0x33`, then reading the 13-byte frame.

---

## Returned Data Frame (I²C read)

Each read returns a **13-byte** buffer:

| Byte index | Contents |
|------------|----------|
| `0` - `7`  | Sensor values (raw or normalized, one per sensor) |
| `8`        | Line position (0..255) — valid in calibrated mode |
| `9`        | Min value (active range) |
| `10`       | Max value (active range) |
| `11`       | Derivative (moving-average of position change, 0..255) |
| `12`       | Line shape classification (see table below) |

### Line shape encoding

| Code | Shape |
|------|-------|
| `0`  | None |
| `1`  | Straight |
| `2`  | T-junction |
| `3`  | L-left |
| `4`  | L-right |
| `5`  | Y-junction |

---

## Operating Modes

| ID | Mode name | Description |
|----|-----------|-------------|
| `0` | `MODE_RAW` | Raw (inverted) sensor values (0..255) |
| `1` | `MODE_CAL` | Calibrated (normalized) values with position/derivative/shape |
| `2` | `MODE_DIG` | Not used in current firmware |
| `3` | `MODE_CALIBRATING` | Auto-calibration running (calibration sweep) |

---

## Hardware pin connections for CH32

| GPIO | function |
|------|----------|
| PB6   | SDC      |
| PB7   | SDA    |
| PB3 | CTRL pin for emitter control | 
| PB11  | NeoPixels |
| PA9   | UART TX |
| PA10 | UART RX |
| PA0   | ADC0  |
| PA1   | ADC1  |
| PA2   | ADC2  |
| PA3   | ADC3  |
| PA4   | ADC4  |
| PA5   | ADC5  |
| PA6   | ADC6  |
| PA7   | ADC7  |
| PB0   | ADC8  |
| PB1   | ADC9  |



## I²C Commands

Write the command byte first. Some commands require additional bytes (parameters).

| Cmd ID | Name | Parameters | Description |
|--------|------|------------|-------------|
| `0` | `CMD_SET_MODE_RAW` | none | Switch to raw output |
| `1` | `CMD_SET_MODE_CAL` | none | Switch to calibrated output |
| `2` | `CMD_GET_VERSION` | none | Returns version bytes in the read frame (`maj`, `min`, ...) |
| `3` | `CMD_DEBUG` | unused | Debug-level command is commented out in firmware |
| `4` | `CMD_CALIBRATE` | none | Start auto calibration (5 s) — enters `MODE_CALIBRATING` |
| `5` | `CMD_IS_CALIBRATED` | none | Returns `1` or `0` (first byte of frame) |
| `6` | `CMD_LOAD_CAL` | none | Load calibration arrays from EEPROM |
| `7` | `CMD_SAVE_CAL` | none | Save current calibration to EEPROM |
| `8` | `CMD_GET_MIN` | none | Read calibration `min[]` array (first bytes of frame) |
| `9` | `CMD_GET_MAX` | none | Read calibration `max[]` array (first bytes of frame) |
| `10` | `CMD_SET_MIN` | 8 bytes | Write `min[]` calibration values (8 bytes) |
| `11` | `CMD_SET_MAX` | 8 bytes | Write `max[]` calibration values (8 bytes) |
| `12` | `CMD_NEOPIXEL` | `led, r, g, b` | Set one NeoPixel color, immediate `show()` |
| `13` | `CMD_LEDS` | `mode` | Set LED mode (`LEDS_OFF`, `LEDS_NORMAL`, `LEDS_INVERTED`, `LEDS_POSITION`) |
| `14` | `CMD_SET_EMITTER` | `level` | Optional Pololu QTR emitter pulse control |

---

## LED modes

| ID | Name | Meaning |
|----|------|---------|
| `0` | `LEDS_OFF` | All neopixels off |
| `1` | `LEDS_NORMAL` | Brightness = sensor intensity (green when calibrated, red when not) |
| `2` | `LEDS_INVERTED` | Inverted brightness |
| `3` | `LEDS_POSITION` | Two LEDs interpolate to show line position |

The firmware keeps a status pixel (index `NUM_SENSORS`) lit when calibration exists.

---

## Calibration

### Auto-calibration

- `CMD_CALIBRATE` puts the device into `MODE_CALIBRATING`.
- The firmware collects min/max extremes for each sensor for ~`CAL_TIME` (5 s).
- During calibration, the status pixel flashes.
- After calibration, `is_calibrated` becomes `true` and the device uses these min/max for normalization.

### Save/load

- `CMD_SAVE_CAL` writes `calMin[]` and `calMax[]` to EEPROM (first 16 bytes).
- `CMD_LOAD_CAL` reads them back and sets `is_calibrated = true`.

### Manual set

- `CMD_SET_MIN`: send 8 bytes (min per sensor).
- `CMD_SET_MAX`: send 8 bytes (max per sensor).

If both min & max have been set, calibration is considered active.

---

## Sensor value interpretation

- **Raw** values are computed as `255 - (analogRead() >> 4)` (range 0..255).
- **Normalized/calibrated** values are scaled to 0..255 using stored `calMin` / `calMax`.
- **Position calculation** uses an inverted normalized value (line = dark) with a weighted average:
  - If `sum > 0`, `pos = (255 * weighted_sum / sum - 255) / 7` → mapped into 0..255 output.
- **Derivative**: moving average of recent `pos` changes, scaled to 0..255 (128 = neutral).
- **Shape detection**: uses threshold `THRESHOLD_BLACK = 100` to classify sensor binary pattern.

---

## Line-shape detection logic (summary)

- Convert each of 8 sensors to binary by: `black = value < 100`.
- Count black sensors and analyze patterns:
  - `T` if many center sensors black in a wide block.
  - `Y` if particular triple patterns exist.
  - `L-left` / `L-right` if most black sensors on one side.
  - Otherwise `STRAIGHT` or `NONE`.

---

## Examples

> Replace `0x33` with `MY_I2C_ADDRESS` if you changed it.

### Arduino: set calibrated mode

```cpp
Wire.beginTransmission(0x33);
Wire.write(1); // CMD_SET_MODE_CAL
Wire.endTransmission();


## Connecting the CH32

| GPIO | function |
|------|----------|
| PB6   | SDC      |
| PB7   | SDA    |
| PB3 | CTRL pin for emitter control | 
| PB11  | NeoPixels |
| PA9   | UART TX |
| PA10 | UART RX |
| PA0   | ADC0  |
| PA1   | ADC1  |
| PA2   | ADC2  |
| PA3   | ADC3  |
| PA4   | ADC4  |
| PA5   | ADC5  |
| PA6   | ADC6  |
| PA7   | ADC7  |
| PB0   | ADC8  |
| PB1   | ADC9  |

#


