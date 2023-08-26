# RangingSensor
Snow depth measurement using Garmin Lidar Lite v4 sensor.

## Table of Contents

- [Description](#description)
- [Features](#features)
- [Hardware Setup](#hardware-setup)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Description

## Features

- Measures distances using the Garmin Lidar Lite v4 sensor.
- Supports both I2C and Ant Protocol for distance measurement

## Hardware Setup


### Interfacing an LCD Display (16x2 Character-based)

LCD Pin | Raspberry Pi GPIO Pin
------- | ---------------------
VCC     | 5V or 3.3V (depends on LCD)
GND     | GND
RS      | GPIO Pin (e.g., 25)
RW      | GND (for write mode)
E       | GPIO Pin (e.g., 24)
D4-D7   | GPIO Pins (e.g., 23, 17, 21, 22)
LED+    | 5V (if the backlight is used)
LED-    | GND (if backlight is used)

### Interfacing a GPS Module (UART Interface)

GPS Module Pin | Raspberry Pi GPIO Pin (UART)
-------------- | ----------------------------
VCC            | 3.3V
GND            | GND
TX             | RX (GPIO Pin, e.g., 10)
RX             | TX (GPIO Pin, e.g., 8)

Sensor Pin | Raspberry Pi GPIO Pin (I2C)
---------- | --------------------------
VCC        | 5V or 3.3V (depending on sensor)
GND        | GND
SDA        | GPIO 2 (SDA)
SCL        | GPIO 3 (SCL)



## Library Dependcy
1. OpenAnt ()
2. I2C
3. GPS
4. LCD Display

## Installation

1. Enable I2C on your Raspberry Pi and install the necessary libraries:
   ```bash
   pip install adafruit-circuitpython-busdevice
