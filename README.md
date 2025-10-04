# ESP32 Smart Kallax

A ESP32 development board, platformIO based, using a custom PCB project.
This board will manage motion detection via mm wave sensor and control a 12V
output such as a led strip, intended to be mounted on a wardrobe.

## Features

- MQTT client support
- Soft turn on and turn off of a led strip
- mmWave LD2411S sensor support
- integration with Home Assistant

## Usage

Include the libraries from this [complementary repository](https://github.com/eeyteo/ESP32_custom_libs), in the platformIO.ini file

```xml
lib_extra_dirs = ..\PlatformIO\Libs
```
Add in folder src a config.h file with
```xml
#pragma once
#define SSID "your ssid"
#define PASSWORD "your password"
```

You can find fabrication files on [my website](https://cortimatteo.it) and a video step by step guide [here](https://youtu.be/v1Ju4GubdEQ)

## Hardware

1. LM2575-5 for generating 5V line
2. IRLZ44N for managing the led strip
3. ESP32 development board
4. Custom PCB

## License

MIT License