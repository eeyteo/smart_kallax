# ESP32 Smart Kallax

Firmware for managing a cusom PCB, based on a ESP32 development boar.
This board will manage motion detection from a mm wave sensor and will
control a 12V output such as a led strip.
This is a platformIO based project.

## Features

- Soft turn on and turn off of a led output
- mmWave LD2411S sensor support
- integration with Home Assistant (get/set)

## Usage

Include the complmentary libraries from this [complementary repository](https://github.com/eeyteo/ESP32_custom_libs), in the platformIO.ini file

```xml
lib_extra_dirs = ..\PlatformIO\Libs
```
Add in folder src a `config.h` file with sensible information
```xml
#pragma once
#define SSID "your ssid"
#define PASSWORD "your password"
#define HA_SERVER "your HA server ip address"
#define NODE_IP "your node intended static ip address"
#define DEFAULT_GATEWAY "your network default gateway"
```
The other configuration file is `node_config.h`, here you'll specify the node name and the hardware configuration.

You can find fabrication files on [my website](https://cortimatteo.it) and a video step by step guide [here](https://youtu.be/v1Ju4GubdEQ)

## Hardware

1. LM2575-5 for generating 5V line
2. IRLZ44N for managing the led strip
3. ESP32 development board
4. Custom PCB

## License

MIT License