# Omnisensor V4R2

Multi sentor with motion detection, temperature, humidity, biughtness.
Easy integration with Home Assistant via RestAPI. Dedicated web interface for update and
debug.
This is a platformIO based project.

## Features

- presence detection with mmWave technology
- integration with Home Assistant (REST API get/set)
- brightness
- temperature and humidity 

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
#define TOKEN "your long lived token"
#define HA_SERVER "your HA server ip address"
#define NODE_IP "your node intended static ip address"
#define DEFAULT_GATEWAY "your network default gateway"
```
The other configuration file is `node_config.h`, here you'll specify the node name and the hardware configuration.

You can find fabrication files on [my website](https://cortimatteo.it) and a video step by step guide [here](https://youtu.be/v1Ju4GubdEQ)


## HA integration

1. You need first to create an uniquie long lived token. Go to Profile → Security → Long
Lived Access Token → Create Token. Paste in the config.h
2. After flashing the firmware and uploading the data file, you will be able to visit the node at it's address (specified in config.h).
There you will find detailed instructions on how to integrate the OmnisensorV4R2 on HomeAssistant

## Hardware

1. LM2575-5 for 5V
2. LM3575-ADJ for 3V3
3. ESP32 development board
4. Custom PCB
5. Buttons
6. Colored LEDs
7. TSL2561 brightness SPI sensor
8. AH10 temperature and humidity SPI sensor 
9. LD2411S mmwave sensor

## License
MIT License