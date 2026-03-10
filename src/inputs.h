#ifndef INPUTS_H
#define INPUTS_H
#include <Arduino.h>
#include <HTTPClient.h>
#include "rest_api.h"

struct InputObj {
    String name;
    uint8_t gpio;
    uint8_t type;
    uint8_t id;
    bool state;
    unsigned long minActiveTime;  // Minimum time required ([ms] specified in config file])

    // Constructor
    InputObj(String n, uint8_t g, uint8_t t, uint8_t i, bool s, unsigned long m) 
        : name(n), gpio(g), type(t), id(i), state(s), minActiveTime(m) {}

    void setup() {
        pinMode(gpio, (type == 0) ? INPUT_PULLUP : INPUT); // type 0 = button with pull-up, type 1 = regular input
    }

    void updateState(bool isOnline, WiFiClient& espClient, HTTPClient& http) {
        bool newState = (type == 0) ? !digitalRead(gpio) : digitalRead(gpio); // invert for pull-up
        if (newState != state) {
            state = newState;
            // Send an update to Home Assistant
            updateInputBoolean(buildEntityId("input_boolean", name), 
                      state, isOnline, espClient, http);
        }
    }
};


void setupInputs(InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton);
void updateInputs(bool isOnline, WiFiClient& espClient, HTTPClient& http, InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton);

#endif