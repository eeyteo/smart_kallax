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
    bool isPresent;
    unsigned long minActiveTime;  // Minimum time required ([ms] specified in config file])
    bool pendingState;            // Candidate state while debouncing
    unsigned long pendingSince;   // When the candidate state was first seen

    // Constructor
    InputObj(String n, uint8_t g, uint8_t t, uint8_t i, bool s, bool present, unsigned long m) 
        : name(n), gpio(g), type(t), id(i), state(s), isPresent(present), minActiveTime(m),
          pendingState(s), pendingSince(0) {}

    void setup() {
        if(!isPresent) return; // skip setup if not present
        pinMode(gpio, (type == 0) ? INPUT_PULLUP : INPUT); // type 0 = button with pull-up, type 1 = regular input
        Serial.printf("Input %s (GPIO %d) setup as %s\n", name.c_str(), gpio, (type == 0) ? "INPUT_PULLUP" : "INPUT");
    }

    void updateState(bool isOnline, WiFiClient& espClient, HTTPClient& http) {
        if(!isPresent) return; // skip update if not present
        bool raw = (type == 0) ? !digitalRead(gpio) : digitalRead(gpio); // invert for pull-up
        unsigned long now = millis();

        if (raw != state) {
            // Debounce: commit a change only after it has been stable for minActiveTime
            if (raw != pendingState) {
                pendingState = raw;
                pendingSince = now;
            } else if (now - pendingSince >= minActiveTime) {
                // Commit only on success so a failed request is retried instead of lost
                if (updateInputBoolean(buildEntityId("input_boolean", name),
                                       raw, isOnline, espClient, http)) {
                    state = raw;
                } else {
                    pendingSince = now; // back off before retrying
                }
            }
        } else {
            // Raw matches the committed state: clear any pending transition
            pendingState = state;
        }
    }
};


void setupInputs(InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton);
void updateInputs(bool isOnline, WiFiClient& espClient, HTTPClient& http, InputObj& redButton, InputObj& greenButton, InputObj& blueButton, InputObj& whiteButton);

#endif