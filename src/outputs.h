#ifndef OUTPUTS_H
#define OUTPUTS_H
#include <Arduino.h>

struct OutputObj {
    String name;
    uint8_t gpio;
    bool state;
    bool isPresent;

    // Constructor
    OutputObj(String n, uint8_t g, bool s, bool present) 
        : name(n), gpio(g), state(s), isPresent(present) {}

    void setup() {
        if(!isPresent) return; // skip setup if not present
        pinMode(gpio, OUTPUT);
        digitalWrite(gpio, state ? HIGH : LOW); // Set initial state
        Serial.printf("Output %s (GPIO %d) setup as OUTPUT with initial state %s\n", name.c_str(), gpio, state ? "HIGH" : "LOW");
    }

    void updateState(bool newState) {
        if(!isPresent) return; // skip update if not present
        if (newState != state) {
            state = newState;
            digitalWrite(gpio, state ? HIGH : LOW);
            Serial.printf("Output %s (GPIO %d) state changed to %s\n", name.c_str(), gpio, state ? "HIGH" : "LOW");
        }
    }
};


#endif