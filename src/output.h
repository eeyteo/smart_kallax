#include <Arduino.h>

struct OutputObj {
    String name;
    uint8_t gpio;
    uint8_t type;
    uint8_t id;
    bool state;

    // Constructor
    OutputObj(String n, uint8_t g, uint8_t t, uint8_t i, bool s) 
        : name(n), gpio(g), type(t), id(i), state(s) {}
        