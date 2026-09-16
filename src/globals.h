#ifndef FW_VERSION
#define FW_VERSION 13

#define EEPROM_SIZE 1024
#define TIMEOUT_SIREN 1200
#define EEPROM_SIZE 1024


#include <HardwareSerial.h>

struct HSobj {
    HardwareSerial* serialPort;
    bool burned;
    // Default constructor
    HSobj() : serialPort(nullptr), burned(false) {}
    // Parameterized constructor for array initialization
    HSobj(HardwareSerial* serial, bool burn) : serialPort(serial), burned(burn) {}
};



// Declare the array globally
extern HSobj availableUARTs[3];  // Declare extern in header

#endif 