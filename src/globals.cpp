#include "globals.h"


// Define and initialize the array
HSobj availableUARTs[3] = {
    {&Serial1, false},   // UART1
    {&Serial2, false},   // UART2 
    {nullptr, false}     // Additional slot (could be SoftwareSerial)
};