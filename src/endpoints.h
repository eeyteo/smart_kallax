#ifndef ENDPOINTS_H
#define ENDPOINTS_H

#include "Update.h"
#include <WebServer.h> // Use WebServer for ESP32
#include <ArduinoJson.h>
#include "mmwave.h"

// Declare the function
void handleFirmwareUpdate(WebServer &server);
#endif // ENDPOINTS_H