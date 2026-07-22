#ifndef REST_API_H
#define REST_API_H

#include <Arduino.h>
#include <HTTPClient.h>
#include "config.h"
#include "globals.h"
#include "node_config.h"

void updateInputBoolean(String entity_id, bool state, bool isOnline, WiFiClient& espClient, HTTPClient& http);

void updateInputNumber(String entity_id, float value, bool isOnline, WiFiClient& espClient, HTTPClient& http);

String buildEntityId(String entity_type, String entity_name = "");
#endif