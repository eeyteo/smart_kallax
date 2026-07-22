#include "rest_api.h"


void updateInputNumber(String entity_id, float value, bool isOnline, WiFiClient& espClient, HTTPClient& http) {
  // This function updates an input_number in Home Assistant via the REST API

  if (!isOnline || WiFi.status() != WL_CONNECTED) return;

  // Use the service API, not states API
  String url = "http://" + String(HA_SERVER) + ":8123/api/services/input_number/set_value";
  
  String payload = "{\"entity_id\": \"" + entity_id + "\", \"value\": " + String(value, 2) + "}";
  
  http.begin(espClient, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(TOKEN));
  
  int httpCode = http.POST(payload);  // Use POST for services!
  
  if (httpCode == 200) {
    Serial.println("OK " + entity_id + " set to " + String(value, 2));
  } else {
    Serial.println("KO Failed: " + String(httpCode) + " for " + entity_id);
  }
  
  http.end();
}


void updateInputBoolean(String entity_id, bool state, bool isOnline, WiFiClient& espClient, HTTPClient& http) {
  // This function updates an input_boolean in Home Assistant via the REST API

  if (!isOnline || WiFi.status() != WL_CONNECTED) return;

  // Use the service API, not states API
  String url = "http://" + String(HA_SERVER) + ":8123/api/services/input_boolean/turn_" + 
               String(state ? "on" : "off");
  
  String payload = "{\"entity_id\": \"" + entity_id + "\"}";
  
  http.begin(espClient, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(TOKEN));
  
  int httpCode = http.POST(payload);  // Use POST for services!
  
  if (httpCode == 200) {
    Serial.println("OK " + entity_id + " set to " + String(state ? "ON" : "OFF"));
  } else {
    Serial.println("KO Failed: " + String(httpCode) + " for " + entity_id);
  }
  
  http.end();
}


String buildEntityId(String entity_type, String entity_name) {
  // This function builds a Home Assistant entity ID based on the node name and entity type
  // Example: input_boolean.gym_kallax_motion_state
    if (entity_name.length() > 0) {
        return entity_type + "." + String(NODE_NAME) + "_" + entity_name;
    }
    return entity_type + "." + String(NODE_NAME);
}
