
#include <WiFi.h>
#include <WebServer.h>
#include "mmwave.h"
#include "led_control.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "serial_com.h"
#include "countTime.h"
#include "esp_timer.h"
#include <HTTPClient.h>
#include "node_config.h"

#define INTERVAL_MS 100
#define HA_SERVER "192.168.1.146"

// Node Variable
const bool led_present = true; // set to true if an led strip is connected to the board


bool manualMode = false; 
bool ledState = false, lastLedState = false; // used in manual mode to store the desired led state
bool notified = false;
String macAddress = "";
String ssid = SSID;
String password = PASSWORD;
bool is_online = false, old_manualMode;
WebServer server(80);
HardwareSerial ld2411Serial(2); // use UART2

mmWaveSensor mmWave; // instance of mmWaveSensor
ledStrip myLedStrip; // instance of ledStrip
WiFiClient espClient;      

HTTPClient http;

int64_t current_time = 0, old_time = 0;


//Function prototypes
void updateInputBoolean(String entity_id, bool state);
String buildEntityId(String entity_type, String entity_name = "");
String buildHAService(String service_type, String entity_id, String state);
void handleNotFound(); 
void handleRoot(); 
void handleCommand(); 
void setupWebServer();


void setup() {
  Serial.begin(115200);

  // Print node info
    Serial.println("========================================");
    Serial.println("Node: " + String(NODE_DISPLAY_NAME));
    Serial.println("ID: " + String(NODE_NAME));
    Serial.println("========================================");


  int uartIndex = -1;
  int gpio = 16; // default GPIO for mmWave UART RX
  // Initialize LED strip if the led strip is present
  if(led_present) myLedStrip.begin(PWM_CHANNEL, LED_PIN, PWM_FREQ, PWM_RES);

  // Initialize mmWave sensor
  // find the first non burned UART
  for (int j = 0; j < 2; j++) { // only 2 HardwareSerial available
      if (!availableUARTs[j].burned) {
          uartIndex = j;
            availableUARTs[j].burned = true; // mark as burned
          break;
      }
  }
  mmWave = mmWaveSensor(availableUARTs[uartIndex].serialPort, gpio, gpio + 1);
  initMMWaveSensor(mmWave);
  Serial.printf("Initialized mmWave sensor with UART%d (RX:%d, TX:%d)\n", 
              uartIndex + 1, gpio, gpio + 1);
  int attempts = 0;
   
  // Connect to Wi-Fi
  IPAddress local_IP(192, 168, 1, 62);  // Set your desired static IP address
  IPAddress gateway(192, 168, 1, 1);    // Replace with your network gateway
  IPAddress subnet(255, 255, 255, 0);   // Replace with your subnet mask
  IPAddress dns(8, 8, 8, 8);            // Google's public DNS server

  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("Failed to configure static IP");
  }
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
      Serial.println("Connecting to WiFi..." + ssid); // + " with the password " +  password);
      delay(2000);  // Wait for 2 seconds before retrying
      attempts++;
  }
  
  if (ssid == "" || WiFi.status() != WL_CONNECTED) {
    // Failed connection
    Serial.println("failed to connect to WiFi.");
  } else {
    // Wi-Fi credentials available and connection successful
    macAddress = WiFi.macAddress();
    Serial.println("Device " + macAddress + " connected at "  + WiFi.localIP().toString() + " . RSSI: " + String(WiFi.RSSI()) + "dBm");  
    setupWebServer(); // Setup web server routes
    is_online = true;
  }
}

void loop() {
  current_time = esp_timer_get_time()/1000; // Convert to milliseconds
  // 100ms loop
  if(current_time - old_time >= INTERVAL_MS ) {    
    old_time = current_time;
    // Count time
    countDown(mmWave.t, notified);

    listenGet(mmWave); // Listen to serial commands
    listenMMwave(mmWave); // Continuously listen to mmWave data
    static bool oldPresence = false;
    
     // Auto mode: control LED based on presence
    if (!manualMode) {
      if (mmWave.presenceDetected && myLedStrip.canFadeIn()) {
        myLedStrip.machine_state = 1; // start fade in
        updateInputBoolean(buildEntityId("input_boolean", "motion_state"), 
                      mmWave.presenceDetected);
        Serial.println("Auto mode: Presence detected, turning LED ON");
      } else if(!mmWave.presenceDetected && myLedStrip.canFadeOut()) {
        myLedStrip.machine_state = 3; // start fade out
        updateInputBoolean(buildEntityId("input_boolean", "motion_state"), 
                      mmWave.presenceDetected);
        Serial.println("Auto mode: No presence, turning LED OFF");
      }
    }
    
    // Manual mode: update LED state if changed
    if (manualMode && ledState != lastLedState) {
      Serial.println("Manual mode: Turning LED " + String(ledState ? "ON" : "OFF"));
      if (ledState && myLedStrip.canFadeIn()) {
        myLedStrip.machine_state = 1;
      } else if(!ledState && myLedStrip.canFadeOut()) {
        myLedStrip.machine_state = 3;
      }
      lastLedState = ledState;
    }

    if(led_present) manageLed(myLedStrip); // manage the led fading if the led is present
    server.handleClient();
  } 
}


void updateInputBoolean(String entity_id, bool state) {
  if (!is_online || WiFi.status() != WL_CONNECTED) return;

  // Use the service API, not states API
  String url = "http://" + String(HA_SERVER) + ":8123/api/services/input_boolean/turn_" + 
               String(state ? "on" : "off");
  
  String payload = "{\"entity_id\": \"" + entity_id + "\"}";
  
  http.begin(espClient, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(TOKEN));
  
  int httpCode = http.POST(payload);  // Use POST for services!
  
  if (httpCode == 200) {
    Serial.println("✓ " + entity_id + " set to " + String(state ? "ON" : "OFF"));
  } else {
    Serial.println("✗ Failed: " + String(httpCode) + " for " + entity_id);
  }
  
  http.end();
}

void handleNotFound() {
  server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"Not found\"}");
}

void handleRoot() {
  server.send(200, "text/plain", "Living Room Sensor Running");
}

void handleCommand() {
  if (server.method() == HTTP_POST) {
    String command = server.arg("command");
    String value = server.arg("value");
    
    Serial.println("Received command: " + command + " = " + value);
    
    if (command == "mode") {
      manualMode = (value == "manual");
      server.send(200, "text/plain", "OK");
    } else if (command == "led") {
      ledState = (value == "on");
      server.send(200, "text/plain", "OK");
    } else if (command == "param") {
      // Handle parameter changes
      String param = server.arg("param");
      int intValue = value.toInt();
      if(param == "maxMotionRange"){
        Serial.println("Setting maxMotionRange to " + String(intValue));
        mmWave.maxMotionRange.value = intValue;
        setMaxMotionRange(mmWave, mmWave.maxMotionRange.value);
      }else if(param == "minMotionRange"){
        Serial.println("Setting minMotionRange to " + String(intValue));
        mmWave.minMotionRange.value = intValue;
        setMinMotionRange(mmWave, mmWave.minMotionRange.value);
      } else if(param == "maxMicroMotionRange") {
        Serial.println("Setting maxMicroMotionRange to " + String(intValue));
        mmWave.maxMicroMotionRange.value = intValue;
        setMaxMicroMotionRange(mmWave, mmWave.maxMicroMotionRange.value);
      } else if(param == "minMicroMotionRange") {
        Serial.println("Setting minMicroMotionRange to " + String(intValue));
        mmWave.minMicroMotionRange.value = intValue;
        setMinMicroMotionRange(mmWave, mmWave.minMicroMotionRange.value);
      }else if(param == "noOneWaitingTime") {
        Serial.println("Setting noOneWaitingTime to " + String(intValue));
        mmWave.noOneWaitingTime.value = intValue;
        setNoOneWaitingTime(mmWave, mmWave.noOneWaitingTime.value);
      }
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Unknown command");
    }
  }
}


void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started on port 80");
  Serial.println("Access it at: http://" + WiFi.localIP().toString());
}


String buildEntityId(String entity_type, String entity_name) {
    if (entity_name.length() > 0) {
        return entity_type + "." + String(NODE_NAME) + "_" + entity_name;
    }
    return entity_type + "." + String(NODE_NAME);
}
