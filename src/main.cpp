
#include <WiFi.h>
#include <LittleFS.h>
#include <WebServer.h>
#include "mmwave.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "serial_com.h"
#include "countTime.h"
#include "esp_timer.h"
#include <HTTPClient.h>
#include "node_config.h"
#include "rest_api.h"
#include "inputs.h"
#include "manageWeb.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "endpoints.h"
#include "outputs.h"
#include "sensors.h"

#define INTERVAL_MS 100
#define INTERVAL_MID 30000
#define VERBOSE 1

// Inputs
InputObj redButton("red_button", RED_BTN, 0, 0, false, true, 500);
InputObj greenButton("green_button", GREEN_BTN, 0, 1, false, true, 500);
InputObj blueButton("blue_button", BLUE_BTN, 0, 2, false, true, 500);
InputObj whiteButton("white_button", WHITE_BTN, 0, 3, false, true, 500);
OutputObj redLED("red_led", LED_RED_PIN, false, true);
OutputObj greenLED("green_led", LED_GREEN_PIN, false, true);
OutputObj blueLED("blue_led", LED_BLUE_PIN, false, true);
OutputObj whiteLED("white_led", LED_WHITE_PIN, false, true);


int uartIndex = 1; // default UART index for mmWave sensor
int gpioRX = 16; // default GPIO for mmWave UART RX
int gpioTX = 17; // default GPIO for mmWave UART TX
bool notified = false;
bool lastPresence = false;
String macAddress = "";
String ssid = SSID;
String password = PASSWORD;
bool is_online = false;
WebServer server(80);
HardwareSerial ld2411Serial(2); // use UART2
File uploadFile;
mmWaveSensor mmWave; // instance of mmWaveSensor
WiFiClient espClient;      

HTTPClient http;

int64_t current_time = 0, old_time = 0, old_time_mid = 0; // for timing loops
tempSensObj tempSensor("temp_humidity_sensor"); 
lightSensObj lightSensor("light_sensor");

//Function prototypes
void handleNotFound(); 
void handleRoot(); 
void handleCommand(); 
void setupWebServer();
void handleFileUpload();


void setup() {
  Serial.begin(115200);

  // Print node info
    Serial.println("========================================");
    Serial.println("Node: " + String(NODE_DISPLAY_NAME));
    Serial.println("ID: " + String(NODE_NAME));
    Serial.println("========================================");

  // Initialize mmWave sensor
  mmWave = mmWaveSensor(availableUARTs[uartIndex].serialPort, gpioRX, gpioTX);
  initMMWaveSensor(mmWave);
  Serial.printf("Initialized mmWave sensor with UART%d (RX:%d, TX:%d)\n", 
              uartIndex + 1, gpioRX, gpioTX);

  // initialize and setup I2C sensors
  Wire.begin(21, 22); 
  Wire.setClock(100000);
  delay(100);
  //scanI2C(); // Scan for I2C devices
  // Setup ambient sensors
  tempSensor.setup();
  lightSensor.setup();
  
  // Setup inputs
  setupInputs(redButton, greenButton, blueButton, whiteButton);
  // Setup outputs
  redLED.setup();
  greenLED.setup();
  blueLED.setup();
  whiteLED.setup();

  // Connect to Wi-Fi
  IPAddress local_IP;
  IPAddress gateway;
  int attempts = 0;
  int a, b, c, d, e, f, g, h;  // use int for sscanf
    if (sscanf(NODE_IP, "%d.%d.%d.%d", &a, &b, &c, &d) == 4) {
        local_IP = IPAddress((uint8_t)a, (uint8_t)b, (uint8_t)c, (uint8_t)d);
    }
    if (sscanf(DEFAULT_GATEWAY, "%d.%d.%d.%d", &e, &f, &g, &h) == 4) {
        gateway = IPAddress((uint8_t)e, (uint8_t)f, (uint8_t)g, (uint8_t)h);
    }
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

    if (!SPIFFS.begin(true)) {   // true = format on fail
      Serial.println("SPIFFS Mount Failed!");
      return;
    } else {
      Serial.println("SPIFFS mounted successfully.");
      setupWebServer(); // Setup web server routes
    
    }
    is_online = true;
  }
}

void loop() {
  current_time = esp_timer_get_time()/1000; // Convert microseconds to milliseconds

  // mid loop
  if(current_time - old_time_mid >= INTERVAL_MID) {
    old_time_mid = current_time;
    // Check Wi-Fi connection
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected. Attempting to reconnect...");
      WiFi.reconnect();
      delay(1000); // Wait a bit before checking again
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Reconnected to WiFi.");
        is_online = true;
      } else {
        Serial.println("Failed to reconnect to WiFi.");
        is_online = false;
      }
    }

    // Update ambient sensors data
    lightSensor.updateData();
    tempSensor.updateData();

    // post data to Home Assistant
    updateInputNumber(
        buildEntityId("input_number", "temperature"),
        tempSensor.getTemperature(),
        is_online,
        espClient,
        http
    );
    updateInputNumber(
        buildEntityId("input_number", "humidity"),
        tempSensor.getHumidity(),
        is_online,
        espClient,
        http
    );
    updateInputNumber(
        buildEntityId("input_number", "light_level"),
        lightSensor.getLightLevel(),
        is_online,
        espClient,
        http
    );
    // Print results in the terminal
    if(VERBOSE) {
      Serial.println("Updated sensor data to Home Assistant:");
      Serial.println("Temperature: " + String(tempSensor.getTemperature()) + "°C");
      Serial.println("Humidity: " + String(tempSensor.getHumidity()) + "%");
      Serial.println("Light Level: " + String(lightSensor.getLightLevel()));
    }
    

  }

  // 100ms loop
  if(current_time - old_time >= INTERVAL_MS ) {    
    old_time = current_time;
    // Count time
    countDown(mmWave.t, notified);

    // Update inputs
    updateInputs(is_online, espClient, http, redButton, greenButton, blueButton, whiteButton);
    
    listenGet(mmWave); // Listen to serial commands
    listenMMwave(mmWave); // Continuously listen to mmWave data
    static bool oldPresence = false;
    
  
    if (mmWave.presenceDetected != lastPresence) {
    lastPresence = mmWave.presenceDetected;

    updateInputBoolean(
        buildEntityId("input_boolean", "motion_state"),
        lastPresence,
        is_online,
        espClient,
        http
    );
}
    
    server.handleClient();
  } 
}

void handleNotFound() {
  server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"Not found\"}");
}

void handleRoot() {
  servePage(server, "index.html"); // Main page
  Serial.println("Requested page index.html");
}


void handleCommand() {
  // This endpoint handles commands sent from Home Assistant to control the node
  if (server.method() == HTTP_POST) {
    String command = server.arg("command");
    String value = server.arg("value");
    
    Serial.println("Received command: " + command + " = " + value);
    
    if (command == "param") {
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
    }else if(command == "led") {
      // Handle LEDs control
      String ledColor = server.arg("color");
      bool ledState = (value == "on");
      if(ledColor == "red") {
        redLED.updateState(ledState);
      } else if(ledColor == "green") {
        greenLED.updateState(ledState);
      } else if(ledColor == "blue") {
        blueLED.updateState(ledState);
      } else if(ledColor == "white") {
        whiteLED.updateState(ledState);
      }
      server.send(200, "text/plain", "OK");
    }else{
      server.send(400, "text/plain", "Unknown command");
    }
  }
}


void setupWebServer() {
  // This function sets up the web server routes for handling incoming HTTP requests
  server.on("/", HTTP_GET, handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started on port 80");
  Serial.println("Access it at: http://" + WiFi.localIP().toString());

  // Endpoint to get JSON configuration
  server.on("/getConfig", HTTP_GET, []() {
    Serial.println("Requested /getConfig");
    JsonDocument doc;
    // Add node general information
    doc["nodeName"] = NODE_NAME;
    doc["nodeDisplayName"] = NODE_DISPLAY_NAME;
    doc["IP_address"] = NODE_IP;
    doc["FW_version"] = FW_VERSION;

    // Populate the JSON document with the mmwave configuration values
    doc["maxMotionRange"] = mmWave.maxMotionRange.value;
    doc["minMotionRange"] = mmWave.minMotionRange.value;
    doc["maxMicroMotionRange"] = mmWave.maxMicroMotionRange.value;
    doc["minMicroMotionRange"] = mmWave.minMicroMotionRange.value;
    doc["noOneWaitingTime"] = mmWave.noOneWaitingTime.value;

    // Add inputs configuration
    JsonArray inputs = doc.createNestedArray("inputs");
    if(redButton.isPresent) {
      JsonObject redBtn = inputs.createNestedObject();
      redBtn["name"] = redButton.name;
      redBtn["gpio"] = redButton.gpio;
      redBtn["state"] = redButton.state;
    }
    if(greenButton.isPresent) {
      JsonObject greenBtn = inputs.createNestedObject();
      greenBtn["name"] = greenButton.name;
      greenBtn["gpio"] = greenButton.gpio;
      greenBtn["state"] = greenButton.state;
    }
    if(blueButton.isPresent) {
      JsonObject blueBtn = inputs.createNestedObject();
      blueBtn["name"] = blueButton.name;
      blueBtn["gpio"] = blueButton.gpio;
      blueBtn["state"] = blueButton.state;
    }
    if(whiteButton.isPresent) {
      JsonObject whiteBtn = inputs.createNestedObject();
      whiteBtn["name"] = whiteButton.name;
      whiteBtn["gpio"] = whiteButton.gpio;
      whiteBtn["state"] = whiteButton.state;
    }

    // Add global configuration
    doc["nodeName"] = NODE_NAME;

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  // Endpoint to get current sensor readings as JSON
  server.on("/getReadings", HTTP_GET, []() {
    Serial.println("Requested /getReadings");

    // Refresh sensor data so the page always shows fresh values
    tempSensor.updateData();
    lightSensor.updateData();

    JsonDocument doc;
    doc["nodeName"] = NODE_NAME;
    doc["nodeDisplayName"] = NODE_DISPLAY_NAME;
    doc["fwVersion"] = FW_VERSION;
    doc["rssi"] = WiFi.RSSI();

    doc["temperature"] = tempSensor.getTemperature();
    doc["humidity"] = tempSensor.getHumidity();
    doc["lightLevel"] = lightSensor.getLightLevel();
    doc["presence"] = mmWave.presenceDetected;
    doc["distance"] = mmWave.distance;

    JsonArray inputs = doc.createNestedArray("inputs");
    JsonObject redBtn = inputs.createNestedObject();
    redBtn["name"] = redButton.name;
    redBtn["state"] = redButton.state;
    JsonObject greenBtn = inputs.createNestedObject();
    greenBtn["name"] = greenButton.name;
    greenBtn["state"] = greenButton.state;
    JsonObject blueBtn = inputs.createNestedObject();
    blueBtn["name"] = blueButton.name;
    blueBtn["state"] = blueButton.state;
    JsonObject whiteBtn = inputs.createNestedObject();
    whiteBtn["name"] = whiteButton.name;
    whiteBtn["state"] = whiteButton.state;

    JsonArray leds = doc.createNestedArray("leds");
    JsonObject redLed = leds.createNestedObject();
    redLed["name"] = redLED.name;
    redLed["state"] = redLED.state;
    JsonObject greenLed = leds.createNestedObject();
    greenLed["name"] = greenLED.name;
    greenLed["state"] = greenLED.state;
    JsonObject blueLed = leds.createNestedObject();
    blueLed["name"] = blueLED.name;
    blueLed["state"] = blueLED.state;
    JsonObject whiteLed = leds.createNestedObject();
    whiteLed["name"] = whiteLED.name;
    whiteLed["state"] = whiteLED.state;

    String json;
    serializeJson(doc, json);
    server.send(200, "application/json", json);
  });

  // Endpoint for the live readings page
  server.on("/readings.html", []() {
        servePage(server, "readings.html");
        Serial.println("Requested page readings.html");
  });

  // Enpoint for the update page
  server.on("/updatePage.html", []() {
        servePage(server, "updatePage.html"); // Update page
        Serial.println("Requested page updatePage.html");
  });

  // Set up firmware update endpoint
  handleFirmwareUpdate(server);

  // Endpoint the updating of the data folder
  server.on("/updateDataFiles", HTTP_POST, []() {
    server.send(200, "text/plain", "File upload successful");
  }, handleFileUpload);

  // Route for serving CSS
  server.on("/style.css", []() {
      File file = SPIFFS.open("/style.css", "r");
      if (!file) {
          Serial.println("Failed to open style.css for reading");
          return;
      }
      server.streamFile(file, "text/css");
        
      file.close();
  });


  // Enpoint for the web version
  server.on("/web_version.json", []() {
      File file = SPIFFS.open("/web_version.json", "r");
      if (!file) {
          Serial.println("Failed to open web_version.json for reading");
          return;
      }
      server.streamFile(file, "application/json");
        
      file.close();
  });


}

// Handle file upload
void handleFileUpload() {
  HTTPUpload& upload = server.upload();
    
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Start uploading: %s\n", upload.filename.c_str());
    uploadFile = SPIFFS.open("/" + upload.filename, FILE_WRITE);
    if (!uploadFile) {
      Serial.println("Failed to open file for writing");
      return;
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadFile.write(upload.buf, upload.currentSize);
      Serial.printf("Writing %d bytes\n", upload.currentSize);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      Serial.printf("Upload complete: %s, Size: %u\n", upload.filename.c_str(), upload.totalSize);
    }
  }
}
