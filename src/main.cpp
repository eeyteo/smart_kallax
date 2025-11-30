
#include <WiFi.h>
#include <WebServer.h>
#include "mmwave.h"
#include "mqtt.h"
#include "led_control.h"
#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "serial_com.h"
#include "countTime.h"
#include "esp_timer.h"

#define INTERVAL_MS 100
// Prototypes
void setupMQTT(PubSubClient &mqttClient);

// Node Variable
const char* mqtt_server = "192.168.1.146";  // your Home Assistant broker
const char* mqtt_client_id  ="livingroom_sensor_01"; // client id for MQTT
const bool led_present = true; // set to true if an led strip is connected to the board

const int LED_PIN = 32;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;   // 5 kHz
const int PWM_RES = 8;       // 8-bit resolution (0..255)

bool manualMode = false; 
bool ledState = false; // used in manual mode to store the desired led state
bool notified = false;
String macAddress = "";
String ssid = SSID;
String password = PASSWORD;
bool is_online = false, old_manualMode;
WebServer server(80);
HardwareSerial ld2411Serial(2); // use UART2

mmWaveSensor mmWave; // instance of mmWaveSensor
ledStrip myLedStrip; // instance of ledStrip
WiFiClient espClient;       // <-- needed by PubSubClient
PubSubClient mqttClient(espClient);

int64_t current_time = 0, old_time = 0;

void setup() {
  Serial.begin(115200);
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
  IPAddress local_IP(192, 168, 1, 61); // Set your desired static IP address
  IPAddress gateway(192, 168, 1, 1);    // Replace with your network gateway
  IPAddress subnet(255, 255, 255, 0);   // Replace with your subnet mask
  IPAddress dns(8, 8, 8, 8); // Google's public DNS server

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
    // failed connection
    Serial.println("failed to connect to WiFi.");
  } else {
    // Wi-Fi credentials available and connection successful
    macAddress = WiFi.macAddress();
    Serial.println("Device " + macAddress + " connected at "  + WiFi.localIP().toString() + " . RSSI: " + String(WiFi.RSSI()) + "dBm");  
    is_online = true;
    // Setup MQTT
    mqttClient.setServer(mqtt_server, 1883);
    
    // Connect to MQTT broker
    if (!mqttClient.connected()) {
      reconnect_mqtt(mqttClient, mqtt_client_id);
    }
    // Setup MQTT
    setupMQTT(mqttClient);

  }

}

void loop() {
  current_time = esp_timer_get_time()/1000; // Convert to milliseconds
  // 100ms loop
  if(current_time - old_time >= INTERVAL_MS ) {    
    old_time = current_time;
    // Count time
    countDown(mmWave.t, notified);

    // Detect changes in manual_mode
    if(manualMode != old_manualMode){
      Serial.println("Detected change in operative mode");
      if(manualMode) mqttClient.publish("home/livingroom/sensor_mode/state", "manual", true);
      else mqttClient.publish("home/livingroom/sensor_mode/state", "auto", true);
      old_manualMode = manualMode;
    }
    
    listenMMwave(mmWave); // Continuously listen to mmWave data
    static bool oldPresence = false;

    // Handle MQTT connection
    if (!mqttClient.connected()) {
      Serial.println("Lost connection to MQTT. Reconnecting to MQTT...");
      reconnect_mqtt(mqttClient, mqtt_client_id);
    }
    mqttClient.loop(); // process incoming messages and maintain connection
    
    if(!manualMode){ // automatic mode
      if(mmWave.presenceDetected != oldPresence ){
        Serial.println("Turning the led " + String(mmWave.presenceDetected ? "ON" : "OFF"));
        if(mmWave.presenceDetected){
          myLedStrip.startFadeIn(); // turn on the led
          mqttClient.publish("home/livingroom/led/state", "ON", true);
        } else {
          myLedStrip.startFadeOut(); // turn off the led
          mqttClient.publish("home/livingroom/led/state", "OFF", true);
        } 
        mqttClient.publish("home/livingroom/ld2411_motion", mmWave.presenceDetected ? "ON" : "OFF", true);
        oldPresence = mmWave.presenceDetected;
      }
    }else{ // manual mode
      if(ledState != myLedStrip.state){
        Serial.println("Manual mode: Turning the led " + String(ledState ? "ON" : "OFF"));
        if(ledState){
          myLedStrip.startFadeIn(); // turn on the led
          mqttClient.publish("home/livingroom/led/state", "ON", true);
        } else {
          myLedStrip.startFadeOut(); // turn off the led
          mqttClient.publish("home/livingroom/led/state", "OFF", true);
        }
      }
    }
    if(led_present) myLedStrip.manageLed(); // manage the led fading if the led is present
  } 
}


void setupMQTT(PubSubClient &mqttClient) {
  // This function sets up MQTT subscriptions and callbacks
  // It is called once after connecting to the broker

  // Define the callback function to handle incoming messages
  mqttClient.setCallback([&mqttClient](char* topic, byte* payload, unsigned int length) {
    String message;
    bool led_state = false;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    if (String(topic) == "home/livingroom/sensor_mode/set") {
      if (message == "manual") {
        manualMode = true;
        mqttClient.publish("home/livingroom/sensor_mode/state", "manual", true);
      } else if (message == "auto") {
        manualMode = false;
        mqttClient.publish("home/livingroom/sensor_mode/state", "auto", true);
      }
    }else if (String(topic) == "home/livingroom/led/set") {
      if (manualMode) {
        ledState = (message == "ON");
        mqttClient.publish("home/livingroom/led/state", ledState ? "ON" : "OFF", true);
      }
    }else if (String(topic) == "home/livingroom/maxMotionRange/set") {
        mmWave.maxMotionRange.value = message.toInt();
        mqttClient.publish("home/livingroom/maxMotionRange/state", message.c_str(), true);
        setMaxMotionRange(mmWave, mmWave.maxMotionRange.value);
        mmWave.showParam();
    }else if (String(topic) == "home/livingroom/minMotionRange/set") {
        mmWave.minMotionRange.value = message.toInt();
        mqttClient.publish("home/livingroom/minMotionRange/state", message.c_str(), true);
        setMinMotionRange(mmWave, mmWave.minMotionRange.value);
        mmWave.showParam();
    }else if (String(topic) == "home/livingroom/maxMicroMotionRange/set") {
        mmWave.maxMicroMotionRange.value = message.toInt();
        mqttClient.publish("home/livingroom/maxMicroMotionRange/state", message.c_str(), true);
        setMaxMicroMotionRange(mmWave, mmWave.maxMicroMotionRange.value);
        mmWave.showParam();
    }else if (String(topic) == "home/livingroom/minMicroMotionRange/set") {
        mmWave.minMicroMotionRange.value = message.toInt();
        mqttClient.publish("home/livingroom/minMicroMotionRange/state", message.c_str(), true);
        setMinMicroMotionRange(mmWave, mmWave.minMicroMotionRange.value);
        mmWave.showParam();
    }else if (String(topic) == "home/livingroom/noOneWaitingTime/set") {
        mmWave.noOneWaitingTime.value = message.toInt();
        mqttClient.publish("home/livingroom/noOneWaitingTime/state", message.c_str(), true);
        setNoOneWaitingTime(mmWave, mmWave.noOneWaitingTime.value);
        mmWave.showParam();
    }

  });

  // Subscribe to topics
  mqttClient.subscribe("home/livingroom/sensor_mode/set");
  mqttClient.subscribe("home/livingroom/led/set");
  mqttClient.subscribe("home/livingroom/maxMotionRange/set");
  mqttClient.subscribe("home/livingroom/minMotionRange/set");
  mqttClient.subscribe("home/livingroom/maxMicroMotionRange/set");
  mqttClient.subscribe("home/livingroom/minMicroMotionRange/set");
  mqttClient.subscribe("home/livingroom/noOneWaitingTime/set");

  // Publish states
  mqttClient.publish("home/livingroom/sensor_mode/state", manualMode ? "manual" : "auto", true);
  mqttClient.publish("home/livingroom/led/state", ledState ? "ON" : "OFF", true);
  mqttClient.publish("home/livingroom/maxMotionRange/state", String(mmWave.maxMotionRange.value).c_str(), true);
  mqttClient.publish("home/livingroom/minMotionRange/state", String(mmWave.minMotionRange.value).c_str(), true);
  mqttClient.publish("home/livingroom/maxMicroMotionRange/state", String(mmWave.maxMicroMotionRange.value).c_str(), true);
  mqttClient.publish("home/livingroom/minMicroMotionRange/state", String(mmWave.minMicroMotionRange.value).c_str(), true);
  mqttClient.publish("home/livingroom/noOneWaitingTime/state", String(mmWave.noOneWaitingTime.value).c_str(), true);
}