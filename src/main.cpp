
#include <WiFi.h>
#include <WebServer.h>
#include "mmwave.h"
#include "mqtt.h"
#include "led_control.h"
#include <Arduino.h>
#include "config.h"


void setupMQTT(PubSubClient &mqttClient);


const int LED_PIN = 32;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;   // 5 kHz
const int PWM_RES = 8;       // 8-bit resolution (0..255)

bool manualMode = false; 
bool ledState =false; // used in manual mode to store the desired led state


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
// MQTT broker
const char* mqtt_server = "192.168.1.146";  // your Home Assistant broker

const char* mqtt_client_id  ="gym_sensor_01"; // client id for MQTT


void setup() {

  Serial.begin(115200);
  // Initialize LED strip
  myLedStrip.begin(PWM_CHANNEL, LED_PIN, PWM_FREQ, PWM_RES);

  // Initialize mmWave sensor
  initMMWaveSensor(ld2411Serial, mmWave);

  int attempts = 0;
   
  // Connect to Wi-Fi
  IPAddress local_IP(192, 168, 1, 53); // Set your desired static IP address
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

  //detect changes in manual_mode
  if(manualMode != old_manualMode){
    Serial.println("Detected change in operative mode");
    if(manualMode) mqttClient.publish("home/gym/sensor_mode/state", "manual", true);
    else mqttClient.publish("home/gym/sensor_mode/state", "auto", true);
    old_manualMode = manualMode;
  }

  listenMMwave(ld2411Serial, mmWave); // Continuously listen to mmWave data
  static bool oldPresence = false;
  // Handle MQTT connection
  if (!mqttClient.connected()) {
    reconnect_mqtt(mqttClient, mqtt_client_id);
  }
  mqttClient.loop(); // process incoming messages and maintain connection
  if(!manualMode){ // automatic mode
    if(mmWave.presenceDetected != oldPresence ){
      Serial.println("Turning the led " + String(mmWave.presenceDetected ? "ON" : "OFF"));
      if(mmWave.presenceDetected){
        myLedStrip.startFadeIn(); // turn on the led
        mqttClient.publish("home/gym/led/state", "ON", true);
      } else {
        myLedStrip.startFadeOut(); // turn off the led
        mqttClient.publish("home/gym/led/state", "OFF", true);
      } 
      mqttClient.publish("home/gym/ld2411_motion", mmWave.presenceDetected ? "ON" : "OFF", true);
      oldPresence = mmWave.presenceDetected;
    }
  }else{
    if(ledState != myLedStrip.state){
      Serial.println("Manual mode: Turning the led " + String(ledState ? "ON" : "OFF"));
      if(ledState){
        myLedStrip.startFadeIn(); // turn on the led
        mqttClient.publish("home/gym/led/state", "ON", true);
      } else {
        myLedStrip.startFadeOut(); // turn off the led
        mqttClient.publish("home/gym/led/state", "OFF", true);
      }
    }
  }
  myLedStrip.manageLed(); // manage the led fading 
}


void setupMQTT(PubSubClient &mqttClient) {
  // THis function sets up MQTT subscriptions and callbacks
  // It is called once after connecting to the broker

  // Define the callback function to handle incoming messages
  mqttClient.setCallback([&mqttClient](char* topic, byte* payload, unsigned int length) {
    String message;
    bool led_state = false;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    if (String(topic) == "home/gym/sensor_mode/set") {
      if (message == "manual") {
        manualMode = true;
        mqttClient.publish("home/gym/sensor_mode/state", "manual", true);
      } else if (message == "auto") {
        manualMode = false;
        mqttClient.publish("home/gym/sensor_mode/state", "auto", true);
      }
    }
    else if (String(topic) == "home/gym/led/set") {
      if (manualMode) {
        ledState = (message == "ON");
        mqttClient.publish("home/gym/led/state", ledState ? "ON" : "OFF", true);
      }
    }
    else if (String(topic) == "home/gym/maxMotionRange/set") {
        mmWave.maxMotionRange.value = message.toInt();
        mqttClient.publish("home/gym/maxMotionRange/state", message.c_str(), true);
        setMaxMotionRange(ld2411Serial, mmWave, mmWave.maxMotionRange.value);
        mmWave.showParam();
    }
     else if (String(topic) == "home/gym/minMotionRange/set") {
        mmWave.minMotionRange.value = message.toInt();
        mqttClient.publish("home/gym/minMotionRange/state", message.c_str(), true);
        setMinMotionRange(ld2411Serial, mmWave, mmWave.minMotionRange.value);
        mmWave.showParam();
    }
    else if (String(topic) == "home/gym/maxMicroMotionRange/set") {
        mmWave.maxMicroMotionRange.value = message.toInt();
        mqttClient.publish("home/gym/maxMicroMotionRange/state", message.c_str(), true);
        setMaxMicroMotionRange(ld2411Serial, mmWave, mmWave.maxMicroMotionRange.value);
        mmWave.showParam();
    }
    else if (String(topic) == "home/gym/minMicroMotionRange/set") {
        mmWave.minMicroMotionRange.value = message.toInt();
        mqttClient.publish("home/gym/minMicroMotionRange/state", message.c_str(), true);
        setMinMicroMotionRange(ld2411Serial, mmWave, mmWave.minMicroMotionRange.value);
        mmWave.showParam();
    }
    else if (String(topic) == "home/gym/noOneWaitingTime/set") {
        mmWave.noOneWaitingTime.value = message.toInt();
        mqttClient.publish("home/gym/noOneWaitingTime/state", message.c_str(), true);
        setNoOneWaitingTime(ld2411Serial, mmWave, mmWave.noOneWaitingTime.value);
        mmWave.showParam();
    }

  });

  // Subscribe to topics
  mqttClient.subscribe("home/gym/sensor_mode/set");
  mqttClient.subscribe("home/gym/led/set");
  mqttClient.subscribe("home/gym/maxMotionRange/set");
  mqttClient.subscribe("home/gym/minMotionRange/set");
  mqttClient.subscribe("home/gym/maxMicroMotionRange/set");
  mqttClient.subscribe("home/gym/minMicroMotionRange/set");
  mqttClient.subscribe("home/gym/noOneWaitingTime/set");

  // Publish initial states
  mqttClient.publish("home/gym/sensor_mode/state", manualMode ? "manual" : "auto", true);
  mqttClient.publish("home/gym/led/state", ledState ? "ON" : "OFF", true);
  mqttClient.publish("home/gym/maxMotionRange/state", String(mmWave.maxMotionRange.value).c_str(), true);
  mqttClient.publish("home/gym/minMotionRange/state", String(mmWave.minMotionRange.value).c_str(), true);
  mqttClient.publish("home/gym/maxMicroMotionRange/state", String(mmWave.maxMicroMotionRange.value).c_str(), true);
  mqttClient.publish("home/gym/minMicroMotionRange/state", String(mmWave.minMicroMotionRange.value).c_str(), true);
  mqttClient.publish("home/gym/noOneWaitingTime/state", String(mmWave.noOneWaitingTime.value).c_str(), true);
}