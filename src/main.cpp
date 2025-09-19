
#include <WiFi.h>
#include <WebServer.h>
#include "mmwave.h"
#include "mqtt.h"
#include "led_control.h"
#include <Arduino.h>
#include "config.h"

const int LED_PIN = 32;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;   // 5 kHz
const int PWM_RES = 8;       // 8-bit resolution (0..255)

extern bool manualMode;
extern bool ledState;


String macAddress = "";
String ssid = SSID;
String password = PASSWORD;
bool is_online = false;
WebServer server(80);
HardwareSerial ld2411Serial(2); // use UART2

mmWaveSensor mmWave; // instance of mmWaveSensor
ledStrip myLedStrip; // instance of ledStrip
WiFiClient espClient;       // <-- needed by PubSubClient
PubSubClient mqttClient(espClient);
// MQTT broker
const char* mqtt_server = "192.168.1.146";  // your Home Assistant broker

const char* mqtt_client_id  ="gym_sensor_01"; // client id for MQTT

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myLedStrip.begin(PWM_CHANNEL, LED_PIN, PWM_FREQ, PWM_RES);

  initMMWaveSensor(ld2411Serial, mmWave);

  int attempts = 0;

  IPAddress local_IP(192, 168, 1, 187); // Set your desired static IP address
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
    // failed connection, start in AP mode
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
  // put your main code here, to run repeatedly:
  listenMMwave(ld2411Serial, mmWave); // Continuously listen to mmWave data
  static bool oldPresence = false;
  // Handle MQTT connection
  if (!mqttClient.connected()) {
    reconnect_mqtt(mqttClient, mqtt_client_id);
  }
  mqttClient.loop(); // process incoming messages and maintain connection
  if(!manualMode){
    if(mmWave.presenceDetected != oldPresence ){
      Serial.println("Turning the led " + String(mmWave.presenceDetected ? "ON" : "OFF"));
      if(mmWave.presenceDetected){
        myLedStrip.startFadeIn(); // turn on the led
      } else {
        myLedStrip.startFadeOut(); // turn off the led
      }
      publish_motion(mmWave.presenceDetected, mqttClient, mqtt_client_id, "home/gym/ld2411_motion"); 
      oldPresence = mmWave.presenceDetected;
    }
  }else{
    if(ledState != myLedStrip.state){
      Serial.println("Manual mode: Turning the led " + String(ledState ? "ON" : "OFF"));
      if(ledState){
        myLedStrip.startFadeIn(); // turn on the led
      } else {
        myLedStrip.startFadeOut(); // turn off the led
      }
    }


  }
  myLedStrip.manageLed(); // manage the led fading
  
}

