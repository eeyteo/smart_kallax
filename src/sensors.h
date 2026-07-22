#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h> 
#include <Adafruit_TSL2561_U.h>

struct lightSensObj{
    Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(123); // Initialize with sensor ID
    String name;
    sensors_event_t lightEvent;
    bool isInitialized = false;
    
    // Default constructor - initializes name to empty string
    lightSensObj() : name("") {
        // tsl is already initialized with sensor ID 12345
    }
    
    // Constructor with name parameter
    lightSensObj(String sensorName) : name(sensorName) {
        // tsl is already initialized with sensor ID 12345
    }
    
    void setup(){
        // Initialize the sensor
        if (!tsl.begin()) {
            Serial.println("Failed to initialize TSL2561 sensor!");
            return;
        }
        Serial.printf("Sensor %s initialized successfully\n", name.c_str());
    }

    void updateData() {
        // Get the light reading
        tsl.getEvent(&lightEvent);
    }
    
    // Optional: Helper method to get light reading
    float getLightLevel() {
        return lightEvent.light;
    }
};


struct tempSensObj{
    Adafruit_AHTX0 aht;
    String name;
    sensors_event_t humidity;
    sensors_event_t temp;
    
    // Default constructor - initializes name to empty string
    tempSensObj() : name("") {
        // Adafruit_AHTX0 has its own default constructor
    }
    
    // Constructor with name parameter
    tempSensObj(String sensorName) : name(sensorName) {
        // sensors_event_t structures are automatically zero-initialized
    }
    
    void setup(){
        // Initialize the sensor
        if (!aht.begin()) {
            Serial.println("Failed to initialize AHT sensor!");
            return;
        }
        Serial.printf("Sensor %s initialized successfully\n", name.c_str());
    }

    void updateData() {
        // Get the humidity and temperature readings
        aht.getEvent(&humidity, &temp);
    }
    
    // Optional: Helper methods to get readings
    float getTemperature() {
        return temp.temperature;
    }
    
    float getHumidity() {
        return humidity.relative_humidity;
    }
};

#endif