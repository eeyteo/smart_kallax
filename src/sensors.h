#ifndef SENSORS_H
#define SENSORS_H


#define TSL_ADDR 0x39

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h> 


uint16_t read16(uint8_t reg);
void scanI2C();
uint8_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint8_t value);


struct lightSensObj {
    String name;
    sensors_event_t lightEvent;
    bool isInitialized = false;
    uint16_t ch0;
    uint16_t ch1;
    
    lightSensObj() : name("") {}
    lightSensObj(String sensorName) : name(sensorName) {}


    void setup()
    {
        writeRegister(0x00, 0x03);  // power on
        delay(10);

        //writeRegister(0x01, 0x02);  // 402ms integration, 16x gain
        writeRegister(0x01,0x12); 
        isInitialized = true;
    }

    void debug(){
        Serial.println("TSL2561 raw test");

        uint8_t id = readRegister(0x0A);

        Serial.print("ID register = 0x");
        Serial.println(id, HEX);


        uint8_t control = readRegister(0x00);

        Serial.print("Control register = 0x");
        Serial.println(control, HEX);

        writeRegister(0x00, 0x03);

        delay(500);

        Serial.print("Control after ON = 0x");
        Serial.println(readRegister(0x00), HEX);


        Serial.println("TSL2561 test done");
    }

    void updateData() {
        // Read raw data from TSL2561 sensor
        ch0 = read16(0x0C);
        ch1 = read16(0x0E);
    }

    void printData(){
        Serial.print("CH0 = ");
        Serial.println(ch0);

        Serial.print("CH1 = ");
        Serial.println(ch1);
    }

    int getLightLevel() {
        if(ch0 == 0)
            return 0;

        float ratio = (float)ch1 / ch0;

        float lux;

        if(ratio <= 0.5)
        {
            lux = 0.0304 * ch0 -
                0.062 * ch0 * pow(ratio,1.4);
        }
        else if(ratio <= 0.61)
        {
            lux = 0.0224 * ch0 -
                0.031 * ch1;
        }
        else if(ratio <= 0.80)
        {
            lux = 0.0128 * ch0 -
                0.0153 * ch1;
        }
        else if(ratio <= 1.30)
        {
            lux = 0.00146 * ch0 -
                0.00112 * ch1;
        }
        else
        {
            lux = 0;
        }

        return max(lux, 0.0f);
    }

    void registryDump(){
        Serial.println("Register dump");
        for(uint8_t r = 0; r <= 0x0F; r++)
            {
                Serial.print("0x");
                Serial.print(r, HEX);
                Serial.print(" = 0x");
                Serial.println(readRegister(r), HEX);
            }
    }



};

struct tempSensObj {
    Adafruit_AHTX0 aht;
    String name;
    sensors_event_t humidity;
    sensors_event_t temp;
    bool isInitialized = false;
    
    tempSensObj() : name("") {}
    tempSensObj(String sensorName) : name(sensorName) {}
    
    void setup() {
        if (!aht.begin()) {
            Serial.printf("Failed to initialize AHT sensor (%s)!\n", name.c_str());
            isInitialized = false;
            return;
        }
        isInitialized = true;
        Serial.printf("Sensor %s initialized successfully\n", name.c_str());
    }

    void updateData() {
        if (!isInitialized) return;
        aht.getEvent(&humidity, &temp);
    }
    
    float getTemperature() {
        return temp.temperature;
    }
    
    float getHumidity() {
        return humidity.relative_humidity;
    }
};


#endif