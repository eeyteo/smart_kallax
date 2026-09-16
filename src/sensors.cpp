#include "sensors.h"

void scanI2C() {
    Serial.println("Scanning I2C...");

    byte count = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("Found device at 0x%02X\n", address);
            count++;
        }
    }

    Serial.printf("Found %d devices\n", count);
}



uint8_t readRegister(uint8_t reg)
{
    Wire.beginTransmission(0x39);

    Wire.write(0x80 | reg);   // command bit + register

    uint8_t err = Wire.endTransmission(false);
    if(err){
        Serial.print("error write=");
        Serial.println(err);
    }

    Wire.requestFrom(0x39, (uint8_t)1);

    if (Wire.available())
        return Wire.read();

    return 0xFF;
}


void writeRegister(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(0x39);
    Wire.write(0x80 | reg);
    Wire.write(value);

    uint8_t result = Wire.endTransmission();

    Serial.print("write=");
    Serial.println(result);
}


uint16_t read16(uint8_t reg)
{
    uint16_t low = readRegister(reg);
    uint16_t high = readRegister(reg + 1);

    return (high << 8) | low;
}