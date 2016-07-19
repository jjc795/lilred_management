/* Utility Functions */

#include "Arduino.h"
#include <Wire.h>

uint16_t readRegister(uint8_t slave_addr, uint8_t reg_addr) {
  uint16_t reading = 0;
  
  Wire.beginTransmission(slave_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();
  
  Wire.requestFrom(slave_addr, 2);

  if (2 <= Wire.available()) {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
  }

  return reading;
}

void writeRegister(uint8_t slave_addr, uint8_t reg_addr, uint16_t val) {
  Wire.beginTransmission(slave_addr);
  Wire.write(reg_addr);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

uint16_t readData(uint8_t slave_addr) {
  uint16_t data = 0;

  Wire.requestFrom(slave_addr, 2);

  if (2 <= Wire.available()) {
    data = Wire.read();
    data = data << 8;
    data |= Wire.read();
  }

  return data;
}

void writeByte(uint8_t slave_addr, uint8_t val) {
  Wire.beginTransmission(slave_addr);
  Wire.write(val);
  Wire.endTransmission();
}

