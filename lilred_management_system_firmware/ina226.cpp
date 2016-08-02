/* Driver for INA226 Current/Power Monitor */

#include "Arduino.h"
#include <Wire.h>

#include "ina226.h"
#include "util.h"

/* Instantiate new ina226 class */
ina226::ina226(uint8_t addr) {
  slave_addr = addr;
  currentLSB = 0;
  powerLSB = 0;
  shuntVoltageLSB = 2.5; // uV
  busVoltageLSB = 1.25;  // mV
}

/* Initialize hardware settings */
void ina226::begin(uint16_t cal, uint16_t avg, uint16_t vbusct, uint16_t vshct, uint16_t mode) {
  calValue = cal;

  if (calValue == CALIBRATION_5V_5A) {
    currentLSB = .2;            // mA
    powerLSB = 25 * currentLSB; // mW
  }
  else if (calValue == CALIBRATION_12V_5A) {
    currentLSB = .2;            // mA
    powerLSB = 25 * currentLSB; // mW
  }
  else {
    currentLSB = 1;             // mA
    powerLSB = 25 * currentLSB; // mW
  }
  
  writeRegister(slave_addr, CALIBRATION_REG, calValue);

  uint16_t config = 0x4000 |
                    avg |
                    vbusct |
                    vshct |
                    mode;
  writeRegister(slave_addr, CONFIG_REG, config);
}

/* Configure the alert settings */
void ina226::alertConfig(uint16_t func, uint16_t limit, uint16_t polarity, uint16_t len) {
  uint16_t config = (func |
                    polarity |
                    len) &
                    MASK_ENABLE_MASK;
  writeRegister(slave_addr, MASK_ENABLE_REG, config);

  writeRegister(slave_addr, ALERT_LIMIT_REG, limit);
}

/* Get raw bus voltage (bytes) */
int16_t ina226::getBusVoltageRaw(void) {
  uint16_t value = readRegister(slave_addr, BUS_VOLTAGE_REG);
  return (int16_t)value;
}

/* Get raw shunt voltage (bytes) */
int16_t ina226::getShuntVoltageRaw(void) {
  uint16_t value = readRegister(slave_addr, SHUNT_VOLTAGE_REG);
  return (int16_t)value;
}

/* Get raw current (bytes) */
int16_t ina226::getCurrentRaw(void) {
  writeRegister(slave_addr, CALIBRATION_REG, calValue);
  
  uint16_t value = readRegister(slave_addr, CURRENT_REG);
  return (int16_t)value;
}

/* Get raw power (bytes) */
int16_t ina226::getPowerRaw(void) {
  writeRegister(slave_addr, CALIBRATION_REG, calValue);

  uint16_t value = readRegister(slave_addr, POWER_REG);
  return (int16_t)value;
}

/* Get bus voltage in V */
float ina226::getBusVoltage(void) {
  int16_t value = getBusVoltageRaw();
  return (value * busVoltageLSB) / 1000.0;
}

/* Get shunt voltage in uV */
float ina226::getShuntVoltage(void) {
  int16_t value = getShuntVoltageRaw();
  return value * shuntVoltageLSB;
}

/* Get current in A */
float ina226::getCurrent(void) {
  int16_t value = getCurrentRaw();
  return (value * currentLSB) / 1000.0;
}

/* Get power in W */
float ina226::getPower(void) {
  int16_t value = getPowerRaw();
  return (value * powerLSB) / 1000.0;
}

