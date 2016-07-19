/* Driver for the ADS7828 A/D converter */

#include "Arduino.h"
#include <Wire.h>

#include "ads7828.h"
#include "util.h"

/* Configure ADC instance with important attributes */
void ads7828::setProps(float low, float high) {
  bottomRange = low;
  topRange = high;
  valueLSB = (topRange - bottomRange) / (float)(1 << 12);
}

/* Instantiate new ads7828 class */
ads7828::ads7828(uint8_t addr, float low, float high) {
  slave_addr = addr;
  setProps(low, high);
}

/* Set command byte for conversion */
void ads7828::config(uint8_t channel_sel, uint8_t powerdown = PWRDWN_SEL_REF_OFF_CNVR_ON) {
  command = channel_sel |
            powerdown;
}

/* Get raw data */
uint16_t ads7828::getDataRaw(void) {
  writeByte(slave_addr, command);
  uint16_t data = readData(slave_addr);
  return data;
}

/* Get data in terms of voltage */
float ads7828::getData(void) {
  float data = getDataRaw();
  return data * valueLSB;
}

