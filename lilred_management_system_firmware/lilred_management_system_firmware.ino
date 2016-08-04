/*
 * lilred_management_system_firmware.ino
 * Firmware for LilRed Management System
 * Jon Cruz
 *
 * Written assuming Uno Board
 *
 * Note: if SRAM usage is high reduce publisher/subscriber buffer size in ros.h
 * Used publisher and subscriber buffer size of 100 bytes
 */

#include <Wire.h>
#include "ina226.h"
#include "ads7828.h"
#include <ros.h>
#include <lilred_msgs/Status.h>
#include <lilred_msgs/Command.h>

#define BAUD 57600
#define DELAY 1000
#define TIMEOUT 3000

/* Device I2C Addresses */

#define BUS_24V_ADDR   0x40
#define BUS_12V_ADDR   0x41
#define BUS_5V_ADDR    0x44

#define ADC_ADDR       0x48

ros::NodeHandle nh;

/* Pin constants */

const int estop_ctrl = 7; // PD7
const int fan_ctrl = 5;   // PD5
const int led1_ctrl = 6;  // PD6
const int led2_ctrl = 9;  // PB1
const int alert_24 = 2;   // PD2
const int alert_12 = 4;   // PD4
const int alert_5 = A1;   // PC1
const int estop_mont = 3; // PD3

boolean estop_status = false;
uint8_t fan_set = 0;
unsigned long timePrevSub = 0;

/* Callback for receiving command messages */
void command_cb( const lilred_msgs::Command& command_msg) {
  timePrevSub = millis(); // time last message received
  fan_set = command_msg.fan_set;
  boolean estop_command = command_msg.estop_command;

  digitalWrite(estop_ctrl, estop_command); // change estop by msg
  
  if (digitalRead(estop_mont))
    estop_status = estop_command;
  else if (!digitalRead(estop_mont))
    estop_status = true;
}

ros::Subscriber<lilred_msgs::Command> command_sub("commands", &command_cb);

lilred_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

ina226 monitor_5V(BUS_5V_ADDR);
ina226 monitor_12V(BUS_12V_ADDR);
ina226 monitor_24V(BUS_24V_ADDR);
ads7828 adc(ADC_ADDR, 0, 5);

void setup() {
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(command_sub);
  nh.advertise(status_pub);

  Wire.begin();
  
  monitor_5V.begin(CALIBRATION_5V_5A, CONFIG_AVG_4, CONFIG_VBUSCT_1100US, CONFIG_VSHCT_1100US, CONFIG_MODE_VSH_VBUS_CONTINUOUS);
  monitor_12V.begin(CALIBRATION_12V_5A, CONFIG_AVG_4, CONFIG_VBUSCT_1100US, CONFIG_VSHCT_1100US, CONFIG_MODE_VSH_VBUS_CONTINUOUS);
  monitor_24V.begin(CALIBRATION_24V_60A, CONFIG_AVG_4, CONFIG_VBUSCT_1100US, CONFIG_VSHCT_1100US, CONFIG_MODE_VSH_VBUS_CONTINUOUS);

  pinMode(estop_ctrl, OUTPUT);
  pinMode(fan_ctrl, OUTPUT);
  pinMode(led1_ctrl, OUTPUT);
  pinMode(led2_ctrl, OUTPUT);
  
  pinMode(alert_24, INPUT);
  pinMode(alert_12, INPUT);
  pinMode(alert_5, INPUT);
  pinMode(estop_mont, INPUT);
}

void loop() {
  digitalWrite(led2_ctrl, estop_status);  // indicator for estop
  analogWrite(fan_ctrl, fan_set);         // control fan

  if (millis() - timePrevSub < TIMEOUT) 
    digitalWrite(led1_ctrl, HIGH);        // indicator for ros working
  else {
    digitalWrite(led1_ctrl, LOW);         // ros not working
    estop_status = true;                  // halt operation
  }
  
  adc.config(CHANNEL_SEL_SINGLE_0);
  status_msg.temp1 = adc.getData(); // Celsius

  adc.config(CHANNEL_SEL_SINGLE_1);
  status_msg.temp2 = adc.getData();

  adc.config(CHANNEL_SEL_SINGLE_2);
  status_msg.temp3 = adc.getData();

  adc.config(CHANNEL_SEL_SINGLE_3);
  status_msg.temp4 = adc.getData();

  status_msg.current_5 = monitor_5V.getCurrent();      // Amps
  status_msg.voltage_5 = monitor_5V.getBusVoltage();   // Volts
  status_msg.power_5 = monitor_5V.getPower();          // Watts

  status_msg.current_12 = monitor_12V.getCurrent();    // Amps
  status_msg.voltage_12 = monitor_12V.getBusVoltage(); // Volts
  status_msg.power_12 = monitor_12V.getPower();        // Watts

  status_msg.current_24 = monitor_24V.getCurrent();    // Amps
  status_msg.voltage_24 = monitor_24V.getBusVoltage(); // Volts
  status_msg.power_24 = monitor_24V.getPower();        // Watts

  status_msg.estop_status = estop_status;

  status_msg.header.stamp = nh.now();

  // Need to do something about alert functionality

  status_pub.publish(&status_msg);
  
  nh.spinOnce();
  delay(DELAY);
}
