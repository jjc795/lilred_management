/*
 * Firmware for LilRed Management System
 * 
 * Written assuming Uno Board
 */

#include <Wire.h>
#include "ina226.h"
#include "ads7828.h"
#include <ros.h>
#include <lilred_msgs/Status.h>

#define DELAY 1000

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

//void estop_cb( const std_msgs::Bool& estop_msg) {
//  digitalWrite(estop_ctrl, estop_msg.data); // change estop by msg
//}

//void temp_cb( const std_msgs::Float32& temp_set_msg) {
//  temp_reading = temp_set_msg.data;
//}

//ros::Subscriber<std_msgs::Bool> estop_sub("estop", &estop_cb);

//ros::Subscriber<std_msgs::Float32> temp_sub("temp_set", &temp_cb);

// Need subscribers for fan, led, and estop controls

lilred_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

ina226 monitor_5V(BUS_5V_ADDR);
ina226 monitor_12V(BUS_12V_ADDR);
ina226 monitor_24V(BUS_24V_ADDR);
ads7828 adc(ADC_ADDR, 0, 5);

void setup() {
  nh.initNode();
  //nh.subscribe(estop_sub);
  //nh.subscribe(temp_sub);
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
}

void loop() {
  adc.config(CHANNEL_SEL_SINGLE_0);
  status_msg.temp1 = adc.getData(); // need to write a function to convert voltage to temp

  adc.config(CHANNEL_SEL_SINGLE_1);
  status_msg.temp2 = adc.getData();

  adc.config(CHANNEL_SEL_SINGLE_2);
  status_msg.temp3 = adc.getData();

  adc.config(CHANNEL_SEL_SINGLE_3);
  status_msg.temp4 = adc.getData();

  status_msg.current_5 = monitor_5V.getCurrent();
  status_msg.voltage_5 = monitor_5V.getBusVoltage();
  status_msg.power_5 = monitor_5V.getPower();

  status_msg.current_12 = monitor_12V.getCurrent();
  status_msg.voltage_12 = monitor_12V.getBusVoltage();
  status_msg.power_12 = monitor_12V.getPower();

  status_msg.current_24 = monitor_24V.getCurrent();
  status_msg.voltage_24 = monitor_24V.getBusVoltage();
  status_msg.power_24 = monitor_24V.getPower();

  // Do we also want to send over shunt voltage?
  // Need to do something about alert functionality

  //digitalWrite(estop_ctrl, /* corresponds to subscriber */);
  //analogWrite(led1_ctrl, /* some number */);
  //analogWrite(led2_ctrl, /* some number */);
  //analogWrite(fan_ctrl, /* some number */);

  status_pub.publish(&status_msg);
  
  nh.spinOnce();
  delay(DELAY);
}
