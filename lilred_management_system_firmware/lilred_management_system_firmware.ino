/*
 * Firmware for LilRed Management System
 * 
 * Written assuming Uno Board
 */

#include <Wire.h>
#include "ina226.h"
#include "ads7828.h"
#include "util.h"
#include <ros.h>
#include <lilred_msgs/Status.h>
#include <std_msgs/Float32.h>

#define DELAY 1000

/* Device I2C Addresses */

#define BUS_24V_ADDR   0x40
#define BUS_12V_ADDR   0x41
#define BUS_5V_ADDR    0x44

#define ADC_ADDR       0x48

ros::NodeHandle nh;

const int estop_ctrl = 7;

float to_send = 0;
int counter = 0;

//void estop_cb( const std_msgs::Bool& estop_msg) {
//  digitalWrite(estop_ctrl, estop_msg.data); // change estop by msg
//}

//void temp_cb( const std_msgs::Float32& temp_set_msg) {
//  temp_reading = temp_set_msg.data;
//}

//ros::Subscriber<std_msgs::Bool> estop_sub("estop", &estop_cb);

//ros::Subscriber<std_msgs::Float32> temp_sub("temp_set", &temp_cb);

lilred_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

std_msgs::Float32 test_msg;
ros::Publisher test_pub("test", &test_msg);

ina226 monitor_5V(BUS_5V_ADDR);
ads7828 adc(ADC_ADDR, 0, 5, 12);

void setup() {
  nh.initNode();
  //nh.subscribe(estop_sub);
  //nh.subscribe(temp_sub);
  //nh.advertise(debug);
  //nh.advertise(temp_pub);
  nh.advertise(status_pub);
  nh.advertise(test_pub);

  Wire.begin();
  monitor_5V.begin(CALIBRATION_5V_5A, CONFIG_AVG_4, CONFIG_VBUSCT_1100US, CONFIG_VSHCT_1100US, CONFIG_MODE_VSH_VBUS_CONTINUOUS);

  pinMode(estop_ctrl, OUTPUT);
}

void loop() {

  switch(counter) {
    case 0:
      //to_send = monitor_5V.getBusVoltage();
      adc.config(CHANNEL_SEL_SINGLE_0);
      to_send = adc.getData();
      break;
    case 1:
      //to_send = monitor_5V.getShuntVoltage();
      adc.config(CHANNEL_SEL_SINGLE_1);
      to_send = adc.getData();
      break;
    case 2:
      //to_send = monitor_5V.getCurrent();
      adc.config(CHANNEL_SEL_SINGLE_2);
      to_send = adc.getData();
      break;
    case 3:
      //to_send = monitor_5V.getPower();
      adc.config(CHANNEL_SEL_SINGLE_3);
      to_send = adc.getData();
      break;
  }
  
  test_msg.data = to_send;

  status_msg.temp1 = 150.43;
  status_msg.temp2 = 34.55;
  status_msg.temp3 = 77.77;

  test_pub.publish(&test_msg);
  status_pub.publish(&status_msg);
  
  nh.spinOnce();
  counter = (counter + 1) % 4;
  delay(DELAY);
}
