/*
 * lilred_interactive_marker_server.cpp
 * Jon Cruz
 *
 * Creates an interactive marker server that displays
 * data collected from LilRed's on-board management system
 */

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <lilred_msgs/Status.h>
#include <lilred_msgs/Command.h>

#include <dynamic_reconfigure/server.h>
#include <lilred_management_system/lilredConfig.h>

#include <string>
#include <sstream>

#include "lilred_management_system/thermistor.h"

#define HZ 1

#define STATUS_MARKER 0
#define ESTOP_MARKER  1
#define FAN_MARKER    2

#define YELLOW 1
#define GREEN  2
#define RED    3

#define ALL     0
#define MINIMAL 1

using namespace visualization_msgs;

// the base text to display
std::string status_text[] = {"Temp 1: ",
                             "Temp 2: ",
                             "Temp 3: ",
                             "Temp 4: ",
                             "24V Bus Current: ",
                             "24V Bus Voltage: ",
                             "24V Bus Power: ",
                             "12V Bus Current: ",
                             "12V Bus Voltage: ",
                             "12V Bus Power: ",
                             "5V Bus Current: ",
                             "5V Bus Voltage: ",
                             "5V Bus Power: ",
                             "Fan Setting: ",
                             "ESTOP: "};

// units to use
std::string status_units[] = {" F", " A", " V", " W"};

// thermistor resistances ordered from lower to higher temps
float resistances[] = {526240, 384520, 284010, 211940, 159720, 121490, 93246, 72181, 56332,
              44308, 35112, 28024, 22520, 18216, 14827, 12142, 10000, 8281.8, 6895.4,
              5770.3, 4852.5, 4100, 3479.8, 2966.3, 2539.2, 2182.4, 1883, 1630.7,
              1417.4, 1236.2, 1081.8, 949.73, 836.4, 738.81, 654.5, 581.44, 517.94,
              462.59, 414.2, 371.79, 334.51, 301.66, 272.64, 246.94, 224.14, 203.85,
              185.77, 169.61, 155.14, 142.16, 130.49, 119.99, 110.51, 101.94, 94.181,
              87.144, 80.751, 74.933, 69.631, 64.791, 60.366, 56.316, 52.602, 49.193,
              46.059, 43.173, 40.514, 38.06, 35.793, 33.696, 31.753, 29.952};

int resListLen = sizeof(resistances) / sizeof(resistances[0]);

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

bool estop_status;
bool estop_command;
int mode = ALL; // default mode

uint8_t fan_settings[] = {0, 64, 128, 191, 255}; // approx 0%, 25%, 50%, 75%, 100%
uint8_t *fan_set = fan_settings; // default is 0%

float voltageLowerLim_24V = 23.0;
float voltageUpperLim_24V = 30.0;
const float tempUpperLim = 302; // fahrenheit -- 150 celsius
const float tempLowerLim = -4;  // fahrenheit -- -20 celsius
const float currentLowerLim_24V = 30.0;
const float currentUpperLim_24V = 60.0;
const float powerLowerLim_24V = 1000;
const float powerUpperLim_24V = 1440;
const float currentLowerLim_12V = 3;
const float currentUpperLim_12V = 5;
const float voltageLowerLim_12V = 11.5;
const float voltageUpperLim_12V = 12.5;
const float powerLowerLim_12V = 35;
const float powerUpperLim_12V = 60;
const float currentLowerLim_5V = 3;
const float currentUpperLim_5V = 5;
const float voltageLowerLim_5V = 4.5;
const float voltageUpperLim_5V = 5.5;
const float powerLowerLim_5V = 15;
const float powerUpperLim_5V = 25;


/* Function Prototypes */
int findTextColor(float value, int text_id);
void updateText(visualization_msgs::Marker &marker, std::string text, int color);
Marker makeText(InteractiveMarker &msg, int text_id, int placement_id, bool isEstop = false);
void makeInteractiveText(int type);
void startDisplay();
void setFanText(InteractiveMarker &fan_marker);


/* Callback for user feedback from rviz */
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
    ROS_INFO_STREAM(feedback->marker_name << " has been clicked");

    if (feedback->marker_name == "estop_marker")
      estop_command = !estop_status;

    // cycle through fan settings
    else if (feedback->marker_name == "fan_marker") {
      if (fan_set >= fan_settings + sizeof(fan_settings) - sizeof(fan_settings[0]))
        fan_set = fan_settings;
      else
        fan_set += sizeof(uint8_t);

      // visually update fan marker text
      InteractiveMarker fan_marker;
      server->get("fan_marker", fan_marker);

      setFanText(fan_marker);
    }
  }
  else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                  << feedback->pose.position.y << ", " << feedback->pose.position.z);

    // keep other markers tied to status marker's pose
    if (feedback->marker_name == "status_marker") {
      InteractiveMarker estop_marker, fan_marker;
      server->get("estop_marker", estop_marker);
      server->get("fan_marker", fan_marker);

      estop_marker.pose = feedback->pose;
      fan_marker.pose = feedback->pose;

      if (mode == MINIMAL) {
        InteractiveMarker status_marker;
        server->get("status_marker", status_marker);

        InteractiveMarkerControl text_control = status_marker.controls.back();
        std::vector<Marker> text_markers = text_control.markers;
        status_marker.controls.clear();
        text_control.markers.clear();

        for (int i = 0; i < text_markers.size(); i++) {
          text_markers[i].pose = feedback->pose;
	        text_markers[i].pose.position.z -= text_markers[i].scale.z * 1.2 * (i + 2);
        }

        text_control.markers = text_markers;
        status_marker.controls.push_back(text_control);

        server->insert(status_marker);
      }
      server->insert(estop_marker);
      server->insert(fan_marker);
    }
  }
  server->applyChanges();
}


/* Callback to process status messages from uC board */
void statusCallback(const lilred_msgs::Status &msg) {
  float status[14];

  status[0] = msg.temp1;
  status[1] = msg.temp2;
  status[2] = msg.temp3;
  status[3] = msg.temp4;

  status[4] = msg.current_24;
  status[5] = msg.voltage_24;
  status[6] = msg.power_24;

  status[7] = msg.current_12;
  status[8] = msg.voltage_12;
  status[9] = msg.power_12;

  status[10] = msg.current_5;
  status[11] = msg.voltage_5;
  status[12] = msg.power_5;

  status[13] = msg.estop_status;
  estop_status = status[13];

  // convert raw thermistor outputs to temps
  thermistor thermistor(resistances, resListLen);
  thermistor.setResPullup(30000); // 30k pullup
  thermistor.setVcc(5); // 5 volts
  thermistor.fillRtTable(-55, 300, 5); // -55 to 300 celsius, increments of 5

  status[0] = thermistor.voltageToTemp(status[0]);
  status[1] = thermistor.voltageToTemp(status[1]);
  status[2] = thermistor.voltageToTemp(status[2]);
  status[3] = thermistor.voltageToTemp(status[3]);

  // update all the text with new data
  InteractiveMarker status_marker, estop_marker;
  server->get("status_marker", status_marker);
  server->get("estop_marker", estop_marker);

  std::ostringstream status_str[13];
  std::ostringstream estop_str;

  for (int i = 0; i < 13; i++) {
    status_str[i] << status_text[i];

    if (status[i] == ERROR_VALUE && i < 4)
      status_str[i] << "OUT OF RANGE";
    else if (i < 4)
      status_str[i] << status[i] << status_units[0];
    else
      status_str[i] << status[i] << status_units[(i+2) % 3 + 1];
  }

  if (estop_status)
    estop_str << status_text[14] << "ON";
  else
    estop_str << status_text[14] << "OFF";

  InteractiveMarkerControl text_control =  status_marker.controls.back();
  std::vector<Marker> text_markers = text_control.markers;
  InteractiveMarkerControl button_control = estop_marker.controls.back();
  Marker button_marker = button_control.markers.back();

  status_marker.controls.clear();
  text_control.markers.clear();
  estop_marker.controls.clear();
  button_control.markers.clear();

  if (mode == ALL) {
    for (int i = 0; i < 13; i++)
      updateText(text_markers[i], status_str[i].str(), findTextColor(status[i], i));
  }
  else if (mode == MINIMAL) {
    InteractiveMarker fan_marker;
    server->get("fan_marker", fan_marker);
    InteractiveMarkerControl fan_control = fan_marker.controls.back();
    fan_marker.controls.clear();
    fan_control.markers.clear();

    text_markers.clear();
    int placement_id = 0;

    for (int i = 0; i < 13; i++) {
      if (findTextColor(status[i], i) != GREEN || i == 5) {
        Marker marker = makeText(status_marker, i, placement_id);
        updateText(marker, status_str[i].str(), findTextColor(status[i], i));
        text_markers.push_back(marker);
        placement_id++;
      }
    }
    Marker fan_button_marker = makeText(fan_marker, 13, placement_id);
    fan_control.markers.push_back(fan_button_marker);
    fan_marker.controls.push_back(fan_control);
    setFanText(fan_marker);

    button_marker = makeText(estop_marker, 14, placement_id+1, true);
  }
  button_marker.text = estop_str.str();

  text_control.markers = text_markers;
  status_marker.controls.push_back(text_control);
  button_control.markers.push_back(button_marker);
  estop_marker.controls.push_back(button_control);

  server->insert(status_marker);
  server->insert(estop_marker);
  server->applyChanges();
}


/* Callback for dynamic reconfigure changes */
void cfgCallback(lilred_management_system::lilredConfig &config, uint32_t level) {
  voltageLowerLim_24V = config.BusVoltageLowerLim_24V;
  voltageUpperLim_24V = config.BusVoltageUpperLim_24V;
  mode = config.Mode;
  std::string mode_str;

  if (mode)
    mode_str = "Minimal";
  else
    mode_str = "All";

  ROS_INFO_STREAM("Mode set to " << mode_str << std::endl << "24V Bus Voltage Lower Limit set to " << voltageLowerLim_24V << ", Upper Limit set to " << voltageUpperLim_24V);

  server->clear();
  server->applyChanges();
  startDisplay();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "status_marker_server");

  server.reset(new interactive_markers::InteractiveMarkerServer("status_marker_server","",false));

  dynamic_reconfigure::Server<lilred_management_system::lilredConfig> cfgServer;
  dynamic_reconfigure::Server<lilred_management_system::lilredConfig>::CallbackType f;

  f = boost::bind(&cfgCallback, _1, _2);
  cfgServer.setCallback(f);

  ros::NodeHandle nh;

  ros::Subscriber status_sub = nh.subscribe("status", 1000, statusCallback);

  ros::Publisher command_pub = nh.advertise<lilred_msgs::Command>("commands", 1000);

  ros::Rate loop_rate(HZ);

  startDisplay();

  // publish the command message at specified rate
  while (ros::ok()) {
    lilred_msgs::Command msg;
    msg.estop_command = estop_command;
    msg.fan_set = *fan_set;
    msg.header.stamp = ros::Time::now();

    command_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  server.reset();
}


/* Starts display with current settings */
void startDisplay() {
  makeInteractiveText(STATUS_MARKER);
  makeInteractiveText(ESTOP_MARKER);
  makeInteractiveText(FAN_MARKER);

  server->applyChanges();

  // start fan text with 0%
  InteractiveMarker fan_marker;
  server->get("fan_marker", fan_marker);

  setFanText(fan_marker);

  server->applyChanges();
}


/* Set fan marker text to match the fan setting */
void setFanText(InteractiveMarker &fan_marker) {
  std::ostringstream fan_str;
  fan_str << status_text[13];

  if (*fan_set == fan_settings[0])
    fan_str << "0%";
  else if (*fan_set == fan_settings[1])
    fan_str << "25%";
  else if (*fan_set == fan_settings[2])
    fan_str << "50%";
  else if (*fan_set == fan_settings[3])
    fan_str << "75%";
  else if (*fan_set == fan_settings[4])
    fan_str << "100%";
  else
    fan_str << "ERROR";

  InteractiveMarkerControl button_control = fan_marker.controls.back();
  Marker button_marker = button_control.markers.back();

  fan_marker.controls.clear();
  button_control.markers.clear();

  button_marker.text = fan_str.str();

  button_control.markers.push_back(button_marker);
  fan_marker.controls.push_back(button_control);

  server->insert(fan_marker);
}


/* Creates basic text markers for interactive markers */
Marker makeText(InteractiveMarker &msg, int text_id, int placement_id, bool isEstop) {
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.z = msg.scale * 0.45;
  marker.text = status_text[text_id];
  marker.pose.position.x = msg.pose.position.x;
  marker.pose.position.y = msg.pose.position.y;

  marker.pose.position.z = msg.pose.position.z - marker.scale.z * 1.2 * placement_id; // space them out vertically

  if (isEstop) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  else {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }

  marker.color.a = 1.0;
  return marker;
}


/* Create an interactive text marker of the specified type */
void makeInteractiveText(int type) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.scale = 1;

  int_marker.pose.position.x = int_marker.pose.position.y = int_marker.pose.position.z = 0.0;

  InteractiveMarkerControl control;

  switch (type) {
    case STATUS_MARKER: { // marker to display status data
                          int_marker.name = "status_marker";

                          if (mode == ALL) {
                            Marker marker[13];
                            for (int i = 0; i < 13; i++) {
                              marker[i] = makeText(int_marker, i, i);
                              control.markers.push_back(marker[i]);
                            }
                          }
                          else if (mode == MINIMAL) {
                            Marker marker = makeText(int_marker, 5, 0);
                            control.markers.push_back(marker);
                          }

                          control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
                          control.name = "move_3d";
                          break;
                        }
    case ESTOP_MARKER:  { // marker for estop interaction
                          int_marker.name = "estop_marker";
			  Marker marker;
			  if (mode == ALL)
                            marker = makeText(int_marker, 14, 14, true);
			  else if (mode == MINIMAL)
			    marker = makeText(int_marker, 14, 2, true);

                          control.interaction_mode = InteractiveMarkerControl::BUTTON;
                          control.name = "text_button";
                          control.markers.push_back(marker);
                          break;
                        }
    case FAN_MARKER:    { // marker for fan interaction
                          int_marker.name = "fan_marker";
			  Marker marker;
			  if (mode == ALL)
			    marker = makeText(int_marker, 13, 13);
			  else if (mode == MINIMAL)
                            marker = makeText(int_marker, 13, 1);

                          control.interaction_mode = InteractiveMarkerControl::BUTTON;
                          control.name = "text_button";
                          control.markers.push_back(marker);
                          break;
                        }
  }
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}


/* Update text markers dynamically to reflect new data */
void updateText(visualization_msgs::Marker &marker, std::string text, int color) {
  marker.text = text;

  switch(color) {
    case GREEN:  { marker.color.r = 0.0;
                   marker.color.g = 1.0;
                   marker.color.b = 0.0;
                   break;
		 }
    case YELLOW: { marker.color.r = 0.5;
                   marker.color.g = 0.5;
                   marker.color.b = 0.0;
                   break;
		 }
    case RED:    { marker.color.r = 1.0;
                   marker.color.g = 0.0;
                   marker.color.b = 0.0;
                   break;
		 }
  }
}


/* Determine which color to make the text based on pre-determined ranges */
int findTextColor(float value, int text_id) {
  // temp data
  if (text_id >= 0 && text_id <= 3) {
    if (value >= tempUpperLim || value < tempLowerLim)
      return RED;
    else if (value <= tempUpperLim - 90  && value >= tempLowerLim + 35)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V current
  else if (text_id == 4) {
    if (value >= currentUpperLim_24V || value < 0)
      return RED;
    else if (value <= currentLowerLim_24V)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V bus voltage
  else if (text_id == 5) {
    if (value <= voltageLowerLim_24V || value >= voltageUpperLim_24V)
      return RED;
    else if (value >= voltageLowerLim_24V + 1 && value <= voltageUpperLim_24V - 1)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V power
  else if (text_id == 6) {
    if (value >= powerUpperLim_24V || value < 0)
      return RED;
    else if (value <= powerLowerLim_24V)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V current
  else if (text_id == 7) {
    if (value >= currentUpperLim_12V || value < 0)
      return RED;
    else if (value <= currentLowerLim_12V)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V bus voltage
  else if (text_id == 8) {
    if (value <= voltageLowerLim_12V || value >= voltageUpperLim_12V)
      return RED;
    else if (value >= voltageLowerLim_12V + 0.3 && value <= voltageUpperLim_12V - 0.3)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V power
  else if (text_id == 9) {
    if (value >= powerUpperLim_12V || value < 0)
      return RED;
    else if (value <= powerLowerLim_12V)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V current
  else if (text_id == 10) {
    if (value >= currentUpperLim_5V || value < 0)
      return RED;
    else if (value <= currentLowerLim_5V)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V bus voltage
  else if (text_id == 11) {
    if (value <= voltageLowerLim_5V || value >= voltageUpperLim_5V)
      return RED;
    else if (value >= voltageLowerLim_5V + 0.3 && value <= voltageUpperLim_5V - 0.3)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V power
  else if (text_id == 12) {
    if (value >= powerUpperLim_5V || value < 0)
      return RED;
    else if (value <= powerLowerLim_5V)
      return GREEN;
    else
      return YELLOW;
  }
}
