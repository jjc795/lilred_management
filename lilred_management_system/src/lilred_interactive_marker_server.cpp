#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <lilred_msgs/Status.h>
#include <lilred_msgs/Command.h>

#include <string>
#include <sstream>

#include "lilred_management_system/thermistor.h"

#define HZ 1

#define STATUS_MARKER 0
#define ESTOP_MARKER  1
#define FAN_MARKER    2

#define YELLOW 3
#define GREEN  4
#define RED    5

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
std::string status_units[] = {" C", " A", " V", " W"};

// thermistor resistances ordered from lower to higher temps
float resistances[] = {526240, 384520, 284010, 211940, 159720, 121490, 93246, 72181, 56332,
              44308, 35112, 28024, 22520, 18216, 14827, 12142, 10000, 8281.8, 6895.4,
              5770.3, 4852.5, 4100, 3479.8, 2966.3, 2539.2, 2182.4, 1883, 1630.7,
              1417.4, 1236.2, 1081.8, 949.73, 836.4, 738.81, 654.5, 581.44, 517.94,
              462.59, 414.2, 371.79, 334.51, 301.66, 272.64, 246.94, 224.14, 203.85,
              185.77, 169.61, 155.14, 142.16, 130.49, 119.99, 110.51, 101.94, 94.181,
              87.144, 80.751, 74.933, 69.631, 64.791, 60.366, 56.316, 52.602, 49.193,
              46.059, 43.173, 40.514, 38.06, 35.793, 33.696, 31.753, 29.952};

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

bool estop_status = false; // default is not triggered
bool prev_server_estop = false;
bool prev_client_estop = false;

uint8_t fan_settings[] = {0, 64, 128, 191, 255}; // approx 0%, 25%, 50%, 75%, 100%
uint8_t *fan_set = fan_settings; // default is 0%

/* Creates basic text markers for interactive markers */
Marker makeText(InteractiveMarker &msg, int text_id, bool isRed = false) {
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.z = msg.scale * 0.45;
  marker.text = status_text[text_id];
  marker.pose.position.x = msg.pose.position.x;
  marker.pose.position.y = msg.pose.position.y;
  marker.pose.position.z = msg.pose.position.z - marker.scale.z * 1.2 * text_id;

  if (isRed) {
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

/* Callback for user feedback from rviz */
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
    ROS_INFO_STREAM(feedback->marker_name << " has been clicked");

    if (feedback->marker_name == "estop_marker")
      estop_status = !estop_status;
    // cycle through fan settings
    else if (feedback->marker_name == "fan_marker") {
      if (fan_set >= fan_settings + sizeof(fan_settings) - sizeof(fan_settings[0]))
        fan_set = fan_settings;
      else
        fan_set += sizeof(uint8_t);

      // visually update fan marker text
      InteractiveMarker fan_marker;
      server->get("fan_marker", fan_marker);

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
  }
  else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    ROS_INFO_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                  << feedback->pose.position.y << ", " << feedback->pose.position.z);

    // keep other markers tied to status marker's pose
    if (feedback->marker_name == "status_marker") {
      InteractiveMarker estop_marker, fan_marker;
      server->get("estop_marker", estop_marker);
      server->get("fan_marker", fan_marker);

      estop_marker.pose.position.x = feedback->pose.position.x;
      estop_marker.pose.position.y = feedback->pose.position.y;
      estop_marker.pose.position.z = feedback->pose.position.z;

      fan_marker.pose.position.x = feedback->pose.position.x;
      fan_marker.pose.position.y = feedback->pose.position.y;
      fan_marker.pose.position.z = feedback->pose.position.z;

      server->insert(estop_marker);
      server->insert(fan_marker);
    }
  }

  server->applyChanges();
}

/* Create an interactive text marker */
void makeInteractiveText(int type) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.scale = 1;

  int_marker.pose.position.x = 0.0;
  int_marker.pose.position.y = 0.0;
  int_marker.pose.position.z = 0.0;

  InteractiveMarkerControl control;

  switch (type) {
    case STATUS_MARKER: { int_marker.name = "status_marker";

                          Marker marker[13];
                          for (int i = 0; i < 13; i++) {
                            marker[i] = makeText(int_marker, i);
                            control.markers.push_back(marker[i]);
                          }

                          control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
                          control.name = "move_3d"; 
                          break;
                        }
    case ESTOP_MARKER:  { int_marker.name = "estop_marker";
                          Marker marker = makeText(int_marker, 14, true);

                          control.interaction_mode = InteractiveMarkerControl::BUTTON;
                          control.name = "text_button";
                          control.markers.push_back(marker);
                          break;
                        }
    case FAN_MARKER:    { int_marker.name = "fan_marker";
                          Marker marker = makeText(int_marker, 13);

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

/* Update text markers to reflect new data */
void updateText(visualization_msgs::Marker &marker, std::string text, int color) {
  marker.text = text;

  switch(color) {
    case GREEN:   marker.color.r = 0.0;
                  marker.color.g = 1.0;
                  marker.color.b = 0.0;
                  break;

    case YELLOW:  marker.color.r = 0.5;
                  marker.color.g = 0.5;
                  marker.color.b = 0.0;
                  break;

    case RED:     marker.color.r = 1.0;
                  marker.color.g = 0.0;
                  marker.color.b = 0.0;
                  break;
  }
}

/* Determine which color to make the text based on pre-determined ranges */
int findTextColor(float value, int text_id) {
  // temp data
  if (text_id >= 0 || text_id <= 3) {
    if (value > 150.0)
      return RED;
    else if (value < 100.0)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V current
  else if (text_id == 4) {
    if (value > 60.0)
      return RED;
    else if (value < 30.0)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V bus voltage
  else if (text_id == 5) {
    if (value < 23.0)
      return RED;
    else if (value > 24.0)
      return GREEN;
    else
      return YELLOW;
  }
  // 24V power
  else if (text_id == 6) {
    if (value > 1440)
      return RED;
    else if (value < 1000)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V current
  else if (text_id == 7) {
    if (value > 5)
      return RED;
    else if (value < 3)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V bus voltage
  else if (text_id == 8) {
    if (value < 11.5)
      return RED;
    else if (value > 12.0)
      return GREEN;
    else
      return YELLOW;
  }
  // 12V power
  else if (text_id == 9) {
    if (value > 60)
      return RED;
    else if (value < 35)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V current
  else if (text_id == 10) {
    if (value > 5)
      return RED;
    else if (value < 3)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V bus voltage
  else if (text_id == 11) {
    if (value < 4.5)
      return RED;
    else if (value > 4.9)
      return GREEN;
    else
      return YELLOW;
  }
  // 5V power
  else if (text_id == 12) {
    if (value > 25)
      return RED;
    else if (value < 15)
      return GREEN;
    else
      return YELLOW;
  }
}

/* Callback to process status messages from client */
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

  // avoid a race condition -- trigger server estop if client's been triggered
  // otherwise any differences are settled by making the client estop the same as the server
  if (!estop_status && status[13]) {
    if (!prev_server_estop && !prev_client_estop) {
      estop_status = true;
    }
  }
  prev_client_estop = status[13];

  // convert raw thermistor outputs to temps
  thermistor thermistor(resistances);
  thermistor.setResPullup(30000);
  thermistor.setVcc(5);
  thermistor.fillRtTable(-55, 300, 5);

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

  for (int i = 0; i < 13; i++)
    updateText(text_markers[i], status_str[i].str(), findTextColor(status[i], i));

  button_marker.text = estop_str.str();

  text_control.markers = text_markers;
  status_marker.controls.push_back(text_control);
  button_control.markers.push_back(button_marker);
  estop_marker.controls.push_back(button_control);

  server->insert(status_marker);
  server->insert(estop_marker);
  server->applyChanges();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "status_marker_server");

  ros::NodeHandle nh;

  ros::Subscriber status_sub = nh.subscribe("status", 1000, statusCallback);

  ros::Publisher command_pub = nh.advertise<lilred_msgs::Command>("commands", 1000);

  ros::Rate loop_rate(HZ);

  server.reset(new interactive_markers::InteractiveMarkerServer("status_marker_server","",false));

  makeInteractiveText(STATUS_MARKER);
  makeInteractiveText(ESTOP_MARKER);
  makeInteractiveText(FAN_MARKER);

  server->applyChanges();

  // start fan text with 0%
  InteractiveMarker fan_marker;
  server->get("fan_marker", fan_marker);

  std::ostringstream fan_str;
  fan_str << status_text[13] << "0%";

  InteractiveMarkerControl button_control = fan_marker.controls.back();
  Marker button_marker = button_control.markers.back();

  fan_marker.controls.clear();
  button_control.markers.clear();

  button_marker.text = fan_str.str();

  button_control.markers.push_back(button_marker);
  fan_marker.controls.push_back(button_control);

  server->insert(fan_marker);

  server->applyChanges();

  // publish the command message
  while (ros::ok()) {
    lilred_msgs::Command msg;
    msg.estop_status = estop_status;
    msg.fan_ctrl = *fan_set;
    msg.header.stamp = ros::Time::now();

    command_pub.publish(msg);

    prev_server_estop = estop_status;

    ros::spinOnce();

    loop_rate.sleep();
  }

  server.reset();
}

