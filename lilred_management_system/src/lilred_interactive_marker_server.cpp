#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <lilred_msgs/Status.h>
#include <std_msgs/Bool.h>

#include <string>
#include <sstream>

#define HZ 1

using namespace visualization_msgs;

const char status_text[14][] = { "Temp 1: ",
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
                               "5V Bus Power: " }

const char estop_text[8] = "ESTOP: ";

//const float thermResponse 

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

bool estop_status = false;

Marker makeText(InteractiveMarker &msg, char* text, bool isButton = false) {
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.z = msg.scale * 0.45;
  marker.text = text;

  if (isButton) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    //marker.text = text
  }
  else {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    //marker.text = text;
  }

  return marker;
}

float voltageToTemp(float voltage) {
  float resPullup = 30000; // pullup resistor value in ohms
  float vcc = 5; // volts

  float resTherm = resPullup * (voltage / (vcc - voltage)); // thermistor resistance in ohms
  return resToTemp(resTherm); // temperature in degree celsius
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
    ROS_INFO_STREAM("button click");
    estop_status = !estop_status;
  }
  else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
    ROS_INFO_STREAM(feedback->marker_name << " is now at " << feedback->pose.position.x << ", "
                  << feedback->pose.position.y << ", " << feedback->pose.position.z);

  server->applyChanges();
}

void makeInteractiveText(bool isButton = false) {
  // TODO: Keep estop marker attached to bottom of status marker

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.scale = 1;

  InteractiveMarkerControl control;

  if (isButton) {
    int_marker.name = "estop_marker";
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;
    Marker marker = makeText(int_marker, estop_text, true);

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "text_button";
    control.markers.push_back(marker);
  }
  else {
    int_marker.name = "status_marker";
    int_marker.pose.position.x = 0.0;
    int_marker.pose.position.y = 0.0;
    int_marker.pose.position.z = 0.0;

    Marker marker[14];
    for (int i = 0; i < 14; i++) {
      marker[i] = makeText(int_marker, status_text[i]);
      control.markers.push_back(marker[i])
    }

    control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    control.name = "move_3d";
  }

  //control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

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

  InteractiveMarker status_marker, estop_marker;
  server->get("status_marker", status_marker);
  server->get("estop_marker", estop_marker);

  std::ostringstream status_str[14];
  std::ostringstream estop_str;

  for (int i = 0; i < 14; i++)
    status_str[i] << status_text[i] << status[i];

  /*status_str << "Temp 1: " << temp1 << "\n"
             << "Temp 2: " << temp2 << "\n"
             << "Temp 3: " << temp3 << "\n"
             << "Temp 4: " << temp4 << "\n"
             << "24V Bus Current: " << current_24 << "\n"
             << "24V Bus Voltage: " << voltage_24 << "\n"
             << "24V Bus Power: " << power_24 << "\n"
             << "12V Bus Current: " << current_12 << "\n"
             << "12V Bus Voltage: " << voltage_12 << "\n"
             << "12V Bus Power: " << power_12 << "\n"
             << "5V Bus Current: " << current_5 << "\n"
             << "5V Bus Voltage: " << voltage_5 << "\n"
             << "5V Bus Power: " << power_5 << std::endl;*/

  if (estop_status)
    estop_str << estop_text << "ON";
  else
    estop_str << estop_text << "OFF";

  InteractiveMarkerControl text_control =  status_marker.controls.back();
  Marker* text_markers = text_control.markers;
  InteractiveMarkerControl button_control = estop_marker.controls.back();
  Marker button_marker = button_control.markers.back();

  status_marker.controls.clear();
  text_control.markers.clear(); // this might cause an issue
  estop_marker.controls.clear();
  button_control.markers.clear();

  for (int i = 0; i < 14; i++) {
    text_markers[i] = status_str[i].str();
    text_control.markers.push_back(text_markers[i]);
  }

  button_marker.text = estop_str.str();

  //text_control.markers.push_back(text_marker);
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

  ros::Publisher estop_pub = nh.advertise<std_msgs::Bool>("estop", 1000);

  ros::Rate loop_rate(HZ);

  server.reset(new interactive_markers::InteractiveMarkerServer("status_marker_server","",false));

  makeInteractiveText();
  makeInteractiveText(true);

  server->applyChanges();

  while (ros::ok()) {
    std_msgs::Bool msg;
    msg.data = estop_status;

    estop_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  server.reset();
}

