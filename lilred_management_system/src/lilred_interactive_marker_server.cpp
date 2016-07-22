#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <lilred_msgs/Status.h>
#include <std_msgs/Bool.h>

#include <string>
#include <sstream>

using namespace visualization_msgs;

#define HZ 1

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

bool estop_status = false;

Marker makeText(InteractiveMarker &msg, bool isButton = false) {
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.z = msg.scale * 0.45;

  if (isButton) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.text = "ESTOP";
  }
  else {
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.text = "Test";
  }

  return marker;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM("text has been moved"); // want to add in what to do on user input

  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK) {
    ROS_INFO_STREAM("button click");
    estop_status = !estop_status;
  }

  server->applyChanges();
}

void makeInteractiveText(bool isButton = false) {
  // TODO: Keep estop marker attached to bottom of status marker

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.scale = 1;

  Marker marker;
  InteractiveMarkerControl control;

  if (isButton) {
    int_marker.name = "estop_marker";
    marker = makeText(int_marker, true);

    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.name = "text_button";
  }
  else {
    int_marker.name = "status_marker";
    marker = makeText(int_marker);

    control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
    control.name = "move_3d";
  }

  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void statusCallback(const lilred_msgs::Status &msg) {
  float temp1 = msg.temp1;
  float temp2 = msg.temp2;
  float temp3 = msg.temp3;
  float temp4 = msg.temp4;

  float current_24 = msg.current_24;
  float voltage_24 = msg.voltage_24;
  float power_24 = msg.power_24;

  float current_12 = msg.current_12;
  float voltage_12 = msg.voltage_12;
  float power_12 = msg.power_12;

  float current_5 = msg.current_5;
  float voltage_5 = msg.voltage_5;
  float power_5 = msg.power_5;

  InteractiveMarker int_marker;
  server->get("status_marker", int_marker);

  std::ostringstream status_str;
  status_str << "Temp 1: " << temp1 << "\n"
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
             << "5V Bus Power: " << power_5 << "\n";

  InteractiveMarkerControl text_control =  int_marker.controls.back();
  Marker text_marker = text_control.markers.back();

  int_marker.controls.clear();
  text_control.markers.clear();

  text_marker.text = status_str.str();

  text_control.markers.push_back(text_marker);
  int_marker.controls.push_back(text_control);

  server->insert(int_marker);
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

