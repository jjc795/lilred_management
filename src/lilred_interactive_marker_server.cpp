#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <lilred_msgs/Status.h>
#include <std_msgs/Bool.h>

#include <string>
#include <sstream>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

float temp1;
float temp2;
float temp3;
float temp4;

Marker makeText(InteractiveMarker &msg) {
  Marker marker;

  marker.type = Marker::TEXT_VIEW_FACING;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  marker.text = "Test";

  return marker;
}

InteractiveMarkerControl& makeTextControl(InteractiveMarker &msg) {
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  control.markers.push_back(makeText(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM("text has been moved"); // want to add in what to do on user input
  server->applyChanges();
}

void makeInteractiveText(void) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.scale = 1;
  int_marker.name = "status_marker";

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MOVE_3D;
  control.name = "move_3d";

  Marker marker = makeText(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void statusCallback(const lilred_msgs::Status &msg) {
  temp1 = msg.temp1;
  temp2 = msg.temp2;
  temp3 = msg.temp3;
  temp4 = msg.temp4;

  InteractiveMarker int_marker;
  server->get("status_marker", int_marker);

  std::ostringstream oss;
  oss << "Temp1: " << temp1 << "\n" << "Temp2: " << temp2 << "\n" << "Temp3: " << temp3 << "\n" << "Temp4: " << temp4;

  InteractiveMarkerControl text_control =  int_marker.controls.back();
  Marker text_marker = text_control.markers.back();
  int_marker.controls.pop_back();
  text_control.markers.pop_back();
  text_marker.text = oss.str();
  text_control.markers.push_back(text_marker);
  int_marker.controls.push_back(text_control);

  server->insert(int_marker);
  server->applyChanges();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "status_marker");

  ros::NodeHandle nh;

  ros::Subscriber status_sub = nh.subscribe("status", 1000, statusCallback);

  server.reset(new interactive_markers::InteractiveMarkerServer("status_marker","",false));

 // char start_text[5] = { "Test" };
  makeInteractiveText();

//  std::ostringstream oss;
//  oss << "Temp1: " << temp1 << "\n" << "Temp2: " << temp2 << "\n" << "Temp3: " << temp3 << "\n" << "Temp4: " << temp4;
//  text_marker.text = oss.str();

  server->applyChanges();

  ros::spin();

  server.reset();
}
