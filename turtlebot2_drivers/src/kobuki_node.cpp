// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "kobuki_driver/kobuki.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

kobuki::Kobuki* g_kobuki;
std::mutex g_kobuki_mutex;
std::chrono::system_clock::time_point g_last_cmd_vel_time;
double g_max_vx;
double g_max_vyaw;

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> kobuki_guard(g_kobuki_mutex);
  std::cout << "Received: (" << msg->linear.x << "," << msg->angular.z << ")" << std::endl;
  double vx = std::min(std::max(msg->linear.x, -g_max_vx), g_max_vx);
  double vyaw = std::min(std::max(msg->angular.z, -g_max_vyaw), g_max_vyaw);
  g_kobuki->setBaseControl(vx, vyaw);
  g_last_cmd_vel_time = std::chrono::system_clock::now();
}

void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // TODO: handle dynamic params
  std::cout << "Parameter event:" << std::endl << " new parameters:" << std::endl;
  for (auto & new_parameter : event->new_parameters) {
    std::cout << "  " << new_parameter.name << std::endl;
  }
  std::cout << " changed parameters:" << std::endl;
  for (auto & changed_parameter : event->changed_parameters) {
    std::cout << "  " << changed_parameter.name << std::endl;
  }
  std::cout << " deleted parameters:" << std::endl;
  for (auto & deleted_parameter : event->deleted_parameters) {
    std::cout << "  " << deleted_parameter.name << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("kobuki_node");
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);
  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);
  auto sub = parameters_client->on_parameter_event(on_parameter_event);

  kobuki::Parameters parameters;
  parameters.device_port = "/dev/kobuki";
  g_max_vx = 0.5;
  g_max_vyaw = 1.0;
  // TODO use to-be-written params API to get a single value, with a default
  for (auto & parameter : parameters_client->get_parameters({"device_port"})) {
    parameters.device_port = parameter.as_string();
  }
  for (auto & parameter : parameters_client->get_parameters({"max_vx"})) {
    g_max_vx = parameter.as_double();
  }
  for (auto & parameter : parameters_client->get_parameters({"max_vyaw"})) {
    g_max_vyaw = parameter.as_double();
  }

  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::parameter::ParameterVariant("device_port", parameters.device_port),
    rclcpp::parameter::ParameterVariant("max_vx", g_max_vx),
    rclcpp::parameter::ParameterVariant("max_vyaw", g_max_vyaw)});
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      fprintf(stderr, "Failed to set parameter: %s\n", result.reason.c_str());
    }
  }

  printf("device_port: %s\n", parameters.device_port.c_str());
  printf("max_vx: %f\n", g_max_vx);
  printf("max_vyaw: %f\n", g_max_vyaw);

  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = "/dev/kobuki";
  parameters.enable_acceleration_limiter = true;
  g_kobuki = new kobuki::Kobuki();
  g_kobuki->init(parameters);
  g_kobuki->enable();

  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", cmdVelCallback, rmw_qos_profile_default);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", rmw_qos_profile_default);

  rclcpp::WallRate loop_rate(20);

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  ecl::Pose2D<double> pose;

  while (rclcpp::ok()) {
    ecl::Pose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    {
      std::lock_guard<std::mutex> kobuki_guard(g_kobuki_mutex);
      g_kobuki->updateOdometry(pose_update, pose_update_rates);
      auto now = std::chrono::system_clock::now(); 
      if ((now - g_last_cmd_vel_time) > std::chrono::milliseconds(200)) {
        g_kobuki->setBaseControl(0.0, 0.0);
        g_last_cmd_vel_time = now;
      }
    }
    pose *= pose_update;

    odom_msg->pose.pose.position.x = pose.x();
    odom_msg->pose.pose.position.y = pose.y();
    odom_msg->pose.pose.position.z = 0.0;
 
    // TODO: do Euler->quaternion conversion
    odom_msg->pose.pose.orientation.x = 0.0;
    odom_msg->pose.pose.orientation.y = 0.0;
    odom_msg->pose.pose.orientation.z = 0.0;
    odom_msg->pose.pose.orientation.w = pose.heading();

    //std::cout << "Publishing: (" <<
      //odom_msg->pose.pose.position.x << ", " <<
      //odom_msg->pose.pose.position.y << ", " <<
      //odom_msg->pose.pose.orientation.w << ")" << std::endl;
    odom_pub->publish(odom_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  g_kobuki->setBaseControl(0.0, 0.0);
  g_kobuki->disable();
  delete g_kobuki;

  return 0;
}

