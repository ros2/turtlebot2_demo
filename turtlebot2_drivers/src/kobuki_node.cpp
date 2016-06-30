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

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> kobuki_guard(g_kobuki_mutex);
  std::cout << "Received: (" << msg->linear.x << "," << msg->angular.z << ")" << std::endl;
  g_kobuki->setBaseControl(msg->linear.x, msg->angular.z);
  g_last_cmd_vel_time = std::chrono::system_clock::now();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("kobuki_node");
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);
  auto parameters_client = std::make_shared<rclcpp::parameter_client::SyncParametersClient>(node);

  kobuki::Parameters parameters;
  parameters.device_port = "/dev/kobuki";
  for (auto & parameter : parameters_client->get_parameters({"device_port"})) {
    parameters.device_port = parameter.value_to_string();
  }
  printf("device_port: %s\n", parameters.device_port.c_str());

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

    std::cout << "Publishing: (" <<
      odom_msg->pose.pose.position.x << ", " <<
      odom_msg->pose.pose.position.y << ", " <<
      odom_msg->pose.pose.orientation.w << ")" << std::endl;
    odom_pub->publish(odom_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  g_kobuki->setBaseControl(0.0, 0.0);
  g_kobuki->disable();
  delete g_kobuki;

  return 0;
}

