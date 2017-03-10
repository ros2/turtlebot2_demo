// Copyright 2014 Open Source Robotics Foundation, Inc.
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
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("dumb_teleop");

  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rmw_qos_profile_default);

  rclcpp::WallRate loop_rate(20);
  int count = 0;
  const int switch_count = 80;

  auto msg = std::make_shared<geometry_msgs::msg::Twist>();
  msg->linear.x = 0.0;
  msg->angular.z = 0.5;

  while (rclcpp::ok()) {
    std::cout << "Publishing: (" << msg->linear.x << ", " << msg->angular.z << ")" << std::endl;
    cmd_vel_pub->publish(msg);
    if (++count >= switch_count) {
      msg->angular.z *= -1.0;
      count = 0;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
