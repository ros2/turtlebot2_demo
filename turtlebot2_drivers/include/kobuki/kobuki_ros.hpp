// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef KOBUKI__KOBUKI_ROS_HPP_
#define KOBUKI__KOBUKI_ROS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wignored-qualifiers"
# pragma GCC diagnostic ignored "-Wsign-compare"
# pragma GCC diagnostic ignored "-Wunused-local-typedefs"
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include <kobuki_driver/kobuki.hpp>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "kobuki/odometry.hpp"

namespace kobuki_ros
{
class KobukiRos {
public:
  KobukiRos(rclcpp::node::Node::SharedPtr & node);
  bool update();

private:
  kobuki::Kobuki kobuki;
  rclcpp::node::Node::SharedPtr node_;
  std::string odom_frame_;
  std::string gyro_link_frame_;
  std::string base_link_frame_;

  ecl::Slot<> slot_stream_data;
  void processStreamData();
  void publishRawInertia();
  void publishWheelState();
  rclcpp::publisher::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_data_publisher_;
  rcutils_time_point_value_t last_raw_imu_time_;

  std::shared_ptr<kobuki::Odometry> odometry;
  std::shared_ptr<tf2_ros::TransformBroadcaster> imu_broadcaster_;
  rclcpp::subscription::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_;
  void subscribeVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::mutex base_control_mutex_;
  bool cmd_vel_timed_out_;
};
}  // namespace kobuki

#endif  // KOBUKI__KOBUKI_ROS_HPP_
