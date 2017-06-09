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

#ifndef KOBUKI__ODOMETRY_HPP_
#define KOBUKI__ODOMETRY_HPP_

#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wignored-qualifiers"
# pragma GCC diagnostic ignored "-Wsign-compare"
# pragma GCC diagnostic ignored "-Wunused-local-typedefs"
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include <ecl/geometry/legacy_pose2d.hpp>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

namespace kobuki
{
class Odometry
{
public:
  Odometry(rclcpp::node::Node::SharedPtr & node);
  void update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
              double imu_heading, double imu_angular_velocity);
  void resetTimeout();
  bool commandTimeout() const;

private:
  rclcpp::node::Node::SharedPtr node_;
  bool use_imu_heading_;
  ecl::LegacyPose2D<double> pose_;
  rclcpp::publisher::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  void publishOdometry(const geometry_msgs::msg::Quaternion &odom_quat,
                       const ecl::linear_algebra::Vector3d &pose_update_rates);
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  void publishTransform(const geometry_msgs::msg::Quaternion &odom_quat);
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  rcutils_time_point_value_t last_cmd_time_;
  rcutils_time_point_value_t cmd_vel_timeout_;
};
}  // namespace kobuki

#endif  // KOBUKI__ODOMETRY_HPP_
