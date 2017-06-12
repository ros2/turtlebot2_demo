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

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wignored-qualifiers"
# pragma GCC diagnostic ignored "-Wsign-compare"
# pragma GCC diagnostic ignored "-Wunused-local-typedefs"
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif
#include "kobuki_driver/kobuki.hpp"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

static kobuki::Kobuki * g_kobuki;
static std::mutex g_kobuki_mutex;
static rcutils_time_point_value_t g_last_cmd_vel_time;
static double g_max_vx;
static double g_max_vyaw;

static void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> kobuki_guard(g_kobuki_mutex);
  double vx = std::min(std::max(msg->linear.x, -g_max_vx), g_max_vx);
  double vyaw = std::min(std::max(msg->angular.z, -g_max_vyaw), g_max_vyaw);
  g_kobuki->setBaseControl(vx, vyaw);
  if (rcutils_system_time_now(&g_last_cmd_vel_time) != RCUTILS_RET_OK) {
    std::cerr << "Failed to get system time" << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // TODO(clalancette): we set the depth to 50 here since it seems to help workaround
  // a bug where rclcpp::spin_some() can go into an infinite loop sometimes.  We should
  // find and fix the root cause instead of this workaround.
  rmw_qos_profile_t cmd_vel_qos_profile;
  cmd_vel_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  cmd_vel_qos_profile.depth = 50;
  cmd_vel_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  cmd_vel_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  rmw_qos_profile_t odom_and_imu_qos_profile;
  odom_and_imu_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  odom_and_imu_qos_profile.depth = 50;
  odom_and_imu_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  odom_and_imu_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  auto node = rclcpp::node::Node::make_shared("kobuki_node");
  auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", cmdVelCallback, cmd_vel_qos_profile);
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", odom_and_imu_qos_profile);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu", odom_and_imu_qos_profile);
  tf2_ros::TransformBroadcaster br(node);

  kobuki::Parameters parameters;
  parameters.device_port = "/dev/kobuki";
  node->get_parameter("device_port", parameters.device_port);
  std::string odom_frame = "odom";
  node->get_parameter("odom_frame", odom_frame);
  std::string gyro_link_frame = "gyro_link";
  node->get_parameter("gyro_link_frame", gyro_link_frame);
  std::string base_link_frame = "base_link";
  node->get_parameter("base_link_frame", base_link_frame);
  g_max_vx = 0.5;
  node->get_parameter("max_vx", g_max_vx);
  g_max_vyaw = 1.0;
  node->get_parameter("max_vyaw", g_max_vyaw);

  printf("device_port: %s\n", parameters.device_port.c_str());
  printf("max_vx: %f\n", g_max_vx);
  printf("max_vyaw: %f\n", g_max_vyaw);

  parameters.sigslots_namespace = "/kobuki";
  parameters.enable_acceleration_limiter = true;
  g_kobuki = new kobuki::Kobuki();
  g_kobuki->init(parameters);
  g_kobuki->enable();

  rclcpp::WallRate loop_rate(20);

  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = odom_frame;
  odom_msg->child_frame_id = base_link_frame;
  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  imu_msg->header.frame_id = gyro_link_frame;
  auto odom_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  odom_tf_msg->header.frame_id = odom_frame;
  odom_tf_msg->child_frame_id = base_link_frame;
  ecl::LegacyPose2D<double> pose;

  auto imu_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
  imu_tf_msg->header.frame_id = base_link_frame;
  imu_tf_msg->child_frame_id = gyro_link_frame;

  while (rclcpp::ok()) {
    rcutils_time_point_value_t now;
    double gyro_yaw, gyro_vyaw;
    ecl::LegacyPose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;
    {
      std::lock_guard<std::mutex> kobuki_guard(g_kobuki_mutex);
      g_kobuki->updateOdometry(pose_update, pose_update_rates);
      gyro_yaw = g_kobuki->getHeading();
      gyro_vyaw = g_kobuki->getAngularVelocity();
      if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
        std::cerr << "Failed to get system time" << std::endl;
      }
      if ((now - g_last_cmd_vel_time) > 200) {
        g_kobuki->setBaseControl(0.0, 0.0);
        g_last_cmd_vel_time = now;
      }
    }
    pose *= pose_update;

    // Stuff and publish /odom
    odom_msg->header.stamp.sec = RCL_NS_TO_S(now);
    odom_msg->header.stamp.nanosec = now - RCL_S_TO_NS(odom_msg->header.stamp.sec);
    odom_msg->pose.pose.position.x = pose.x();
    odom_msg->pose.pose.position.y = pose.y();
    odom_msg->pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.heading());
    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    for (unsigned int i = 0; i < odom_msg->pose.covariance.size(); ++i) {
      odom_msg->pose.covariance[i] = 0.0;
    }
    // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
    // Odometry yaw covariance must be much bigger than the covariance provided
    // by the imu, as the later takes much better measures
    odom_msg->pose.covariance[0] = 0.1;
    odom_msg->pose.covariance[7] = 0.1;
    // odom_msg->pose.covariance[35] = use_imu_heading ? 0.05 : 0.2;
    odom_msg->pose.covariance[35] = 0.2;

    odom_msg->pose.covariance[14] = DBL_MAX;  // set a non-zero covariance on unused
    odom_msg->pose.covariance[21] = DBL_MAX;  // dimensions (z, pitch and roll); this
    odom_msg->pose.covariance[28] = DBL_MAX;  // is a requirement of robot_pose_ekf

    odom_msg->twist.twist.linear.x = pose_update_rates[0];
    odom_msg->twist.twist.linear.y = pose_update_rates[1];
    odom_msg->twist.twist.linear.z = 0.0;
    odom_msg->twist.twist.angular.x = 0.0;
    odom_msg->twist.twist.angular.y = 0.0;
    odom_msg->twist.twist.angular.z = pose_update_rates[2];

    odom_pub->publish(odom_msg);

    // Stuff and publish /imu_data
    imu_msg->header.stamp = odom_msg->header.stamp;

    tf2::Quaternion q_imu;
    q_imu.setRPY(0.0, 0.0, gyro_yaw);
    imu_msg->orientation.x = q_imu.x();
    imu_msg->orientation.y = q_imu.y();
    imu_msg->orientation.z = q_imu.z();
    imu_msg->orientation.w = q_imu.w();

    // set a non-zero covariance on unused dimensions (pitch and roll); this is
    // a requirement of robot_pose_ekf set yaw covariance as very low, to make
    // it dominate over the odometry heading when combined 1: fill once, as its
    // always the same;  2: using an invented value; cannot we get a realistic
    // estimation?
    imu_msg->orientation_covariance[0] = DBL_MAX;
    imu_msg->orientation_covariance[4] = DBL_MAX;
    imu_msg->orientation_covariance[8] = 0.05;

    // fill angular velocity; we ignore acceleration for now
    imu_msg->angular_velocity.z = gyro_vyaw;

    // angular velocity covariance; useless by now, but robot_pose_ekf's
    // roadmap claims that it will compute velocities in the future
    imu_msg->angular_velocity_covariance[0] = DBL_MAX;
    imu_msg->angular_velocity_covariance[4] = DBL_MAX;
    imu_msg->angular_velocity_covariance[8] = 0.05;

    imu_msg->linear_acceleration.x = 0.0;
    imu_msg->linear_acceleration.y = 0.0;
    imu_msg->linear_acceleration.z = 9.8;

    imu_pub->publish(imu_msg);

    // Stuff and publish /tf
    odom_tf_msg->header.stamp = odom_msg->header.stamp;
    odom_tf_msg->transform.translation.x = pose.x();
    odom_tf_msg->transform.translation.y = pose.y();
    odom_tf_msg->transform.translation.z = 0.0;
    odom_tf_msg->transform.rotation.x = q.x();
    odom_tf_msg->transform.rotation.y = q.y();
    odom_tf_msg->transform.rotation.z = q.z();
    odom_tf_msg->transform.rotation.w = q.w();

    br.sendTransform(*odom_tf_msg);

    // Stuff and publish tf for the IMU; the values for XYZ come from:
    // https://github.com/yujinrobot/kobuki/blob/554e541/kobuki_description/urdf/kobuki.urdf.xacro
    // TODO(clalancette):  We publish the static transform here
    // because it is an intrinsic part of the robot, and thus it seems to
    // make sense to put it in the kobuki node.  Eventually this should
    // probably move out into the URDF.
    imu_tf_msg->header.stamp = imu_msg->header.stamp;
    imu_tf_msg->transform.translation.x = 0.056;
    imu_tf_msg->transform.translation.y = 0.062;
    imu_tf_msg->transform.translation.z = 0.0202;
    imu_tf_msg->transform.rotation.x = 0.0;
    imu_tf_msg->transform.rotation.y = 0.0;
    imu_tf_msg->transform.rotation.z = 0.0;
    imu_tf_msg->transform.rotation.w = 1.0;

    br.sendTransform(*imu_tf_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  g_kobuki->setBaseControl(0.0, 0.0);
  g_kobuki->disable();
  delete g_kobuki;

  return 0;
}
