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

#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"


int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::node::Node::make_shared("ros_bridge");

  // Bridge map topic
  std::string map_topic_name = "map";
  std::string ros1_map_type_name = "nav_msgs/OccupancyGrid";
  std::string ros2_map_type_name = "nav_msgs/OccupancyGrid";
  size_t queue_size = 50;

  auto map_handles = ros1_bridge::create_bridge_from_2_to_1(ros2_node, ros1_node, ros2_map_type_name, map_topic_name, queue_size, ros1_map_type_name, map_topic_name, 50);

  // Bridge tf topic
  std::string tf_topic_name = "tf";
  std::string ros1_tf_type_name = "tf2_msgs/TFMessage";
  std::string ros2_tf_type_name = "tf2_msgs/TFMessage";

  auto tf_handles = ros1_bridge::create_bridge_from_2_to_1(ros2_node, ros1_node, ros2_tf_type_name, tf_topic_name, queue_size, ros1_tf_type_name, tf_topic_name, 50);

  // Bridge tf_static topic
  std::string tf_static_topic_name = "tf_static";
  std::string ros1_tf_static_type_name = "tf2_msgs/TFMessage";
  std::string ros2_tf_static_type_name = "tf2_msgs/TFMessage";

  auto tf_static_handles = ros1_bridge::create_bridge_from_2_to_1(ros2_node, ros1_node, ros2_tf_static_type_name, tf_static_topic_name, queue_size, ros1_tf_static_type_name, tf_static_topic_name, 50);

  // TODO(clalancette): it would be nice to bridge the laser scan topic as well,
  // but it sometimes causes the turtlebot to lose its mind over wireless.
  // Leave it disabled for now.
  // Bridge scan topic
  //std::string scan_topic_name = "scan";
  //std::string ros1_scan_type_name = "sensor_msgs/LaserScan";
  //std::string ros2_scan_type_name = "sensor_msgs/LaserScan";

  //auto scan_handles = ros1_bridge::create_bidirectional_bridge(
  //  ros1_node, ros2_node, ros1_scan_type_name, ros2_scan_type_name, scan_topic_name, queue_size);
  //auto scan_handles = ros1_bridge::create_bridge_from_2_to_1(ros2_node, ros1_node, ros2_scan_type_name, scan_topic_name, queue_size, ros1_scan_type_name, scan_topic_name, 50);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::utilities::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
