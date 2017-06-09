#include "kobuki/odometry.hpp"

#include <limits>

using namespace kobuki;

Odometry::Odometry(rclcpp::node::Node::SharedPtr & node) : node_(node), use_imu_heading_(true), odom_frame_("odom"), base_frame_("base_link"), publish_tf_(true)
{
  double timeout_sec = 0.6;
  node_->get_parameter("odom_frame", odom_frame_);
  node_->get_parameter("base_frame", base_frame_);
  node_->get_parameter("publish_tf", publish_tf_);
  node_->get_parameter("use_imu_heading", use_imu_heading_);
  node_->get_parameter("cmd_vel_timeout", timeout_sec);

  // cmd_vel_timeout_ is in nanoseconds
  cmd_vel_timeout_ = RCUTILS_S_TO_NS(timeout_sec);

  pose_.setIdentity();

  rmw_qos_profile_t sensor_qos_profile;
  sensor_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sensor_qos_profile.depth = 50;
  sensor_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sensor_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", sensor_qos_profile);
}

void Odometry::update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
                      double imu_heading, double imu_angular_velocity)
{
  pose_ *= pose_update;

  if (use_imu_heading_ == true) {
    // Overwite with gyro heading data
    pose_.heading(imu_heading);
    pose_update_rates[2] = imu_angular_velocity;
  }

  geometry_msgs::msg::Quaternion odom_quat;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.heading());
  odom_quat.x = q.x();
  odom_quat.y = q.y();
  odom_quat.z = q.z();
  odom_quat.w = q.w();

  if (rclcpp::ok()) {
    publishTransform(odom_quat);
    publishOdometry(odom_quat, pose_update_rates);
  }
}

void Odometry::publishTransform(const geometry_msgs::msg::Quaternion &odom_quat)
{
  if (publish_tf_ == false) {
    return;
  }

  rcutils_time_point_value_t now;
  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
    std::cerr << "Failed to get system time" << std::endl;
    return;
  }

  geometry_msgs::msg::TransformStamped::SharedPtr odom_trans = std::make_shared<geometry_msgs::msg::TransformStamped>();
  odom_trans->header.frame_id = odom_frame_;
  odom_trans->child_frame_id = base_frame_;

  odom_trans->header.stamp.sec = RCUTILS_NS_TO_S(now);
  odom_trans->header.stamp.nanosec = now - RCUTILS_S_TO_NS(odom_trans->header.stamp.sec);
  odom_trans->transform.translation.x = pose_.x();
  odom_trans->transform.translation.y = pose_.y();
  odom_trans->transform.translation.z = 0.0;
  odom_trans->transform.rotation = odom_quat;
  odom_broadcaster_->sendTransform(*odom_trans);
}

void Odometry::publishOdometry(const geometry_msgs::msg::Quaternion &odom_quat,
                               const ecl::linear_algebra::Vector3d &pose_update_rates)
{
  nav_msgs::msg::Odometry::SharedPtr odom = std::make_shared<nav_msgs::msg::Odometry>();

  rcutils_time_point_value_t now;
  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
    std::cerr << "Failed to get system time" << std::endl;
    return;
  }

  // Header
  odom->header.stamp.sec = RCUTILS_NS_TO_S(now);
  odom->header.stamp.nanosec = now - RCUTILS_S_TO_NS(odom->header.stamp.sec);
  odom->header.frame_id = odom_frame_;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = pose_.x();
  odom->pose.pose.position.y = pose_.y();
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = odom_quat;

  // Velocity
  odom->twist.twist.linear.x = pose_update_rates[0];
  odom->twist.twist.linear.y = pose_update_rates[1];
  odom->twist.twist.angular.z = pose_update_rates[2];

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  // Odometry yaw covariance must be much bigger than the covariance provided
  // by the imu, as the later takes much better measures
  odom->pose.covariance[0]  = 0.1;
  odom->pose.covariance[1]  = 0.0;
  odom->pose.covariance[2]  = 0.0;
  odom->pose.covariance[3]  = 0.0;
  odom->pose.covariance[4]  = 0.0;
  odom->pose.covariance[5]  = 0.0;
  odom->pose.covariance[6]  = 0.0;
  odom->pose.covariance[7]  = 0.1;
  odom->pose.covariance[8]  = 0.0;
  odom->pose.covariance[9]  = 0.0;
  odom->pose.covariance[10]  = 0.0;
  odom->pose.covariance[11]  = 0.0;
  odom->pose.covariance[12]  = 0.0;
  odom->pose.covariance[13]  = 0.0;
  odom->pose.covariance[14] = std::numeric_limits<double>::max(); // set a non-zero covariance on unused
  odom->pose.covariance[15]  = 0.0;
  odom->pose.covariance[16]  = 0.0;
  odom->pose.covariance[17]  = 0.0;
  odom->pose.covariance[18]  = 0.0;
  odom->pose.covariance[19]  = 0.0;
  odom->pose.covariance[20]  = 0.0;
  odom->pose.covariance[21] = std::numeric_limits<double>::max(); // dimensions (z, pitch and roll); this
  odom->pose.covariance[22]  = 0.0;
  odom->pose.covariance[23]  = 0.0;
  odom->pose.covariance[24]  = 0.0;
  odom->pose.covariance[25]  = 0.0;
  odom->pose.covariance[26]  = 0.0;
  odom->pose.covariance[27]  = 0.0;
  odom->pose.covariance[28] = std::numeric_limits<double>::max(); // is a requirement of robot_pose_ekf
  odom->pose.covariance[29]  = 0.0;
  odom->pose.covariance[30]  = 0.0;
  odom->pose.covariance[31]  = 0.0;
  odom->pose.covariance[32]  = 0.0;
  odom->pose.covariance[33]  = 0.0;
  odom->pose.covariance[34]  = 0.0;
  odom->pose.covariance[35] = use_imu_heading_ ? 0.05 : 0.2;

  odom_publisher_->publish(odom);
}

void Odometry::resetTimeout()
{
  if (rcutils_system_time_now(&last_cmd_time_) != RCUTILS_RET_OK) {
    fprintf(stderr, "Failed to get system time\n");
  }
}

bool Odometry::commandTimeout() const
{
  rcutils_time_point_value_t now;
  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
    std::cerr << "Failed to get system time" << std::endl;
    return true;
  }

  //if ( (!last_cmd_time.isZero()) && ((ros::Time::now() - last_cmd_time) > cmd_vel_timeout) ) {
  if ((last_cmd_time_ != 0) && ((now - last_cmd_time_) > cmd_vel_timeout_)) {
    return true;
  } else {
    return false;
  }
}
