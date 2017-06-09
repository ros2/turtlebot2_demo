#include <rclcpp/rclcpp.hpp>

#include "kobuki/kobuki_ros.hpp"

using namespace kobuki_ros;
using namespace kobuki;

static inline double from_degrees(double degrees)
{
  return degrees * M_PI / 180.0;
}

KobukiRos::KobukiRos(rclcpp::node::Node::SharedPtr & node) : node_(node), odom_frame_("odom"), gyro_link_frame_("gyro_link"), base_link_frame_("base_link"), slot_stream_data(&KobukiRos::processStreamData, *this), last_raw_imu_time_(0), cmd_vel_timed_out_(false)
{
  kobuki::Parameters parameters;

  parameters.device_port = "/dev/kobuki";
  node_->get_parameter("device_port", parameters.device_port);
  node_->get_parameter("odom_frame", odom_frame_);
  node_->get_parameter("gyro_link_frame", gyro_link_frame_);
  node_->get_parameter("base_link_frame", base_link_frame_);

  printf("device_port: %s\n", parameters.device_port.c_str());

  parameters.sigslots_namespace = "/kobuki";
  parameters.enable_acceleration_limiter = true;

  kobuki.init(parameters);
  kobuki.enable();

  rmw_qos_profile_t sensor_qos_profile;
  sensor_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sensor_qos_profile.depth = 50;
  sensor_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sensor_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  raw_imu_data_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu_raw", sensor_qos_profile);

  odometry = std::make_shared<kobuki::Odometry>(node_);

  imu_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  rmw_qos_profile_t cmd_vel_qos_profile;
  cmd_vel_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  cmd_vel_qos_profile.depth = 50;
  cmd_vel_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  cmd_vel_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

  velocity_command_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", std::bind(&KobukiRos::subscribeVelocityCommand, this, std::placeholders::_1), cmd_vel_qos_profile);

  slot_stream_data.connect("/kobuki" + std::string("/stream_data"));
}

void KobukiRos::subscribeVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(base_control_mutex_);
  if (kobuki.isEnabled()) {
    kobuki.setBaseControl(msg->linear.x, msg->angular.z);
    odometry->resetTimeout();
  }
}

void KobukiRos::processStreamData()
{
  publishRawInertia();
  publishWheelState();
}

void KobukiRos::publishRawInertia()
{
  if (!rclcpp::ok()) {
    return;
  }

  rcutils_time_point_value_t now;

  if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
    std::cerr << "Failed to get system time" << std::endl;
    return;
  }

  auto imu_tf_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();

  // Stuff and publish tf for the IMU; the values for XYZ come from:
  // https://github.com/yujinrobot/kobuki/blob/devel/kobuki_description/urdf/kobuki.urdf.xacro
  // TODO(clalancette):  We publish the static transform here
  // because it is an intrinsic part of the robot, and thus it seems to
  // make sense to put it in the kobuki node.  Eventually this should
  // probably move out into the URDF.
  imu_tf_msg->header.stamp.sec = RCUTILS_NS_TO_S(now);
  imu_tf_msg->header.stamp.nanosec = now - RCUTILS_S_TO_NS(imu_tf_msg->header.stamp.sec);
  imu_tf_msg->header.frame_id = base_link_frame_;
  imu_tf_msg->child_frame_id = gyro_link_frame_;
  imu_tf_msg->transform.translation.x = 0.056;
  imu_tf_msg->transform.translation.y = 0.062;
  imu_tf_msg->transform.translation.z = 0.0202;
  imu_tf_msg->transform.rotation.x = 0.0;
  imu_tf_msg->transform.rotation.y = 0.0;
  imu_tf_msg->transform.rotation.z = 0.0;
  imu_tf_msg->transform.rotation.w = 1.0;

  imu_broadcaster_->sendTransform(*imu_tf_msg);

  if (node_->count_subscribers("imu_raw") <= 0) {
    return;
  }

  sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>();
  ThreeAxisGyro::Data data = kobuki.getRawInertiaData();

  const double digit_to_dps = 0.00875;  // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
  unsigned int length = data.followed_data_length / 3;
  for( unsigned int i=0; i<length; i++) {
    // Each sensor reading has id, that circulate 0 to 255.
    msg->header.frame_id = "gyro_link";

    // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
    msg->angular_velocity.x = from_degrees(-digit_to_dps * (short)data.data[i*3+1]);
    msg->angular_velocity.y = from_degrees(digit_to_dps * (short)data.data[i*3+0]);
    msg->angular_velocity.z = from_degrees(digit_to_dps * (short)data.data[i*3+2]);

    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation.w = 1.0;

    msg->orientation_covariance[0] = 0.0;
    msg->orientation_covariance[1] = 0.0;
    msg->orientation_covariance[2] = 0.0;
    msg->orientation_covariance[3] = 0.0;
    msg->orientation_covariance[4] = 0.0;
    msg->orientation_covariance[5] = 0.0;
    msg->orientation_covariance[6] = 0.0;
    msg->orientation_covariance[7] = 0.0;
    msg->orientation_covariance[8] = 0.0;

    msg->angular_velocity_covariance[0] = 0.0;
    msg->angular_velocity_covariance[1] = 0.0;
    msg->angular_velocity_covariance[2] = 0.0;
    msg->angular_velocity_covariance[3] = 0.0;
    msg->angular_velocity_covariance[4] = 0.0;
    msg->angular_velocity_covariance[5] = 0.0;
    msg->angular_velocity_covariance[6] = 0.0;
    msg->angular_velocity_covariance[7] = 0.0;
    msg->angular_velocity_covariance[8] = 0.0;

    msg->linear_acceleration.x = 0.0;
    msg->linear_acceleration.y = 0.0;
    msg->linear_acceleration.z = 9.8;

    // Update rate of 3d gyro sensor is 100 Hz, but robot's update rate is 50 Hz.
    // So, here is some compensation.
    // See also https://github.com/yujinrobot/kobuki/issues/216
    rcutils_time_point_value_t fixed = now - RCUTILS_MS_TO_NS(10) * length-i-1;
    if (fixed >= last_raw_imu_time_) {
      msg->header.stamp.sec = RCUTILS_NS_TO_S(fixed);
      msg->header.stamp.nanosec = fixed - RCUTILS_S_TO_NS(msg->header.stamp.sec);
      raw_imu_data_publisher_->publish(msg);
    }
    last_raw_imu_time_ = fixed;
  }
}

void KobukiRos::publishWheelState()
{
  // Take latest encoders and gyro data
  ecl::LegacyPose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);

  // Update and publish odometry and joint states
  odometry->update(pose_update, pose_update_rates, kobuki.getHeading(), kobuki.getAngularVelocity());
}

bool KobukiRos::update()
{
  std::lock_guard<std::mutex> guard(base_control_mutex_);

  if (kobuki.isShutdown()) {
    return false;
  }

  if (kobuki.isEnabled() && odometry->commandTimeout()) {
    if (!cmd_vel_timed_out_) {
      kobuki.setBaseControl(0, 0);
      cmd_vel_timed_out_ = true;
    }
  } else {
    cmd_vel_timed_out_ = false;
  }

  return true;
}
