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

#include <image_geometry/pinhole_camera_model.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

static rclcpp::publisher::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr g_pub_point_cloud;

static sensor_msgs::msg::CameraInfo::SharedPtr g_cam_info;

// TODO(clalancette): The DepthTraits structure and the convert<>() function are
// both copied from
// https://github.com/ros-perception/image_pipeline/tree/indigo/depth_image_proc .
// However, it is not clear that it is worthwhile to port depth_image_proc from
// ROS1, since it is heavily tied to ROS1 nodelets.  For now, we leave the
// copied code in here until we figure out what we want to do.

// Encapsulate differences between processing float and uint16_t depths
template<typename T>
struct DepthTraits {};

template<>
struct DepthTraits<uint16_t>
{
  static inline bool valid(uint16_t depth) {return depth != 0; }
  static inline float toMeters(uint16_t depth) {return depth * 0.001f; }  // originally mm
  static inline uint16_t fromMeters(float depth) {return (depth * 1000.0f) + 0.5f; }
  static inline void initializeBuffer(std::vector<uint8_t> &) {}  // Do nothing
};

template<>
struct DepthTraits<float>
{
  static inline bool valid(float depth) {return std::isfinite(depth); }
  static inline float toMeters(float depth) {return depth; }
  static inline float fromMeters(float depth) {return depth; }

  static inline void initializeBuffer(std::vector<uint8_t> & buffer)
  {
    float * start = reinterpret_cast<float *>(&buffer[0]);
    float * end = reinterpret_cast<float *>(&buffer[0] + buffer.size());
    std::fill(start, end, std::numeric_limits<float>::quiet_NaN());
  }
};

// Handles float or uint16 depths
template<typename T>
void convert(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
  const image_geometry::PinholeCameraModel & model,
  double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
      T depth = depth_row[u];

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = DepthTraits<T>::fromMeters(range_max);
        } else {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

static void depthCb(const sensor_msgs::msg::Image::SharedPtr image)
{
  // The meat of this function is a port of the code from:
  // https://github.com/ros-perception/image_pipeline/blob/indigo/depth_image_proc/src/nodelets/point_cloud_xyz.cpp

  if (g_cam_info == nullptr) {
    // we haven't gotten the camera info yet, so just drop until
    // we do.
    fprintf(stderr, "No camera info, skipping point cloud conversion\n");
    return;
  }
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header = image->header;
  cloud_msg->height = image->height;
  cloud_msg->width = image->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  cloud_msg->fields.clear();
  cloud_msg->fields.reserve(1);

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // g_cam_info here is a sensor_msg::msg::CameraInfo::ConstSharedPtr,
  // which we get from the cam_info topic.
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(g_cam_info);

  if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convert<uint16_t>(image, cloud_msg, model);
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convert<float>(image, cloud_msg, model);
  } else {
    fprintf(stderr, "Depth image has unsupported encoding [%s]\n", image->encoding.c_str());
    return;
  }

  g_pub_point_cloud->publish(cloud_msg);
}

void infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info)
{
  g_cam_info = info;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::node::Node::SharedPtr node = rclcpp::node::Node::make_shared("depth_to_pointcloud");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;

  custom_qos_profile.depth = 1;
  custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;

  g_pub_point_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("points2",
      custom_qos_profile);

  auto image_sub = node->create_subscription<sensor_msgs::msg::Image>("depth", depthCb,
      custom_qos_profile);
  auto cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info",
      infoCb, custom_qos_profile);

  rclcpp::spin(node);

  return 0;
}
