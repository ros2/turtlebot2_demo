/**
 * @file /ecl_linear_algebra/src/lib/sophus_helpers.cpp
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/linear_algebra.hpp>
#include "../../include/ecl/linear_algebra/sophus/helpers.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Sophus {

/*****************************************************************************
** Vanilla Implementation
*****************************************************************************/

Eigen::Vector3f toPose2D(const Sophus::SE3f& pose)
{
  Eigen::Vector3f pose2d;

  Eigen::Matrix3f R = pose.rotationMatrix();
  float yaw_angle = std::atan2( R(1,0), R(0,0) );
  pose2d.head<2>() = pose.translation().head<2>();
  pose2d.z() = yaw_angle;
  return pose2d;
}

Sophus::SE3f toPose3D(const Eigen::Vector3f& pose)
{
  Eigen::Matrix3f rotation_matrix = (
      Eigen::AngleAxis<float> (pose(2), Eigen::Vector3f::UnitZ ()) *
      Eigen::AngleAxis<float> (0.0f, Eigen::Vector3f::UnitY ()) *
      Eigen::AngleAxis<float> (0.0f, Eigen::Vector3f::UnitX ()) ).matrix();
  Eigen::Vector3f translation;
  translation << pose.x(), pose.y(), 0.0f;
  Sophus::SE3f pose3d(rotation_matrix, translation);

  return pose3d;
}

/*****************************************************************************
** C++11 Implementation
*****************************************************************************/

#if defined(ECL_CXX11_FOUND)
  Sophus::SE3fPtr points2DToSophusTransform(float from_x, float from_y, float to_x, float to_y) {
    // copied code from output receiver
    Eigen::Vector3f origin(from_x, from_y, 0.0);
    double angle = std::atan2(to_y-from_y, to_x-from_x);
    Eigen::Quaternion<float> q; q = Eigen::AngleAxis<float>(angle, Eigen::Vector3f::UnitZ());
    return std::make_shared<Sophus::SE3f>(q, origin);
  //  std::cout << "  Origin: " << origin.transpose() << std::endl;
  //  std::cout << "  Angle : " << angle << std::endl;
  }
#endif

} // namespace Sophus

/*****************************************************************************
** Converters
*****************************************************************************/

namespace ecl {

Sophus::SE3f Converter<Sophus::SE3f, Eigen::Vector3f>::operator()(const Eigen::Vector3f& pose) {
  return Sophus::toPose3D(pose);
}

Eigen::Vector3f Converter<Eigen::Vector3f, Sophus::SE3f>::operator()(const Sophus::SE3f& pose) {
  return Sophus::toPose2D(pose);
}

} // namespace ecl
