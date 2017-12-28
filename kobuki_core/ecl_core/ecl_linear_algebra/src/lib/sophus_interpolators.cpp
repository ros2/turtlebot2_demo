/**
 * @file /ecl_linear_algebra/src/sophus_interpolators.cpp
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/linear_algebra.hpp>
#include "../../include/ecl/linear_algebra/sophus/interpolators.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Sophus {

/*****************************************************************************
** Planar Interpolator
*****************************************************************************/

PlanarInterpolator::PlanarInterpolator(const Sophus::SE3f& T_a, const Sophus::SE3f& T_b) throw(ecl::StandardException)
: T_a(T_a)
{
  double epsilon = 0.00001;
  Sophus::SE3f T_b_rel_a = T_b*T_a.inverse();
  Sophus::SE3f pose_b_rel_a = T_b_rel_a.inverse();
  Eigen::Vector3f translation = pose_b_rel_a.translation();
  ecl_assert_throw( std::abs(translation.z()) < epsilon, ecl::StandardException(LOC, ecl::InvalidArgError, "Input transforms are not coplanar."));

  // TODO check the angle axis is the z-axis

  float axis_angle = T_b_rel_a.inverse().so3().log()(2);
  Sophus::SE2f t_b_rel_a = Sophus::SE2f(axis_angle, translation.head<2>()).inverse();
  tangent = Sophus::SE2f::log(t_b_rel_a);
}
Sophus::SE3f PlanarInterpolator::operator()(const double& t) {
  Sophus::SE2f t_t_rel_a = Sophus::SE2f::exp(t*tangent);
  float angle = t_t_rel_a.inverse().so2().log();
  Eigen::Vector3f translation;
  translation.head<2>() = t_t_rel_a.inverse().translation();
  translation(2) = 0.0;
  Eigen::Matrix3f R = Eigen::AngleAxis<float> (angle, Eigen::Vector3f::UnitZ ()).matrix();
  Sophus::SE3f T_t_rel_a = Sophus::SE3f(R, translation).inverse();
  return T_t_rel_a*T_a;
}

/*****************************************************************************
** Sliding Interpolator
*****************************************************************************/

SlidingInterpolator::SlidingInterpolator(const Sophus::SE3f& T_a, const Sophus::SE3f& T_b)
: interpolator(T_a, T_b), T_a(T_a),  T_b(T_b)
{
}

Sophus::SE3f SlidingInterpolator::operator()(const double& t) {
  Sophus::SE3f T_b_rel_a = T_b*T_a.inverse();
  Eigen::Vector3f translation_a = T_a.inverse().translation();
  Eigen::Vector3f translation_b = T_b.inverse().translation();
  Eigen::Vector3f translation = translation_a + t*(translation_b - translation_a);
  Sophus::SE3f T_t_rel_a = interpolator(t);
  return Sophus::SE3f(T_t_rel_a.inverse().unit_quaternion(),  translation).inverse();
}


} // namespace Sophus
