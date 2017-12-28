/**
 * @file /ecl_mobile_robot/src/lib/kinematics/differential_drive.cpp
 *
 * @brief Implementation of the differential drive class.
 *
 * @date May 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>
#include <ecl/geometry/angle.hpp>
#include <ecl/geometry/legacy_pose2d.hpp>
#include "../../../include/ecl/mobile_robot/kinematics/differential_drive.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace mobile_robot {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Angle;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector3d;
using ecl::linear_algebra::Rotation2D;

/*****************************************************************************
** Implementation [Kinematics]
*****************************************************************************/

ecl::LegacyPose2D<double> DifferentialDriveKinematics::forward(const double &dleft, const double& dright) const {

  LegacyPose2D<double> pose_update;
  double ds = radius*(dleft+dright)/2.0;
  double domega = radius*(dright-dleft)/bias;
  // Local robot frame of reference has the x axis pointing forward
  // Since the pose update is using the robot frame of reference, the y update is zero
  pose_update.translation(ds, 0);
  pose_update.rotation(domega);
  return pose_update;
}

ecl::LegacyPose2D<double> DifferentialDriveKinematics::forwardWithPlatformVelocity(const double & linearVelocity, const double & angularVelocity ) const
{
  ecl::LegacyPose2D<double> pose_update;
  // Local robot frame of reference has the x axis pointing forward
  // Since the pose update is using the robot frame of reference, the y update is zero
  pose_update.translation(linearVelocity, 0);
  pose_update.rotation(angularVelocity);
  return pose_update;
}

Vector2d DifferentialDriveKinematics::inverse(const double &linear_velocity, const double &angular_velocity) const {
//	radius*(theta_dot_l+theta_dot_r)/2.0 = linear_velocity
//	radius*(theta_dot_r-theta_dot_l)/bias = angular_velocity
	Vector2d rates;
	double right = (2*linear_velocity+angular_velocity*bias)/(2*radius);
	double left = 2*linear_velocity/radius - right;
	rates << left, right;
	return rates;
}

Vector2d DifferentialDriveKinematics::PartialInverse(const ecl::LegacyPose2D<double> &a,
                                                     const ecl::LegacyPose2D<double> &b)
{
  Vector2d diff;
  diff << b.x() - a.x(), b.y() - a.y();
// This rotation doesn't affect the magnitude of ds,
// but it so the sign can work out properly
  Vector2d dr = a.rotationMatrix().inverse()*diff;
  double ds = sqrt(dr(0)*dr(0)+dr(1)*dr(1));
  if ( dr(0) < 0.0 ) {
    ds = -ds;
  }
  double dtheta = b.heading() - a.heading(); // difference in headings
  ecl::wrap_angle(dtheta);

// diff << ds, dtheta;
  return (diff << ds, dtheta).finished();
}

// Depracated

Vector2d DifferentialDriveKinematics::Inverse(const Vector3d &a, const Vector3d &b) {
	Vector2d diff;
	Vector2d dxy;
	ecl::LegacyPose2D<double> posea(a[2], a.segment<2>(0));
	ecl::LegacyPose2D<double> poseb(b[2], b.segment<2>(0));
	dxy << poseb.x() - posea.x(), poseb.y() - posea.y();
// This rotation doesn't affect the magnitude of ds,
// but it so the sign can work out properly
	Rotation2D<double> rotation(-posea.heading());
	Vector2d dr = rotation*dxy;
// New LegacyPose2D class can extract the rotation matrix directly.
//	Vector2d dr = a.rotationMatrix().inverse()*dxy;
	double ds = sqrt(dr(0)*dr(0)+dr(1)*dr(1));
	if ( dr(0) < 0.0 ) {
		ds = -ds;
	}
	double dtheta = b[2] - a[2]; // difference in headings
	ecl::wrap_angle(dtheta);

	diff << ds, dtheta;
	return diff;
}

} // namespace mobile_robot
} // namespace ecl
