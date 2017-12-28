/**
 * @file /ecl_mobile_robot/include/ecl/mobile_robot/kinematics/differential_drive.hpp
 *
 * @brief Kinematics equations for differential drive type bases.
 *
 * @date 20/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_
#define ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/geometry/legacy_pose2d.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

namespace mobile_robot {

/*****************************************************************************
** Interface
*****************************************************************************/

/**
 * @brief Differential drive kinematics.
 *
 * This presents some of the differential drive kinematics transforms
 * for a standard differential drive mobile base. The general structure is
 * for a  fixed axis between two separately controlled wheels, usually
 * you can find a balancing castor wheel of some sort to the front or rear.
 *
 * @code
 *
 *        o
 *        |
 *        |
 *   ||-------||
 *
 * @endcode
 *
 * There are three sets of systems used to describe the
 * kinematics.
 *
 * - /dot{theta}_L, /dot{theta}_R  : wheel angular velocities.
 * - /dot{s}, /omega : base linear speed and angular rotational velocities.
 * - /dot{x}, /dot{y}, /dot{phi} : pose velocities wrt global frame.
 *
 * For all of these conversions, it is assumed the forward direction is the
 * x axis for the local pose frame of reference and the frame of reference
 * is located centrally between the two wheels.
 */
class ecl_mobile_robot_PUBLIC DifferentialDriveKinematics {
public:
	/**
	 * @brief Configures the parameters defining the structure.
	 *
	 * @param fixed_axis_length : length between the two wheels [m].
	 * @param wheel_radius : radius of each differential drive wheel [m].
	 */
	DifferentialDriveKinematics(
			const double &fixed_axis_length,
			const double& wheel_radius) :
		bias(fixed_axis_length),
		radius(wheel_radius)
	{}

	/**
	 * @brief Generates a relative (to the robot's frame) pose differential.
	 *
	 * Uses the odometry updates to generate a relative pose update.
	 * This differential can be used to update (add to) the robot's current
	 * pose estimate.
	 *
	 * @code
	 * LegacyPose2D pose;
	 * // ...
	 * pose *= forwardDifferential(0.1,0.2);
	 * @endcode
	 *
	 * @param dleft : incoming left wheel angle change.
	 * @param dright : incoming right wheel angle change.
	 * @return Pose : pose update (differential).
	 */
	ecl::LegacyPose2D<double> forward(const double &dleft, const double &dright) const;

	/**
	 * @brief Generates a relative (to the robot's frame) pose differential
	 * when you know platform velocity directly. In this case you do not need to
	 * know radius informaiton
	 *
	 * @code
	 * LegacyPose2D pose;
	 * // ...
	 * double linearVelocity = ....;
	 * double angularVelocity = ..../
	 * pose *= forwardDifferential(linearVelocity, angularVelocity);
	 *
	 * @param linearVelocity : incoming linear velocity of platform
	 * @param angularVelocity : incoming angular velocity of platform
	 * @return Pose : pose update (differential)
	 */
	ecl::LegacyPose2D<double> forwardWithPlatformVelocity(const double & linearVelocity, const double & angularVelocity ) const;

	/**
	 * @brief Generates wheel angle rates from scalar linear/angular velocity commands.
	 *
	 * Accepts the usual differential drive separation of linear and angular
	 * velocity components and computes the required wheel angle rates for control.
	 *
	 * @param linear_velocity : linear translation control command (in direction of facing).
	 * @param angular_velocity : angular rotation control command (around robot centre).
	 * @return Vector2d : left and right wheel angular rate commands respectively.
	 */
	ecl::linear_algebra::Vector2d inverse(const double &linear_velocity, const double &angular_velocity) const;

	/**
	 * @brief Rough conversion from [dx,dy,dtheta] -> [ds, dw].
	 *
	 * The aim here is to provide a differential between the two poses that is
	 * crudely approximated for non-holonomic motion (ds - linear translation, dw -
	 * angular rotation).
	 *
	 * Moving in this direction provides a non-unique solution (reducing from
	 * 3 dof to 2 dof). If the update period is very small, this limit does,
	 * however, approach a unique solution. Consequently, so long as this
	 * is small, the following is a reasonably simple, lightweight method.
	 *
	 * The ds is simply the scalar distance between two poses, given a sign
	 * for the direction it is pointing in (relative to the robot facing). Alternatively
	 * you could assume an angle + linear translation + angle or some such, but
	 * even then, its still an approximation.
	 *
	 * The angular difference, dw is more simple, it is always just the
	 * angular difference between poses.
	 *
	 * Jae-Yeong was talking about moving this to firmware for better accuracy
	 * (smaller update periods).
	 *
	 * @param a : initial pose
	 * @param b : final pose
	 * @return Vector2d : the differential [ds, dw].
	 */
	ECL_DEPRECATED static ecl::linear_algebra::Vector2d Inverse(const ecl::linear_algebra::Vector3d &a, const ecl::linear_algebra::Vector3d &b);

	/**
	 * @brief Rough conversion from a pose difference -> [ds, dw].
	 *
	 * The aim here is to provide a differential between the two poses that is
	 * crudely approximated for non-holonomic motion (ds - linear translation, dw -
	 * angular rotation).
	 *
	 * Moving in this direction provides a non-unique solution (reducing from
	 * 3 dof to 2 dof). If the update period is very small, this limit does,
	 * however, approach a unique solution. Consequently, so long as this
	 * is small, the following is a reasonably simple, lightweight method.
	 *
	 * The ds is simply the scalar distance between two poses, given a sign
	 * for the direction it is pointing in (relative to the robot facing). Alternatively
	 * you could assume an angle + linear translation + angle or some such, but
	 * even then, its still an approximation.
	 *
	 * The angular difference, dw is more simple, it is always just the
	 * angular difference between poses.
	 *
	 * Jae-Yeong was talking about moving this to firmware for better accuracy
	 * (smaller update periods).
	 *
	 * @param a : initial pose
	 * @param b : final pose
	 * @return Vector2d : the differential [ds, dw].
	 */
	static ecl::linear_algebra::Vector2d PartialInverse(const ecl::LegacyPose2D<double> &a, const ecl::LegacyPose2D<double> &b);
private:
	double bias, radius;
};

} // namespace mobile_robot
} // namespace ecl

#endif /* ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_KINEMATICS_HPP_ */
