/**
 * @file /include/ecl/geometry/odometry_typedefs.hpp
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ODOMETRY_ODOMETRY_TYPEDEFS_HPP_
#define ECL_ODOMETRY_ODOMETRY_TYPEDEFS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/linear_algebra.hpp>

#if defined(ECL_CXX11_FOUND)
    #include <memory>
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace odometry {

/*****************************************************************************
** Methods
*****************************************************************************/

/**
 * @brief Float representation for a 2D position (x-, y-position).
 *
 * Represents a point in 2D. Uses a 2x1 Eigen vector.
 */
typedef Eigen::Vector2f Position2D;

/**
 * @brief Float representation of a path in 2D (x1,y1; x2,y2; ..; xn,yn).
 *
 * Represents a path in 2D of 2D points (no heading / orientation.
 * Uses a 2xn Eigen matrix where each colum is one point of the path.
 */
typedef Eigen::Matrix2Xf Path2D;

/**
 * @brief Float representation of a pose in 2D (x, y, heading).
 *
 * Represents a pose in 2D (2D position + heading).
 * Uses a 3x1 Eigen vector.
 */
typedef Eigen::Vector3f Pose2D;

/**
 * @brief Float representation of a trajectory in 2D (poses in 2D).
 *
 * Represents a trajectory in 2D of 2D poses (x, y, heading).
 * Uses a 3xn Eigen matrix where each column is one pose of the trajectory.
 */
typedef Eigen::Matrix3Xf Trajectory2D;

/**
 * @brief Float representation of velocities in 2D (v_x, v_y, w).
 *
 * Represents a twist in 2D with the linear velocities in x- and y-direction
 * (v_x, v_y) and the angular velocity (w).
 * Uses a 3x1 Eigen vector.
 */
typedef Eigen::Vector3f Twist2D;

/**
 * @brief Float collection of 2D twists (twist: v_x, v_y, w).
 *
 * Represents a collection of linear and angular velocities in 2D (=twists).
 * Uses a 3xn Eigen matrix where each column is one twist.
 */
typedef Eigen::Matrix3Xf Twist2DVector;

/**
 * @brief Float representation of 2D odometry (x, y, heading, v_x, v_y, w).
 *
 * Represents a pose (x, y, heading) and the twist (linear velocities,
 * angular velocities) of an object in a 2D plane, e.g. a mobile robot.
 *
 * Uses a 6x1 Eigen vector where the upper half is the pose and the
 * lower half the twist.
 */
typedef Eigen::Matrix<float, 6, 1> Odom2D;

/**
 * @brief Float collection of 2D odometries (x, y, heading, v_x, v_y, w).
 *
 * Represents a collection of poses (x, y, heading) together with
 * linear and angular velocities in 2D (=twists).
 *
 * Uses a 6xn Eigen matrix where each column is one odometry with
 * the upper half being the pose and the lower half the twist.
 */
typedef Eigen::Matrix<float, 6, Eigen::Dynamic> Odom2DTrajectory;


/*****************************************************************************
** c++11 shared_ptr typedefs
*****************************************************************************/

#if defined(ECL_CXX11_FOUND)

  /** @brief std::shared_ptr of Position2D (2x1 Eigen vector) */
  typedef std::shared_ptr<Position2D> Position2DPtr;
  /** @brief std::shared_ptr of Path2D (2xn Eigen matrix) */
  typedef std::shared_ptr<Path2D> Path2DPtr;

  /** @brief std::shared_ptr of Pose2D (3x1 Eigen vector: x, y, heading) */
  typedef std::shared_ptr<Pose2D> Pose2DPtr;
  /** @brief std::shared_ptr of Trajectory2D (3xn Eigen matrix) */
  typedef std::shared_ptr<Trajectory2D> Trajectory2DPtr;

  /** @brief std::shared_ptr of Twist2D (3x1 Eigen vector: v_x, v_y, w) */
  typedef std::shared_ptr<Twist2D> Twist2DPtr;
  /** @brief std::shared_ptr of Twist2DVector (3xn Eigen matrix) */
  typedef std::shared_ptr<Twist2DVector> Twist2DVectorPtr;

  /** @brief std::shared_ptr of Odom2D (6xn Eigen vector) */
  typedef std::shared_ptr<Odom2D> Odom2DPtr;
  /** @brief std::shared_ptr of Odom2DTrajectory (6xn Eigen matrix) */
  typedef std::shared_ptr<Odom2DTrajectory> Odom2DTrajectoryPtr;

#endif /*ECL_CXX11_FOUND*/

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace odometry
} // namespace ecl

#endif /*ECL_ODOMETRY_ODOMETRY_TYPEDEFS_HPP_*/
