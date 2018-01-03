/**
 * @file /ecl_geometry/include/ecl/geometry/pose.hpp
 *
 * It is much more flexible for users to have standard pose data structures
 * that can be passed around easily (often with existing code of their
 * own) and have an accompanying library for extra functionality than to
 * have custom c++ pose classes (as we do in pose2d.hpp and pose3d.hpp).
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_geometry_GEOMETRY_POSE_HPP_
#define ecl_geometry_GEOMETRY_POSE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/

// prefer to typedef these as and where needed rather than 'force' typedefs
// on users of this library (this lets us fling classes and libraries around
// for which the only dependencies are Eigen and Sophus)

//typedef Eigen::Vector3f Pose2D;  /**< @brief 2D float representation of planner poses (x, y, heading). **/
//typedef Sophus::SE3f    Pose3D;  /**< @brief 3D float representation of planner poses. **/

// Haven't had the need to use doubles yet...

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace ecl

#endif /* ecl_geometry_GEOMETRY_POSE_HPP_ */
