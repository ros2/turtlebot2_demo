/**
 * @file /include/ecl/geometry/odometry_helper.hpp
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ODOMETRY_ODOMETRY_HELPER_HPP_
#define ECL_ODOMETRY_ODOMETRY_HELPER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include "macros.hpp"
#include "odometry_typedefs.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace odometry {

/*****************************************************************************
** Methods
*****************************************************************************/

/** @brief Shortest distance between a pose and a trajectory
 *
 * This uses straight lines between the trajectory poses
 * to calculate the distance to the pose
 */
ecl_geometry_PUBLIC double distance(const Pose2D& pose, const Trajectory2D& trajectory);
/** @brief Shortest distance between a position and a trajectory
 *
 * This uses straight lines between the trajectory poses
 * to calculate the distance to the pose
 */
ecl_geometry_PUBLIC double distance(const Position2D& position, const Trajectory2D& trajectory);

ecl_geometry_PUBLIC bool empty(const Trajectory2D& trajectory); /**< @brief Check if trajectory ptr is empty (ptr not set or has no poses) */
ecl_geometry_PUBLIC bool empty(const Odom2DTrajectory& trajectory); /**< @brief Check if trajectory ptr is empty (ptr not set or has no odometries) */

ecl_geometry_PUBLIC int size(const Trajectory2D& trajectory); /**< @brief Get the size of the trajectory */
ecl_geometry_PUBLIC int size(const Odom2DTrajectory& trajectory); /**< @brief Get the size of the trajectory */

ecl_geometry_PUBLIC double distance(const Odom2D& a, const Odom2D& b); /**< @brief Distance between the positions of odometries */
ecl_geometry_PUBLIC double distance(const Pose2D& a, const Pose2D& b); /**< @brief Distance between poses */
ecl_geometry_PUBLIC double distance(const Pose2D& a, const Odom2D& b); /**< @brief Distance between a pose and the position of a odometry */
ecl_geometry_PUBLIC double distanceSqared(const Odom2D& a, const Odom2D& b); /**< @brief Squared distance between the positions of odometries */
ecl_geometry_PUBLIC double distanceSqared(const Pose2D& a, const Pose2D& b); /**< @brief Squared distance between poses */

/**< @brief Concat two trajectories
 *
 * Adds a trajectory to the end of another trajectory.
 * Shouldn't be used for frequent adding because of bad performance.
 */
ecl_geometry_PUBLIC void addAtEnd(Trajectory2D& target, const Trajectory2D& addition);
/**< @brief Concat two odometry trajectories
 *
 * Adds a trajectory to the end of another trajectory.
 * Shouldn't be used for frequent adding because of bad performance.
 */
ecl_geometry_PUBLIC void addAtEnd(Odom2DTrajectory& target, const Odom2DTrajectory& addition);

ecl_geometry_PUBLIC Trajectory2D vectorToTrajectory(const std::vector<Pose2D>& vec); /**< @brief Convert vector of Pose2D to Trajectory2D */
ecl_geometry_PUBLIC Odom2DTrajectory vectorToTrajectory(const std::vector<Odom2D>& vec); /**< @brief Convert vector of Odom2D to Odom2DTrajectory */

/**< @brief Resizes trajectory appending uninitialised values if needed
 *
 * This is conservative, meaning elements in the trajectory don't change.
 * (Opposed to the default Eigen resize)
 */
ecl_geometry_PUBLIC void resize(Trajectory2D& trajectory, const int& size);
/**< @brief Resizes trajectory appending uninitialised values if needed
 *
 * This is conservative, meaning elements in the trajectory don't change.
 * (Opposed to the default Eigen resize)
 */
ecl_geometry_PUBLIC void resize(Odom2DTrajectory& trajectory, const int& size);

ecl_geometry_PUBLIC void setAt(Trajectory2D& trajectory, const int& index,  const Pose2D& pose); /**< @brief Set element at index of trajectory */
ecl_geometry_PUBLIC void setAt(Odom2DTrajectory& trajectory, const int& index, const Odom2D& odom); /**< @brief Set element at index of trajectory */

ecl_geometry_PUBLIC Pose2D getAt(const Trajectory2D& trajectory, const int& index); /**< @brief Get element of trajectory */
ecl_geometry_PUBLIC Odom2D getAt(const Odom2DTrajectory& trajectory, const int& index); /**< @brief Get element of trajectory */

ecl_geometry_PUBLIC Pose2D getFront(const Trajectory2D& trajectory); /**< @brief Get front (first) element of trajectory */
ecl_geometry_PUBLIC Pose2D getBack(const Trajectory2D& trajectory); /**< @brief Get back (last) element of trajectory */
ecl_geometry_PUBLIC Odom2D getFront(const Odom2DTrajectory& trajectory); /**< @brief Get front (first) element of trajectory */
ecl_geometry_PUBLIC Odom2D getBack(const Odom2DTrajectory& trajectory); /**< @brief Get back (last) element of trajectory */

ecl_geometry_PUBLIC Trajectory2D getPoses(const Odom2DTrajectory& trajectory); /**< @brief Extract poses of odom trajectory */
ecl_geometry_PUBLIC Twist2DVector getTwists(const Odom2DTrajectory& trajectory); /**< @brief Extract twists of odom trajectory */

ecl_geometry_PUBLIC void setVelocityX(Odom2D& odom, const float& value); /**< @brief Set linear velocity x direction */
ecl_geometry_PUBLIC void setVelocityY(Odom2D& odom, const float& value); /**< @brief Set linear velocity y direction */
ecl_geometry_PUBLIC void setVelocityAngular(Odom2D& odom, const float& value); /**< @brief Set angular velocity */

ecl_geometry_PUBLIC void setVelocityX(Twist2D& twist, const float& value); /**< @brief Set linear velocity x direction */
ecl_geometry_PUBLIC void setVelocityY(Twist2D& twist, const float& value); /**< @brief Set linear velocity y direction */
ecl_geometry_PUBLIC void setVelocityAngular(Twist2D& twist, const float& value); /**< @brief Set angular velocity */

ecl_geometry_PUBLIC void setX(Odom2D& odom, const float& value); /**< @brief Set x position */
ecl_geometry_PUBLIC void setY(Odom2D& odom, const float& value); /**< @brief Set y position */
ecl_geometry_PUBLIC void setYaw(Odom2D& odom, const float& value); /**< @brief Set yaw (heading) */

ecl_geometry_PUBLIC void setX(Pose2D& pose, const float& value); /**< @brief Set x position */
ecl_geometry_PUBLIC void setY(Pose2D& pose, const float& value); /**< @brief Set y position */
ecl_geometry_PUBLIC void setYaw(Pose2D& pose, const float& value); /**< @brief Set yaw (heading) */

ecl_geometry_PUBLIC void setX(Position2D& position, const float& value); /**< @brief Set x position */
ecl_geometry_PUBLIC void setY(Position2D& position, const float& value); /**< @brief Set y position */

ecl_geometry_PUBLIC float getVelocityX(const Odom2D& odom); /**< @brief Get linear velocity x direction */
ecl_geometry_PUBLIC float getVelocityY(const Odom2D& odom); /**< @brief Get linear velocity y direction */
ecl_geometry_PUBLIC float getVelocityAngular(const Odom2D& odom); /**< @brief Get angular velocity */

ecl_geometry_PUBLIC float getVelocityX(const Twist2D& twist); /**< @brief Get linear velocity x direction */
ecl_geometry_PUBLIC float getVelocityY(const Twist2D& twist); /**< @brief Get linear velocity y direction */
ecl_geometry_PUBLIC float getVelocityAngular(const Twist2D& twist); /**< @brief Get angular velocity */

ecl_geometry_PUBLIC float getX(const Odom2D& odom); /**< @brief Get x position */
ecl_geometry_PUBLIC float getY(const Odom2D& odom); /**< @brief Get y position */
ecl_geometry_PUBLIC float getYaw(const Odom2D& odom); /**< @brief Get yaw (heading) */

ecl_geometry_PUBLIC Pose2D getPose(const Odom2D& odom); /**< @brief Extract pose from odometry */
ecl_geometry_PUBLIC Position2D getPosition(const Odom2D& odom); /**< @brief Extract position from odometry */
ecl_geometry_PUBLIC Twist2D getTwist(const Odom2D& odom); /**< @brief Extract twist from odometry */

ecl_geometry_PUBLIC float getX(const Pose2D& pose); /**< @brief Get x position */
ecl_geometry_PUBLIC float getY(const Pose2D& pose); /**< @brief Get y position */
ecl_geometry_PUBLIC float getYaw(const Pose2D& pose); /**< @brief Get yaw (heading) */

ecl_geometry_PUBLIC Position2D getPosition(const Pose2D& pose); /**< @brief Extract position from pose */

ecl_geometry_PUBLIC float getX(const Position2D& position); /**< @brief Get x position */
ecl_geometry_PUBLIC float getY(const Position2D& position); /**< @brief Get y position */

/*****************************************************************************
** C++11 helpers
*****************************************************************************/

#if defined(ECL_CXX11_FOUND)

  ecl_geometry_PUBLIC bool empty(const Trajectory2DPtr& trajectory_ptr); /**< @brief Check if trajectory ptr is empty (ptr not set or has no poses) */
  ecl_geometry_PUBLIC bool empty(const Odom2DTrajectoryPtr& trajectory_ptr); /**< @brief Check if trajectory ptr is empty (ptr not set or has no odometries) */

  ecl_geometry_PUBLIC int size(const Trajectory2DPtr& trajectory); /**< @brief Get the size of the trajectory */
  ecl_geometry_PUBLIC int size(const Odom2DTrajectoryPtr& trajectory); /**< @brief Get the size of the trajectory */

  ecl_geometry_PUBLIC Trajectory2DPtr vectorToTrajectoryPtr(const std::vector<Pose2D>& vec); /**< @brief Convert vector of Pose2D to Trajectory2DPtr */
  ecl_geometry_PUBLIC Odom2DTrajectoryPtr vectorToTrajectoryPtr(const std::vector<Odom2D>& vec); /**< @brief Convert vector of Odom2D to Odom2DTrajectoryPtr */

#endif /*ECL_CXX11_FOUND*/

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace odometry
} // namsepace ecl

#endif /*ECL_ODOMETRY_ODOMETRY_HELPER_HPP_*/

