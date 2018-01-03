/**
 * @file /src/lib/odometry_helper.cpp
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include "../../include/ecl/geometry/odometry_helper.hpp"
#include "../../include/ecl/geometry/linear_segment.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace odometry {

/*****************************************************************************
** Implementation
*****************************************************************************/

double distance(const Pose2D& pose, const Trajectory2D& trajectory)
{
  return distance(getPosition(pose), trajectory);
}

double distance(const Position2D& position, const Trajectory2D& trajectory)
{
  Position2D segment_start = trajectory.topLeftCorner<2, 1>();

  if (size(trajectory) == 1)
  {
    return (position - segment_start).norm();
  }

  double min_squared_distance = std::numeric_limits<double>::infinity();
  double temp_squared_distance;

  for (int i = 1; i < size(trajectory); ++i)
  {
    Position2D segment_end = trajectory.block<2, 1>(0, i);
    ecl::LinearSegment segment(getX(segment_start), getY(segment_start), getX(segment_end), getY(segment_end));

    temp_squared_distance = segment.squaredDistanceFromPoint(getX(position), getY(position));

    if (min_squared_distance > temp_squared_distance)
    {
      min_squared_distance = temp_squared_distance;
    }

    segment_start = segment_end;
  }

  return std::sqrt(min_squared_distance);
}

bool empty(const Trajectory2D& trajectory)
{
  return size(trajectory) == 0;
}

bool empty(const Odom2DTrajectory& trajectory)
{
  return size(trajectory) == 0;
}

int size(const Trajectory2D& trajectory)
{
  return trajectory.cols();
}

int size(const Odom2DTrajectory& trajectory)
{
  return trajectory.cols();
}

double distance(const Odom2D& a, const Odom2D& b)
{
  return (getPosition(a) - getPosition(b)).norm();
}

double distance(const Pose2D& a, const Odom2D& b)
{
  return (getPosition(a) - getPosition(b)).norm();
}

double distance(const Pose2D& a, const Pose2D& b)
{
  return (getPosition(a) - getPosition(b)).norm();
}

double distanceSqared(const Odom2D& a, const Odom2D& b)
{
  return (getPosition(a) - getPosition(b)).squaredNorm();
}

double distanceSqared(const Pose2D& a, const Pose2D& b)
{
  return (getPosition(a) - getPosition(b)).squaredNorm();
}

void addAtEnd(Trajectory2D& target, const Trajectory2D& addition)
{
  if(size(target) == 0)
  {
    target = addition;
    return;
  }

  target << addition;
}

void addAtEnd(Odom2DTrajectory& target, const Odom2DTrajectory& addition)
{
  if(size(target) == 0)
  {
    target = addition;
    return;
  }

  target << addition;
}

Trajectory2D vectorToTrajectory(const std::vector<Pose2D>& vec)
{
  Trajectory2D trajectory;
  trajectory.resize(Eigen::NoChange, vec.size());

  for (int i = 0; i < vec.size(); ++i)
  {
    setAt(trajectory, i, vec[i]);
  }

  return trajectory;
}

Odom2DTrajectory vectorToTrajectory(const std::vector<Odom2D>& vec)
{
  Odom2DTrajectory trajectory;
  trajectory.resize(Eigen::NoChange, vec.size());

  for (int i = 0; i < vec.size(); ++i)
  {
    setAt(trajectory, i, vec[i]);
  }

  return trajectory;
}

void resize(Trajectory2D& trajectory, const int& size)
{
  trajectory.conservativeResize(Eigen::NoChange, size);
}

void resize(Odom2DTrajectory& trajectory, const int& size)
{
  trajectory.conservativeResize(Eigen::NoChange, size);
}

void setAt(Trajectory2D& trajectory, const int& index, const Pose2D& pose)
{
  trajectory.col(index) = pose;
}

void setAt(Odom2DTrajectory& trajectory, const int& index, const Odom2D& odom)
{
  trajectory.col(index) = odom;
}

Pose2D getAt(const Trajectory2D& trajectory, const int& index)
{
  return trajectory.col(index);
}

Odom2D getAt(const Odom2DTrajectory& trajectory, const int& index)
{
  return trajectory.col(index);
}

Pose2D getFront(const Trajectory2D& trajectory)
{
  return trajectory.leftCols<1>();
}

Pose2D getBack(const Trajectory2D& trajectory)
{
  return trajectory.rightCols<1>();
}

Odom2D getFront(const Odom2DTrajectory& trajectory)
{
  return trajectory.leftCols<1>();
}

Odom2D getBack(const Odom2DTrajectory& trajectory)
{
  return trajectory.rightCols<1>();
}

Trajectory2D getPoses(const Odom2DTrajectory& trajectory)
{
  return trajectory.topRows<3>();
}

Twist2DVector getTwists(const Odom2DTrajectory& trajectory)
{
  return trajectory.bottomRows<3>();
}

void setVelocityX(Odom2D& odom, const float& value)
{
  odom(3) = value;
}

void setVelocityY(Odom2D& odom, const float& value)
{
  odom(4) = value;
}

void setVelocityAngular(Odom2D& odom, const float& value)
{
  odom(5) = value;
}

void setVelocityX(Twist2D& twist, const float& value)
{
  twist(0) = value;
}

void setVelocityY(Twist2D& twist, const float& value)
{
  twist(1) = value;
}

void setVelocityAngular(Twist2D& twist, const float& value)
{
  twist(2) = value;
}

void setX(Odom2D& odom, const float& value)
{
  odom(0) = value;
}

void setY(Odom2D& odom, const float& value)
{
  odom(1) = value;
}

void setYaw(Odom2D& odom, const float& value)
{
  odom(2) = value;
}

void setX(Pose2D& pose, const float& value)
{
  pose(0) = value;
}

void setY(Pose2D& pose, const float& value)
{
  pose(1) = value;
}

void setYaw(Pose2D& pose, const float& value)
{
  pose(2) = value;
}

void setX(Position2D& position, const float& value)
{
  position(0) = value;
}

void setY(Position2D& position, const float& value)
{
  position(1) = value;
}

float getVelocityX(const Odom2D& odom)
{
  return getVelocityX(getTwist(odom));
}

float getVelocityY(const Odom2D& odom)
{
  return getVelocityY(getTwist(odom));
}

float getVelocityAngular(const Odom2D& odom)
{
  return getVelocityAngular(getTwist(odom));
}

float getVelocityX(const Twist2D& twist)
{
  return twist(0);
}

float getVelocityY(const Twist2D& twist)
{
  return twist(1);
}

float getVelocityAngular(const Twist2D& twist)
{
  return twist(2);
}

float getX(const Odom2D& odom)
{
  return getX(getPose(odom));
}
float getY(const Odom2D& odom)
{
  return getY(getPose(odom));
}
float getYaw(const Odom2D& odom)
{
  return getYaw(getPose(odom));
}

Position2D getPosition(const Odom2D& odom)
{
  return getPosition(getPose(odom));
}

Pose2D getPose(const Odom2D& odom)
{
  return odom.head<3>();
}

Twist2D getTwist(const Odom2D& odom)
{
  return odom.tail<3>();
}

float getX(const Pose2D& pose)
{
  return getX(getPosition(pose));
}

float getY(const Pose2D& pose)
{
  return getY(getPosition(pose));
}

float getYaw(const Pose2D& pose)
{
  return pose(2);
}

Position2D getPosition(const Pose2D& pose)
{
  return pose.head<2>();
}

float getX(const Position2D& position)
{
  return position(0);
}

float getY(const Position2D& position)
{
  return position(1);
}

/*****************************************************************************
** C++11 Implementations
*****************************************************************************/

#if defined(ECL_CXX11_FOUND)

  bool empty(const Trajectory2DPtr& trajectory_ptr)
  {
    return !trajectory_ptr || size(trajectory_ptr) == 0;
  }

  bool empty(const Odom2DTrajectoryPtr& trajectory_ptr)
  {
    return !trajectory_ptr || size(trajectory_ptr) == 0;
  }

  int size(const Trajectory2DPtr& trajectory)
  {
    return size(*trajectory);
  }

  int size(const Odom2DTrajectoryPtr& trajectory)
  {
    return size(*trajectory);
  }

  Trajectory2DPtr vectorToTrajectoryPtr(const std::vector<Pose2D>& vec)
  {
    Trajectory2DPtr trajectory = std::make_shared<Trajectory2D>();
    trajectory->resize(Eigen::NoChange, vec.size());

    for (int i = 0; i < vec.size(); ++i)
    {
      setAt(*trajectory, i, vec[i]);
    }

    return trajectory;
  }

  Odom2DTrajectoryPtr vectorToTrajectoryPtr(const std::vector<Odom2D>& vec)
  {
    Odom2DTrajectoryPtr trajectory = std::make_shared<Odom2DTrajectory>();
    trajectory->resize(Eigen::NoChange, vec.size());

    for (int i = 0; i < vec.size(); ++i)
    {
      setAt(*trajectory, i, vec[i]);
    }

    return trajectory;
  }

#endif /*ECL_CXX11_FOUND*/

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace odometry
} // namespace ecl
