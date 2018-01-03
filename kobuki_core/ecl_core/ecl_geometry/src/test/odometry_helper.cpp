/**
 * @file /src/test/odometry_helper.cpp
 *
 * @ingroup UnitTests
 *
 * Use this to test the odometry helper functions.
 *
 * @date November 2016
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <gtest/gtest.h>
#include "../../include/ecl/geometry/odometry_helper.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::odometry;

/*****************************************************************************
** Tests
*****************************************************************************/

#if defined(ECL_CXX11_FOUND)

  TEST(OdometryTests,distances) {
    Trajectory2D trajectory(3, 5);

    /**   ______
     *    \_    |
     *      \   |
     *          |
     *    ______|
     */
    trajectory << -2.0, 2.0, 2.0, -2.0, 0.0,
                  -2.0, -2.0, 2.0, 2.0, 0.0,
                  0.3, -2.3, 0.0, 4.0, -4.0;

    Pose2D pose;
    double distance_pose = 0.0;
    double distance_position = 0.0;

    pose = Pose2D(0.0, -3.0, 4.0);
    distance_pose = distance(pose, trajectory);

    EXPECT_EQ(1.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(3.0, 1.0, 0.1);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(1.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(-3.0, 1.0, -0.1);
    distance_pose = distance(pose, trajectory);
    // diagnoal -> sqrt(2)
    EXPECT_GT(1.4143, distance_pose);
    EXPECT_LT(1.4142, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(1.0, 3.0, -3.2);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(1.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(1.0, -3.0, 4.1);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(1.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(-2.0, -4.0, 0.1);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(2.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(-2.0, -2.0, 0.1);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(0.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);

    pose = Pose2D(1.0, 0.0, -8.0);
    distance_pose = distance(pose, trajectory);
    EXPECT_EQ(1.0, distance_pose);
    distance_position = distance(getPosition(pose), trajectory);
    EXPECT_EQ(distance_pose, distance_position);
  }

  TEST(OdometryTests,trajectories) {
    Trajectory2DPtr trajectory;
    EXPECT_EQ(true, empty(trajectory));

    Odom2DTrajectoryPtr odom_traj;
    EXPECT_EQ(true, empty(odom_traj));

    trajectory = std::make_shared<Trajectory2D>();
    EXPECT_EQ(true, empty(trajectory));
    EXPECT_EQ(0, size(trajectory));
    EXPECT_EQ(0, size(*trajectory));

    odom_traj = std::make_shared<Odom2DTrajectory>();
    EXPECT_EQ(true, empty(odom_traj));
    EXPECT_EQ(0, size(odom_traj));
    EXPECT_EQ(0, size(*odom_traj));

    trajectory = std::make_shared<Trajectory2D>(3, 1);
    *trajectory << 0.0, 0.0, 0.0;
    EXPECT_EQ(false, empty(trajectory));
    EXPECT_EQ(1, size(trajectory));
    EXPECT_EQ(1, size(*trajectory));

    odom_traj = std::make_shared<Odom2DTrajectory>(6, 1);
    *odom_traj << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    EXPECT_EQ(false, empty(odom_traj));
    EXPECT_EQ(1, size(odom_traj));
    EXPECT_EQ(1, size(*odom_traj));

    resize(*trajectory, 4);
    EXPECT_EQ(false, empty(trajectory));
    EXPECT_EQ(4, size(trajectory));
    EXPECT_EQ(4, size(*trajectory));

    resize(*odom_traj, 7);
    EXPECT_EQ(false, empty(odom_traj));
    EXPECT_EQ(7, size(odom_traj));
    EXPECT_EQ(7, size(*odom_traj));

    resize(*trajectory, 0);
    EXPECT_EQ(true, empty(trajectory));
    EXPECT_EQ(0, size(trajectory));
    EXPECT_EQ(0, size(*trajectory));

    resize(*odom_traj, 0);
    EXPECT_EQ(true, empty(odom_traj));
    EXPECT_EQ(0, size(odom_traj));
    EXPECT_EQ(0, size(*odom_traj));

    resize(*odom_traj, 3);
    addAtEnd(*trajectory, getPoses(*odom_traj));
    EXPECT_EQ(false, empty(trajectory));
    EXPECT_EQ(3, size(trajectory));
    EXPECT_EQ(3, size(*trajectory));

    EXPECT_EQ(getAt(*trajectory, 0), getPose(getAt(*odom_traj, 0)));
    EXPECT_EQ(getAt(*trajectory, 2), getAt(getPoses(*odom_traj), 2));

    EXPECT_EQ(getAt(*trajectory, 0), getFront(*trajectory));
    EXPECT_EQ(getAt(*odom_traj, 0), getFront(*odom_traj));

    EXPECT_EQ(getAt(*trajectory, size(trajectory) - 1), getBack(*trajectory));
    EXPECT_EQ(getAt(*odom_traj, 2), getBack(*odom_traj));

    setAt(*trajectory, 2, getFront(*trajectory));
    EXPECT_EQ(getAt(*trajectory, 2), getFront(*trajectory));

    setAt(*odom_traj, 1, getFront(*odom_traj));
    EXPECT_EQ(getAt(*odom_traj, 1), getFront(*odom_traj));
  }
#endif

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


