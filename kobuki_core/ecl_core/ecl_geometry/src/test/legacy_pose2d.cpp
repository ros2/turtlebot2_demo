/**
 * @file /src/test/pose2d.cpp
 *
 * @ingroup UnitTests
 *
 * Use this to test the Pose2D class
 *
 * @date August 2010
 **/
/*****************************************************************************
** Defines
*****************************************************************************/

#define ECL_USE_EIGEN3

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <ecl/linear_algebra.hpp>
#include <ecl/math/constants.hpp>
#include "../../include/ecl/geometry/angle.hpp"
#include "../../include/ecl/geometry/legacy_pose2d.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Angle;
using ecl::LegacyPose2D;
using ecl::RotationAngleStorage;
using ecl::linear_algebra::Matrix2d;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector3d;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(LegacyPose2DTests,RotationMatrixConstructors) {
	LegacyPose2D<double> pose1;
	EXPECT_EQ(0.0,pose1.x());
	EXPECT_EQ(0.0,pose1.y());
	EXPECT_GT(1.01,pose1.rotation()(0,0));
	EXPECT_LT(0.99,pose1.rotation()(0,0));
	EXPECT_GT(0.01,pose1.rotation()(0,1));
	EXPECT_LT(-0.01,pose1.rotation()(0,1));
	EXPECT_GT(0.01,pose1.rotation()(1,0));
	EXPECT_LT(-0.01,pose1.rotation()(1,0));
	EXPECT_GT(1.01,pose1.rotation()(1,1));
	EXPECT_LT(0.99,pose1.rotation()(1,1));
	LegacyPose2D<double> pose2(1.0, 2.0, 3.14);
	EXPECT_EQ(1.0,pose2.x());
	EXPECT_EQ(2.0,pose2.y());
	EXPECT_EQ(3.14,pose2.heading());
	Vector2d trans; trans << 1.0, 2.0;
	Matrix2d rot = Angle<double>(3.14).rotationMatrix();
	LegacyPose2D<double> pose3(rot,trans);
	EXPECT_EQ(1.0,pose3.x());
	EXPECT_EQ(2.0,pose3.y());
	EXPECT_GT(3.15, pose3.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose3.heading());
	LegacyPose2D<double> pose4(3.14,trans);
	EXPECT_EQ(1.0,pose4.x());
	EXPECT_EQ(2.0,pose4.y());
	EXPECT_GT(3.15, pose4.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose4.heading());
	LegacyPose2D<double> pose5(pose4);
	EXPECT_EQ(1.0,pose5.x());
	EXPECT_EQ(2.0,pose5.y());
	EXPECT_GT(3.15, pose5.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose5.heading());
	LegacyPose2D<double,RotationAngleStorage> pose_other(3.14,trans);
	LegacyPose2D<double> pose6(pose_other);
	EXPECT_EQ(1.0,pose6.x());
	EXPECT_EQ(2.0,pose6.y());
	EXPECT_GT(3.15, pose6.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose6.heading());
}

TEST(LegacyPose2DTests,RotationAngleConstructors) {
	LegacyPose2D<double,RotationAngleStorage> pose1;
	EXPECT_EQ(0.0,pose1.x());
	EXPECT_EQ(0.0,pose1.y());
	EXPECT_GT(0.01,pose1.rotation());
	EXPECT_LT(-0.01,pose1.rotation());
	LegacyPose2D<double,RotationAngleStorage> pose2(1.0, 2.0, 3.14);
	EXPECT_EQ(1.0,pose2.x());
	EXPECT_EQ(2.0,pose2.y());
	EXPECT_EQ(3.14,pose2.heading());
	Vector2d trans; trans << 1.0, 2.0;
	Matrix2d rot = Angle<double>(3.14).rotationMatrix();
	LegacyPose2D<double,RotationAngleStorage> pose3(rot,trans);
	EXPECT_EQ(1.0,pose3.x());
	EXPECT_EQ(2.0,pose3.y());
	EXPECT_GT(3.15, pose3.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose3.heading());
	LegacyPose2D<double,RotationAngleStorage> pose4(3.14,trans);
	EXPECT_EQ(1.0,pose4.x());
	EXPECT_EQ(2.0,pose4.y());
	EXPECT_GT(3.15, pose4.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose4.heading());
	LegacyPose2D<double,RotationAngleStorage> pose5(pose4);
	EXPECT_EQ(1.0,pose5.x());
	EXPECT_EQ(2.0,pose5.y());
	EXPECT_GT(3.15, pose5.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose5.heading());
	LegacyPose2D<double> pose_other(1.0, 2.0, 3.14);
	LegacyPose2D<double,RotationAngleStorage> pose6(pose_other);
	EXPECT_EQ(1.0,pose6.x());
	EXPECT_EQ(2.0,pose6.y());
	EXPECT_GT(3.15, pose6.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose6.heading());
}

TEST(LegacyPose2DTests,assignment) {
	LegacyPose2D<double> pose_r1(2.0, 1.0, 1.14);
	LegacyPose2D<double,RotationAngleStorage> pose_a1(1.0, 2.0, 3.14);
	LegacyPose2D<double> pose_r2;
	LegacyPose2D<double,RotationAngleStorage> pose_a2;
	pose_r2 = pose_r1;
	EXPECT_EQ(2.0,pose_r2.x());
	EXPECT_EQ(1.0,pose_r2.y());
	EXPECT_GT(1.15, pose_r2.heading());// Allow some roundoff error here
	EXPECT_LT(1.13, pose_r2.heading());
	pose_r2 = pose_a1;
	EXPECT_EQ(1.0,pose_r2.x());
	EXPECT_EQ(2.0,pose_r2.y());
	EXPECT_GT(3.15, pose_r2.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose_r2.heading());
	pose_a2 = pose_r1;
	EXPECT_EQ(2.0,pose_a2.x());
	EXPECT_EQ(1.0,pose_a2.y());
	EXPECT_GT(1.15, pose_a2.heading());// Allow some roundoff error here
	EXPECT_LT(1.13, pose_a2.heading());
	pose_a2 = pose_a1;
	EXPECT_EQ(1.0,pose_a2.x());
	EXPECT_EQ(2.0,pose_a2.y());
	EXPECT_GT(3.15, pose_a2.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, pose_a2.heading());
}

TEST(LegacyPose2DTests,eigenStyle) {
	Vector2d trans; trans << 1.0, 2.0;
	Matrix2d rot = Angle<double>(3.14).rotationMatrix();
	LegacyPose2D<double> rpose;
	rpose.rotation(rot);
	rpose.translation(trans);
	EXPECT_EQ(1.0,rpose.x());
	EXPECT_EQ(2.0,rpose.y());
	EXPECT_GT(3.15, rpose.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, rpose.heading());
	rot = rpose.rotation();
	trans = rpose.translation();
	EXPECT_EQ(1.0,trans[0]);
	EXPECT_EQ(2.0,trans[1]);
	EXPECT_GT(3.15, Angle<double>(rot));// Allow some roundoff error here
	EXPECT_LT(3.13, Angle<double>(rot));
	LegacyPose2D<double,RotationAngleStorage> apose;
	apose.rotation(3.14);
	apose.translation(trans);
	EXPECT_EQ(1.0,apose.x());
	EXPECT_EQ(2.0,apose.y());
	EXPECT_GT(3.15, apose.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, apose.heading());
	double angle = apose.rotation();
	trans = apose.translation();
	EXPECT_EQ(1.0,trans[0]);
	EXPECT_EQ(2.0,trans[1]);
	EXPECT_GT(3.15, angle);// Allow some roundoff error here
	EXPECT_LT(3.13, angle);
}
TEST(LegacyPose2DTests,convenienceStyle) {
	Vector2d trans; trans << 1.0, 2.0;
	Matrix2d rot;
	LegacyPose2D<double> rpose;
	rpose.x(1.0); rpose.y(2.0); rpose.heading(3.14);
	EXPECT_EQ(1.0,rpose.x());
	EXPECT_EQ(2.0,rpose.y());
	EXPECT_GT(3.15, rpose.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, rpose.heading());
	rpose.heading(0.0);
	rot = rpose.rotationMatrix();
	EXPECT_GT(1.01,rot(0,0));
	EXPECT_LT(0.99,rot(0,0));
	EXPECT_GT(0.01,rot(0,1));
	EXPECT_LT(-0.01,rot(0,1));
	EXPECT_GT(0.01,rot(1,0));
	EXPECT_LT(-0.01,rot(1,0));
	EXPECT_GT(1.01,rot(1,1));
	EXPECT_LT(0.99,rot(1,1));
	LegacyPose2D<double,RotationAngleStorage> apose;
	apose.x(1.0); apose.y(2.0); apose.heading(3.14);
	EXPECT_EQ(1.0,apose.x());
	EXPECT_EQ(2.0,apose.y());
	EXPECT_GT(3.15, apose.heading());// Allow some roundoff error here
	EXPECT_LT(3.13, apose.heading());
	apose.heading(0.0);
	rot = apose.rotationMatrix();
	EXPECT_GT(1.01,rot(0,0));
	EXPECT_LT(0.99,rot(0,0));
	EXPECT_GT(0.01,rot(0,1));
	EXPECT_LT(-0.01,rot(0,1));
	EXPECT_GT(0.01,rot(1,0));
	EXPECT_LT(-0.01,rot(1,0));
	EXPECT_GT(1.01,rot(1,1));
	EXPECT_LT(0.99,rot(1,1));
}
TEST(LegacyPose2DTests,inverse) {
	LegacyPose2D<double> pose(1.0, 2.0, ecl::pi);
	LegacyPose2D<double> inverse = pose.inverse();
	EXPECT_GT(1.01,inverse.x()); // Allow some roundoff error here
	EXPECT_LT(0.99,inverse.x());
	EXPECT_GT(2.01,inverse.y()); // Allow some roundoff error here
	EXPECT_LT(1.99,inverse.y());
	EXPECT_GT(-ecl::pi+0.01,inverse.heading()); // Allow some roundoff error here
	EXPECT_LT(-ecl::pi-0.01,inverse.heading());
	LegacyPose2D<double,RotationAngleStorage> apose(1.0, 2.0, ecl::pi);
	LegacyPose2D<double,RotationAngleStorage> ainverse = pose.inverse();
	EXPECT_GT(1.01,ainverse.x()); // Allow some roundoff error here
	EXPECT_LT(0.99,ainverse.x());
	EXPECT_GT(2.01,ainverse.y()); // Allow some roundoff error here
	EXPECT_LT(1.99,ainverse.y());
	EXPECT_GT(-ecl::pi+0.01,ainverse.heading()); // Allow some roundoff error here
	EXPECT_LT(-ecl::pi-0.01,ainverse.heading());
}

TEST(LegacyPose2DTests,operators) {
	LegacyPose2D<double> a(1.0, 2.0, 0.0), b(1.0, 3.0, ecl::pi);
	LegacyPose2D<double> diff = a.inverse()*b; //	diff = b - a;
	EXPECT_EQ(0.0,diff.x());
	EXPECT_EQ(1.0,diff.y());
	EXPECT_EQ(ecl::pi,diff.heading());
	LegacyPose2D<double,RotationAngleStorage> a2(1.0, 2.0, 0.0), b2(1.0, 3.0, ecl::pi);
	LegacyPose2D<double,RotationAngleStorage> diff2 = a2.inverse()*b2; //	diff = b - a;
	EXPECT_EQ(0.0,diff2.x());
	EXPECT_EQ(1.0,diff2.y());
	EXPECT_EQ(ecl::pi,diff2.heading());
}
TEST(LegacyPose2DTests,relative) {
	LegacyPose2D<double> a(1.0, 1.0, 1.57), b(1.0, 2.0, 3.14);
	LegacyPose2D<double> brela = b.relative(a);
	EXPECT_GT(1.01,brela.x());
	EXPECT_LT(0.99,brela.x());
	EXPECT_GT(0.01,brela.y());
	EXPECT_LT(-0.01,brela.y());
	EXPECT_GT(1.58,brela.heading());
	EXPECT_LT(1.56,brela.heading());
	LegacyPose2D<double,RotationAngleStorage> a_(1.0, 1.0, 1.57), b_(1.0, 2.0, 3.14);
	LegacyPose2D<double,RotationAngleStorage> brela_ = b.relative(a);
	EXPECT_GT(1.01,brela_.x());
	EXPECT_LT(0.99,brela_.x());
	EXPECT_GT(0.01,brela_.y());
	EXPECT_LT(-0.01,brela_.y());
	EXPECT_GT(1.58,brela_.heading());
	EXPECT_LT(1.56,brela_.heading());
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
