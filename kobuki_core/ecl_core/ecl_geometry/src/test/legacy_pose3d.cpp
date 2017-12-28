/**
 * @file /src/test/pose3d.cpp
 *
 * @ingroup UnitTests
 *
 * Use this to test the Pose3D class
 *
 * @date September 2010
 **/
/*****************************************************************************
** Preprocessor
*****************************************************************************/

#define ECL_USE_EIGEN3 1

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <ecl/linear_algebra.hpp>
#include <ecl/math/constants.hpp>
#include "../../include/ecl/geometry/legacy_pose2d.hpp"
#include "../../include/ecl/geometry/legacy_pose3d.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::LegacyPose2D;
using ecl::LegacyPose3D;
using ecl::linear_algebra::AngleAxis;
using ecl::linear_algebra::Matrix3d;
using ecl::linear_algebra::Vector3d;
using ecl::linear_algebra::Quaternion;
using ecl::linear_algebra::Translation3d;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(LegacyPose3DTests,constructors) {
	bool result;
	Vector3d v; v << 1.0,2.0,3.0;
	Matrix3d zero_rotation = Matrix3d::Identity();
	Vector3d zero_translation; zero_translation << 0.0,0.0,0.0;
	LegacyPose2D<double> pose_2d(0.0,1.0,ecl::pi);
	LegacyPose3D<double> pose1(zero_rotation,v);
	EXPECT_TRUE(pose1.rotation().isApprox(zero_rotation,0.1));
	EXPECT_TRUE(pose1.translation().isApprox(v,0.1));
	LegacyPose3D<double> pose2(pose_2d);
	result = pose2.rotation().block<2,2>(0,0).isApprox(pose_2d.rotationMatrix(),0.1);
	EXPECT_TRUE(result);
	result = pose2.translation().segment<2>(0).isApprox(pose_2d.translation(),0.1);
	EXPECT_TRUE(result);
	LegacyPose3D<double> pose3(Quaternion<double>(pose2.rotation()),v);
	EXPECT_TRUE(pose3.rotation().isApprox(pose2.rotation(),0.1));
	EXPECT_TRUE(pose3.translation().isApprox(v,0.1));
	LegacyPose3D<double> pose4(pose3);
	EXPECT_TRUE(pose4.rotation().isApprox(pose3.rotation(),0.1));
	EXPECT_TRUE(pose4.translation().isApprox(pose3.translation(),0.1));
}


TEST(LegacyPose3DTests,assignment) {
	Vector3d v; v << 0.0,1.0,0.0;
	Matrix3d rot; rot << -1.0,  0.0,  0.0,
						  0.0, -1.0,  0.0,
						  0.0,  0.0,  1.0;;
	LegacyPose2D<double> pose_2d(0.0,1.0,ecl::pi);
	LegacyPose3D<double> pose1, pose2;
	pose1 = pose_2d;
	EXPECT_TRUE(pose1.rotation().isApprox(rot,0.1));
	EXPECT_TRUE(pose1.translation().isApprox(v,0.1));
	pose2 = pose1;
	EXPECT_TRUE(pose2.rotation().isApprox(rot,0.1));
	EXPECT_TRUE(pose2.translation().isApprox(v,0.1));
}

TEST(LegacyPose3DTests,eigenStyle) {
	Vector3d v; v << 0.0,1.0,0.0;
	Matrix3d rot; rot << -1.0,  0.0,  0.0,
						  0.0, -1.0,  0.0,
						  0.0,  0.0,  1.0;;
	LegacyPose3D<double> pose1;
	pose1.rotation(rot);
	pose1.translation(v);
	EXPECT_TRUE(pose1.rotation().isApprox(rot,0.1));
	EXPECT_TRUE(pose1.translation().isApprox(v,0.1));
}
TEST(LegacyPose3DTests,inverse) {
	Vector3d z_axis; z_axis << 0.0, 0.0, 1.0;
	Vector3d zero = Vector3d::Zero();
	Matrix3d rot = AngleAxis<double>(ecl::pi/2.0,z_axis).toRotationMatrix();
	Vector3d trans; trans << 1.0, 2.0, 3.0;
	LegacyPose3D<double> pose(rot, trans);
	LegacyPose3D<double> inverse = pose.inverse();
	LegacyPose3D<double> repose = pose*inverse;
	EXPECT_TRUE(Matrix3d::Identity().isApprox(repose.rotation(),0.1));
	for ( unsigned int i = 0; i < 3; ++i ) {
		EXPECT_LT(zero[i],repose.translation()[i]+0.01);
		EXPECT_GT(zero[i]+0.01,repose.translation()[i]);
	}
}

TEST(LegacyPose3DTests,operators) {
	Vector3d z_axis; z_axis << 0.0, 0.0, 1.0;
	Matrix3d rot = AngleAxis<double>(ecl::pi/2.0,z_axis).toRotationMatrix();
	Matrix3d rot_pi = AngleAxis<double>(ecl::pi,z_axis).toRotationMatrix();
	Vector3d trans; trans << 1.0, 2.0, 3.0;
	Vector3d expected_final_trans; expected_final_trans << -1.0, 3.0, 6.0;
	LegacyPose3D<double> pose(rot, trans);
	LegacyPose3D<double> final_pose = pose*pose;
	EXPECT_TRUE(rot_pi.isApprox(final_pose.rotation()));
	EXPECT_TRUE(expected_final_trans.isApprox(final_pose.translation()));
	pose *= pose;
	EXPECT_TRUE(rot_pi.isApprox(pose.rotation()));
	EXPECT_TRUE(expected_final_trans.isApprox(pose.translation()));
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
