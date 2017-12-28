/**
 * @file /src/test/homogeneous_points.cpp
 *
 * @brief Unit Test for the homogeneous point classes.
 *
 * @date March 2012
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/geometry/homogeneous_point.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::linear_algebra::Vector3d;
using ecl::linear_algebra::Vector4d;
using ecl::linear_algebra::Vector4f;
using ecl::HomogeneousPoint;
using ecl::HomogeneousPointf;
using ecl::HomogeneousPointd;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(HomogeneousPointTests,construct) {
	HomogeneousPointd point(0.1,0.2,0.3);
	EXPECT_EQ(0.1,point.x());
	EXPECT_EQ(0.2,point.y());
	EXPECT_EQ(0.3,point.z());
    Vector4d v; v << 0.1, 0.2, 0.3, 1.0;
	HomogeneousPointd p2(v);
	EXPECT_EQ(0.1,p2.x());
	EXPECT_EQ(0.2,p2.y());
	EXPECT_EQ(0.3,p2.z());
    Vector3d v3; v3 << 1.0, 2.0, 3.0;
	HomogeneousPointd p3(v3);
	EXPECT_EQ(1.0,p3.x());
	EXPECT_EQ(2.0,p3.y());
	EXPECT_EQ(3.0,p3.z());
}

TEST(HomogeneousPointTests,assignment) {
	HomogeneousPointf point;
	point << 1.0, 2.0, 3.0, 1.0;
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
	point.x(1.0);
	point.y(2.0);
	point.z(3.0);
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
    Vector4f v; v << 1.0, 2.0, 3.0, 1.0;
    point = v;
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
}

TEST(HomogeneousPointTests,vectorHandle) {
	HomogeneousPointd point;
	point << 1.0, 2.0, 3.0, 1.0;
	const Vector4d& v = point.positionVector();
	EXPECT_EQ(1.0,v[0]);
	EXPECT_EQ(2.0,v[1]);
	EXPECT_EQ(3.0,v[2]);
}

TEST(HomogeneousPointTests,constAccessors) {
	HomogeneousPointd point;
	point << 1.0, 2.0, 3.0, 1.0;
	const int cx = point.x();
	const int cy = point.y();
	const int cz = point.z();
	EXPECT_EQ(1.0,cx);
	EXPECT_EQ(2.0,cy);
	EXPECT_EQ(3.0,cz);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


