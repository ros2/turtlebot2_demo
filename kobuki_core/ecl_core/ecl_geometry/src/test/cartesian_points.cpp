/**
 * @file /src/test/cartesian_points.cpp
 *
 * @brief Unit Test for the cartesian point classes.
 *
 * @date October 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/geometry/cartesian_point.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::linear_algebra::Vector3d;
using ecl::CartesianPoint;
using ecl::CartesianPoint3d;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(CartesianPointTests,construct) {
	CartesianPoint3d point(0.1,0.2,0.3);
	EXPECT_EQ(0.1,point.x());
	EXPECT_EQ(0.2,point.y());
	EXPECT_EQ(0.3,point.z());
    Vector3d v; v << 0.1, 0.2, 0.3;
	CartesianPoint3d p2(v);
	EXPECT_EQ(0.1,p2.x());
	EXPECT_EQ(0.2,p2.y());
	EXPECT_EQ(0.3,p2.z());
}

TEST(CartesianPointTests,assignment) {
	CartesianPoint3d point;
	point << 1.0, 2.0, 3.0;
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
	point.x(1.0);
	point.y(2.0);
	point.z(3.0);
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
    Vector3d v; v << 1.0, 2.0, 3.0;
    point = v;
	EXPECT_EQ(1.0,point.x());
	EXPECT_EQ(2.0,point.y());
	EXPECT_EQ(3.0,point.z());
}

TEST(CartesianPointTests,vectorHandle) {
	CartesianPoint3d point;
	point << 1.0, 2.0, 3.0;
	const Vector3d& v = point.positionVector();
	EXPECT_EQ(1.0,v[0]);
	EXPECT_EQ(2.0,v[1]);
	EXPECT_EQ(3.0,v[2]);
}

TEST(CartesianPointTests,constAccessors) {
	CartesianPoint3d point;
	point << 1.0, 2.0, 3.0;
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


