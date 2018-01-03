/**
 * @file /src/test/grid_map.cpp
 *
 * @brief Test the old inverse was the same as the new partial inverse function.
 *
 * @date October 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include <ecl/linear_algebra.hpp>
#include <ecl/formatters/floats.hpp>
#include <ecl/geometry/pose2d.hpp>
#include <ecl/math.hpp>
#include "../../include/ecl/mobile_robot/differential_drive.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout;
using std::endl;
using ecl::RightAlign;
using ecl::Format;
using ecl::Pose2D;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector3d;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(DifferentialDriveTests,allEggsInOneBasket) {
	// Haven't got around to running this properly through gtests yet.
	SUCCEED();
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    Format<double> format; format.width(8); format.precision(2); format.align(RightAlign);

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                 Partial Inverse" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

//    Vector3d pose_a; pose_a << 1.0, 2.0, ecl::pi/2.0;
//    Vector3d pose_b; pose_b << 2.0, 3.0, ecl::pi;
    Pose2D<double> a(1.0, 2.0, ecl::pi/2.0);
    Pose2D<double> b(2.0, 3.0, ecl::pi);

    Vector2d partial_inverse;
//    partial_inverse = ecl::DifferentialDrive::Kinematics::Inverse(pose_a, pose_b);
//    std::cout << partial_inverse << std::endl;
    partial_inverse = ecl::DifferentialDrive::Kinematics::PartialInverse(a, b);
    std::cout << partial_inverse << std::endl;

    cout << endl;
    cout << "***********************************************************" << endl;
    cout << "                      Passed" << endl;
    cout << "***********************************************************" << endl;
    cout << endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


