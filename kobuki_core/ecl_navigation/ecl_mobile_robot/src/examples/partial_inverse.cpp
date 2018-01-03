/**
 * @file src/examples/partial_inverse.cpp
 *
 * @brief Demos the mobile robot functions.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/linear_algebra.hpp>
#include <ecl/formatters/floats.hpp>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/math.hpp>
#include "../../include/ecl/mobile_robot/differential_drive.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout;
using std::endl;
using ecl::RightAlign;
using ecl::Format;
using ecl::LegacyPose2D;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Vector3d;

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
    LegacyPose2D<double> a(1.0, 2.0, ecl::pi/2.0);
    LegacyPose2D<double> b(2.0, 3.0, ecl::pi);

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

    return 0;
}
