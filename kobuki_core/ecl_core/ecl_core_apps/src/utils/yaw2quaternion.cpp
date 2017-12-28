/**
 * @file /src/utils/yaw2quaternion.cpp
 *
 * @brief Angle converter.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/command_line.hpp>
#include <ecl/linear_algebra.hpp>

/*****************************************************************************
 * Using
 ****************************************************************************/

using ecl::ArgException;
using ecl::CmdLine;
using ecl::UnlabeledValueArg;

/*****************************************************************************
** Main program
*****************************************************************************/
int main(int argc, char** argv) {

    /******************************************
     * Parse for the port name
     ******************************************/
    bool hex(false);
    double yaw;

    try {
        CmdLine cmd("Calculator for yaw to quaternion.",' ',"0.1");
        UnlabeledValueArg<double> arg_yaw("yaw","Yaw angle to convert",true,0.0,"float", cmd);
        cmd.parse(argc,argv);
        yaw = arg_yaw.getValue();
    } catch ( ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
    Eigen::Quaternion<double> q;
    q = Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
    std::cout << "Quaternion: [x: " << q.x() << " y: " << q.y() << " z: " << q.z() << " w: " << q.w() << "]" << std::endl;

    return 0;
}
