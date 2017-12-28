/**
 * @file /src/utils/quaternion2yaw.cpp
 *
 * @brief Angle converter.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/command_line.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/math.hpp>

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
    double x, y, z, w;

    try {
        CmdLine cmd("Calculator for yaw to quaternion.",' ',"0.1");
        UnlabeledValueArg<double> arg_x("x","Quaternion component - x",true,0.0,"float", cmd);
        UnlabeledValueArg<double> arg_y("y","Quaternion component - y",true,0.0,"float", cmd);
        UnlabeledValueArg<double> arg_z("z","Quaternion component - z",true,0.0,"float", cmd);
        UnlabeledValueArg<double> arg_w("w","Quaternion component - w",true,0.0,"float", cmd);
        cmd.parse(argc,argv);
        x = arg_x.getValue();
        y = arg_y.getValue();
        z = arg_z.getValue();
        w = arg_w.getValue();
    } catch ( ArgException &e ) {
        std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    }
    if ( ( x != 0.0 ) && ( y != 0.0 ) ) {
      std::cout << "This quaternion has non-yaw components, aborting." << std::endl;
    }
    // This is a very naive way of calculating - it is only for the case where it is a
    // rotation around a z-axis.
    double rads = 2*acos(w)* ecl::sign(z);
    double degrees = rads * 360.0 / (2 * ecl::pi);
    std::cout << "Quaternion {x: " << x << " y: " << y << " z: " << z << " w: " << w << "} -> ";
    std::cout << rads << " radians -> " << degrees << " degrees." << std::endl;

    return 0;
}
