/**
 * @file /src/test/angles.cpp
 *
 * @ingroup UnitTests
 *
 * Use this to test the angle classes.
 *
 * @date February 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <ecl/math/constants.hpp>
#include "../../include/ecl/geometry/angle.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::pi;
using ecl::Angle;
using ecl::wrap_angle;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(AngleTests,setters) {
    Angle<double> angle;
    double d = angle; // default
    EXPECT_EQ(0.0,d);
    angle = Angle<double>::Degrees(45.0);
    d = angle;
    EXPECT_GT(0.786,d); // Allow some roundoff error here
    EXPECT_LT(0.785,d);
    angle = Angle<double>::Radians(0.3);
    d = angle;
    EXPECT_EQ(0.3,d);
    Angle<float> angle_f;
    float f = angle_f; // default
    EXPECT_EQ(0.0,f);
    angle_f = Angle<float>::Degrees(45.0);
    f = angle_f;
    EXPECT_GT(0.786,f); // Allow some roundoff error here
    EXPECT_LT(0.785,f);
    angle_f = Angle<float>::Radians(0.3);
    f = angle_f;
    EXPECT_EQ(0.3,d);
}

TEST(AngleTests,conversions) {
    Angle<double> angle(0.3);
    double d = angle.degrees();
    EXPECT_GT(17.19,d); // Allow some roundoff error here
    EXPECT_LT(17.18,d);
    Angle<float> angle_f(0.3);
    float f = angle.degrees();
    EXPECT_GT(17.19,f); // Allow some roundoff error here
    EXPECT_LT(17.18,f);
}

TEST(AngleTests,wrap) {
	double angle = 2*pi+0.1;
    wrap_angle(angle);
    EXPECT_GT(0.11,angle); // Allow some roundoff error here
    EXPECT_LT(0.09,angle);
	angle = 2*pi+0.1;
    double wa = wrap_angle(angle);
    EXPECT_GT(0.11,wa); // Allow some roundoff error here
    EXPECT_LT(0.09,wa);
	float angle_f = 2*pi+0.1;
    wrap_angle(angle_f);
    EXPECT_GT(0.11,angle_f); // Allow some roundoff error here
    EXPECT_LT(0.09,angle_f);
	angle_f = 2*pi+0.1;
    float wa_f = wrap_angle(angle_f);
    EXPECT_GT(0.11,wa_f); // Allow some roundoff error here
    EXPECT_LT(0.09,wa_f);
}

TEST(AngleTests,assignment) {
    Angle<double> angle;
    angle = 0.3;
    double d = angle;
    EXPECT_GT(0.31,d); // Allow some roundoff error here
    EXPECT_LT(0.29,d);
    Angle<float> angle_f;
    angle_f = 0.3;
    float f = angle_f;
    EXPECT_GT(0.31,f); // Allow some roundoff error here
    EXPECT_LT(0.29,f);
}

// operator tests

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


