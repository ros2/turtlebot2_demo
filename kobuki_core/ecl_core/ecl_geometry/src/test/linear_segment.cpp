/**
 * @file /src/test/linear_segment.cpp
 *
 * @ingroup UnitTests
 *
 * Use this to test the linear segment classes.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <ecl/math/constants.hpp>
#include "../../include/ecl/geometry/linear_segment.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(LinearSegment,construction) {
    ecl::LinearSegment horizontal_segment(0.0,1.0,4.0,1.0);
    EXPECT_EQ(0.0, horizontal_segment.A());
    EXPECT_EQ(1.0, horizontal_segment.B());
    EXPECT_EQ(-1.0, horizontal_segment.C());
    ecl::LinearSegment vertical_segment(2.0,1.0,2.0,4.0);
    EXPECT_EQ(1.0, vertical_segment.A());
    EXPECT_EQ(0.0, vertical_segment.B());
    EXPECT_EQ(2.0, vertical_segment.C());
    ecl::LinearSegment segment(0.0,1.0,1.0,2.0);
    EXPECT_EQ(-1.0, segment.A());
    EXPECT_EQ(1.0,  segment.B());
    EXPECT_EQ(-1.0, segment.C());
//    EXPECT_GT(0.786,f); // Allow some roundoff error here
//    EXPECT_LT(0.785,f);
}

TEST(LinearSegment,distance) {
    ecl::LinearSegment horizontal_segment(0.0,1.0,4.0,1.0);
    EXPECT_EQ(1.0, horizontal_segment.squaredDistanceFromPoint(0.0, 2.0));
    EXPECT_EQ(1.0, horizontal_segment.squaredDistanceFromPoint(-1.0, 1.0));
    EXPECT_EQ(0.0, horizontal_segment.squaredDistanceFromPoint(1.0, 1.0));
    EXPECT_EQ(1.0, horizontal_segment.squaredDistanceFromPoint(4.0, 0.0));
    ecl::LinearSegment vertical_segment(2.0,1.0,2.0,3.0);
    EXPECT_EQ(1.0, vertical_segment.squaredDistanceFromPoint(2.0, 0.0));
    EXPECT_EQ(1.0, vertical_segment.squaredDistanceFromPoint(1.0, 1.0));
    EXPECT_EQ(1.0, vertical_segment.squaredDistanceFromPoint(3.0, 1.0));
    EXPECT_EQ(1.0, vertical_segment.squaredDistanceFromPoint(1.0, 2.0));
    ecl::LinearSegment segment(0.0,1.0,1.0,2.0);
    EXPECT_EQ(1.0, segment.squaredDistanceFromPoint(1.0, 3.0));
    EXPECT_EQ(0.0, segment.squaredDistanceFromPoint(0.5, 1.5));

//    EXPECT_GT(0.786,f); // Allow some roundoff error here
//    EXPECT_LT(0.785,f);
}

// operator tests

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


