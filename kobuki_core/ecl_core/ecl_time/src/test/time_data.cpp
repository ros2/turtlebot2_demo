/**
 * @file /src/test/timestamp.cpp
 *
 * @brief Unit Test for timestamp objects.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <gtest/gtest.h>
#include "../../include/ecl/time/time_data.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Duration;
using ecl::TimeData;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(TimeDataTests,container) {
    TimeData time_data;
    Duration duration;
    duration.stamp(1,0);
    time_data.push_back(duration);
    time_data.push_back(duration);
    time_data.clear();
    SUCCEED();
}

TEST(TimeDataTests,statistics) {
    TimeData time_data;
    Duration duration;
    duration.stamp(1,0);
    time_data.push_back(duration);
    duration.stamp(2,0);
    time_data.push_back(duration);
    double avg = time_data.average();
    EXPECT_LT(1.49,avg); // Allow for some roundoff error.
    EXPECT_GT(1.51,avg);
    double std_dev = time_data.stdDev();
    EXPECT_LT(0.49,std_dev); // Allow for some roundoff error.
    EXPECT_GT(0.51,std_dev);
    double variance = time_data.variance();
    EXPECT_LT(0.24,variance); // Allow for some roundoff error.
    EXPECT_GT(0.26,variance);
}
/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
