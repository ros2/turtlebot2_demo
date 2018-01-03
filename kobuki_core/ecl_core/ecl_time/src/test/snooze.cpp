/**
 * @file /src/test/snooze.cpp
 *
 * @brief Unit Test for the snoozer.
 *
 * @date April 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

//#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/time/snooze.hpp"
#include "../../include/ecl/time/timestamp.hpp"

/*****************************************************************************
** Platform Check
*****************************************************************************/

// Only posix implementation so far
#include <ecl/config.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Duration;
using ecl::Snooze;
using ecl::TimeStamp;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SnoozeTests,snoozeConfigured) {
	// should check for exceptions here?
	Duration duration(0.5);
	Snooze snooze;
	snooze.period(duration);
	Snooze snooze_from_duration(duration);
	Snooze snooze_with_validation(duration, true);

    SUCCEED();
}

TEST(SnoozeTests,setGetPeriod) {
	Duration duration(0.5);
	Snooze snooze;
	snooze.period(duration);
    double period = snooze.period();
    EXPECT_FLOAT_EQ(0.5,period);
}

TEST(SnoozeTests,snooze) {
	Duration duration(0.5);
	Snooze snooze(duration);
	TimeStamp start, finish;
	snooze.initialise();
	snooze();
	finish.stamp();
	double difference = finish - start;
    EXPECT_LT(0.5, difference);
    EXPECT_GT(1.0, difference);
}

#endif /* ECL_IS_POSIX */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


