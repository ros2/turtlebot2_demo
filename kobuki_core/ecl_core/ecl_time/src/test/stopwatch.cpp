/**
 * @file /src/test/stopwatch.cpp
 *
 * @brief Unit Test for the StopWatch class.
 *
 * @date July, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/time/timestamp.hpp"
#include "../../include/ecl/time/stopwatch.hpp"

/*****************************************************************************
** Platform Check
*****************************************************************************/

#ifdef ECL_HAS_TIMESTAMP


/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StopWatch;
using ecl::TimeStamp;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(StopwatchTests,silent) {

	StopWatch stopwatch;
	TimeStamp time;

	time = stopwatch.split();
	time = stopwatch.split();
	time = stopwatch.elapsed();

    SUCCEED();
}

#endif /* ECL_HAS_TIMESTAMP */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
