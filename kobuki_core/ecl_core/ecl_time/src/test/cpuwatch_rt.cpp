/**
 * @file /src/test/cpuwatch_rt.cpp
 *
 * @brief Unit Test for the CpuWatch class.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/time/cpuwatch.hpp"

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

#if defined(ECL_IS_POSIX)
  // monotonic clock, cpu clock -> clock_gettime; clock_selection -> clock_nanosleep
  #if defined(_POSIX_MONOTONIC_CLOCK) && (_POSIX_MONOTONIC_CLOCK) >= 0L && defined(_POSIX_CLOCK_SELECTION) && (_POSIX_CLOCK_SELECTION) >= 0L

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::CpuWatch;
using ecl::TimeStamp;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(StopwatchTests,silent) {

	CpuWatch stopwatch;
	TimeStamp time;

	time = stopwatch.split();
	time = stopwatch.split();
	time = stopwatch.elapsed();
	// Can't really verify stuff, so this is just a unit test to make sure
	// it compiles.
    SUCCEED();
}

  #endif /* MANY POSIX TIME REQ'MENTS */
#endif /* ECL_IS_POSIX */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

