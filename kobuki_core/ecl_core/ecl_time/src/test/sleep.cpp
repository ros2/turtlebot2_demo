/**
 * @file /src/test/sleep.cpp
 *
 * @brief Unit Test for the sleepers.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

//#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/time/sleep.hpp"
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
using ecl::Sleep;
using ecl::MilliSleep;
using ecl::MicroSleep;
using ecl::NanoSleep;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SleepTests,sleepConfigured) {

	Duration duration(1,300000000);
    Sleep sleep;
    MilliSleep sleep_ms;
    MicroSleep sleep_us;
    NanoSleep sleep_ns;

    sleep(duration);
    sleep(1);
    sleep_ms(500);
    sleep_us(500000);
    sleep_ns(500000000);

    SUCCEED();
}

TEST(SleepTests,sleepPreConfigured) {
	Duration duration(1,300000000);
    Sleep sleep_1_3s(duration);
    sleep_1_3s();
    MilliSleep sleep_1400ms(1400);
    sleep_1400ms();
    MicroSleep sleep_1400us(1400);
    sleep_1400us();
    NanoSleep sleep_1400ns(1400);
    sleep_1400ns();
    SUCCEED();
}

#endif /* ECL_IS_POSIX */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


