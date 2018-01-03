/**
 * @file /ecl_time_lite/src/examples/time_functions.cpp
 *
 * @brief Ctest style coverage test for the ecl_time_lite functions.
 *
 * Not really feasible converting this into a gtest.
 *
 * @date February, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include <cstdlib>
#include <ecl/time_lite/config.hpp>
#include "../../include/ecl/time_lite/date.hpp"
#include "../../include/ecl/time_lite/functions.hpp"
#include "../../include/ecl/time_lite/cpu_time.hpp"


/*****************************************************************************
** Main
*****************************************************************************/

int main() {

#if defined(ECL_HAS_POSIX_TIMERS) || defined(ECL_HAS_RT_TIMERS) || defined(ECL_HAS_WIN_TIMERS) || defined(ECL_HAS_MACH_TIMERS)
	ecl::TimeError error;
    ecl::TimeStructure duration, timestamp;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Sleep" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;


    std::cout << "Sleep: Duration 1.3s" << std::endl;
    duration.tv_sec = 1;
    duration.tv_nsec = 300000000;
    error = ecl::sleep(duration);
    if ( error.flag() != ecl::NoError ) {
    	abort();
    }


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Epoch Time" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    error = ecl::epoch_time(timestamp);
    if ( error.flag() != ecl::NoError ) {
    	abort();
    }
    std::cout << "Epoch Time:" << std::endl;
    std::cout << "  sec : " << timestamp.tv_sec << std::endl;
    std::cout << "  nsec: " << timestamp.tv_nsec << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Sleep Until" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    timestamp.tv_sec += 2;
    std::cout << "Sleeping until current_time + 2 seconds." << std::endl;
    error = ecl::sleep_until(timestamp);
    if ( error.flag() != ecl::NoError ) {
    	abort();
    }

#ifdef ECL_HAS_CPUTIME
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Cpu Time" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    error = ecl::cpu_time(timestamp);
    if ( error.flag() != ecl::NoError ) {
    	abort();
    }
    std::cout << "Cpu Time:" << std::endl;
    std::cout << "  sec : " << timestamp.tv_sec << std::endl;
    std::cout << "  nsec: " << timestamp.tv_nsec << std::endl;
#endif

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Date String" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Date String: " << ecl::get_date_string() << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
#else
    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "           This platform does not support timers. " << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;
#endif

}
