/**
 * @file src/examples/sleep.cpp
 *
 * @brief Demos the sleep functions.
 *
 * @date April, 2011
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/time/sleep.hpp"

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Sleep" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ecl::Duration duration(1,300000000);
    ecl::MilliSleep sleep;
    ecl::MilliSleep sleep_ms;
    ecl::MicroSleep sleep_us;
    ecl::NanoSleep sleep_ns;

    std::cout << "Sleep: Duration 1s 300000000ns" << std::endl;
    sleep(duration);
    std::cout << "Sleep: 1s" << std::endl;
    sleep(1);
    std::cout << "MilliSleep: 500 x 2 = 1s" << std::endl;
    for ( int i = 0; i < 2; ++i ) { sleep_ms(500); }
    std::cout << "MicroSleep: 500,000 x 2 = 1s" << std::endl;
    for ( int i = 0; i < 2; ++i ) { sleep_us(500000); }
    std::cout << "NanoSleep:  500,000,000 x 2 = 1s" << std::endl;
    for ( int i = 0; i < 2; ++i ) { sleep_ns(500000000); }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Pre Configured Sleep" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ecl::Sleep sleep_1_3s(duration);
    std::cout << "Sleep" << std::endl;
    sleep_1_3s();
    ecl::MilliSleep sleep_1400ms(1400);
    std::cout << "MilliSleep" << std::endl;
    sleep_1400ms();
    ecl::MicroSleep sleep_1400us(1400);
    std::cout << "MicroSleep" << std::endl;
    sleep_1400us();
    ecl::NanoSleep sleep_1400ns(1400);
    std::cout << "NanoSleep" << std::endl;
    sleep_1400ns();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}
