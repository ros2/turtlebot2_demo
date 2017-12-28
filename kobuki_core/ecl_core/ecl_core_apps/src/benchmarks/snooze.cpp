/**
 * @file /src/benchmarks/snooze.cpp
 *
 * @brief Benchmark for the absolute periodic timers (snoozers).
 *
 * @date December 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cmath>
#include <ecl/threads/priority.hpp>
#include <ecl/time_lite/functions.hpp>
#include <ecl/time/snooze.hpp>
#include <ecl/time/timestamp.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace benchmarks {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Duration;
using ecl::Snooze;
using ecl::StandardException;

/*****************************************************************************
** Classes
*****************************************************************************/

class LogSnooze : public Snooze {

public:
	LogSnooze(const long &ms) : Snooze(Duration(0,ms*1000000L)), count(0), avg(0.0), avg_noise(0.0), max(0), period_ms(ms) {};

	// Need to run this at least 10 times before the average is close
	// to correct.
	void log() {
		ecl::epoch_time(time_actual);
		latency = (time_actual.tv_nsec - time_value.tv_nsec)/1000000.0; // convert to ms
		if ( count == 0 ) {
			avg = latency;
		} else {
			avg = 0.9*avg + 0.1*latency;
			avg_noise = 0.9*avg_noise + 0.1*fabs(latency-avg);
		}
		if ( latency > max ) { max = latency; }
		++count;
	}

	void printLog() {
		std::cout << "Loop period [ms]:     " << period_ms << std::endl;
		std::cout << "Average latency [ms]: " << avg << std::endl;
		std::cout << "Maximum latency [ms]: " << max << std::endl;
	}

private:
	/*********************
	** Logging Variables
	**********************/
	TimeStructure time_actual;
	double latency;
	int count;
	double avg,avg_noise;
	double max;
	long period_ms;
};


} // namespace benchmarks
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::benchmarks;

/*****************************************************************************
** Main program
*****************************************************************************/

int main() {

	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Snooze Latency" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "* " << std::endl;
    std::cout << "* Latency is the absolute time behind where it should be." << std::endl;
    std::cout << "* This is only a factor for the first snooze loop." << std::endl;
    std::cout << "*" << std::endl;
    std::cout << "* Latency noise is more important. After the first loop," << std::endl;
    std::cout << "* with an ideal noise value of zero, the loop will time" << std::endl;
    std::cout << "* exactly as you specify. Practically, this is not the case," << std::endl;
    std::cout << "* but you should get good results from this without having" << std::endl;
    std::cout << "* to use system timers. Use with loops which need to be" << std::endl;
    std::cout << "* 5ms or over. Faster loops really require a RTOS." << std::endl;
    std::cout << "* " << std::endl;
    std::cout << "*****************************************************************" << std::endl;
    std::cout << std::endl;
    std::cout << "Looping commencing..." << std::endl;

    LogSnooze snooze(30);
    snooze.initialise(); // Get it ready to start looping.
    for(int i = 0; i < 100; ++i ) {
        snooze();
        snooze.log();
    }
    snooze.printLog();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}


