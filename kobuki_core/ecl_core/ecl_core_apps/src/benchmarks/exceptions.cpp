/**
 * @file /src/benchmarks/exceptions.cpp
 *
 * @brief Benchmarks the performance of ecl exceptions
 *
 * @date September 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <iostream>
#include <sstream>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/timestamp.hpp>
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::StopWatch;
using ecl::TimeStamp;
using ecl::ConfigurationError;
using ecl::StandardException;

/*****************************************************************************
** Globals
*****************************************************************************/

void f1() {
	int i = 0;
	if ( i == 1 ) {}
}

void f2() throw(StandardException) {
	int i = 0;
	if ( i == 1 ) {}
}

void f3() throw(StandardException) {
	int i = 0;
	if ( i == 1 ) {
		throw StandardException(LOC,ConfigurationError);
	}
}

void f4() throw(StandardException) {
	int i = 0;
	if ( i == 1 ) {
		throw StandardException(LOC,ConfigurationError, "Standard exception with an extra string message.");
	}
}

void f5() throw(StandardException) {
	throw StandardException(LOC,ConfigurationError);
}

void f6() throw(StandardException) {
	throw StandardException(LOC,ConfigurationError, "Standard exception with an extra string message.");
}

void g1() throw(StandardException) {
	double dude[5];
	for ( unsigned int j = 0; j < 5; ++j ) {
		dude[j] = 3.15*543/230.235;
	}
}

void g2() throw(StandardException) {
	int i = 0;
	double dude[5];
	for ( unsigned int j = 0; j < 5; ++j ) {
		dude[j] = 3.15*543/230.235;
	}
	if ( i == 1 ) {
		throw StandardException(LOC,ConfigurationError, "Standard exception with an extra string message.");
	}
}

/*****************************************************************************
** Main
*****************************************************************************/

int main()
{
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}
    StopWatch stopwatch;
    TimeStamp times[9];

    stopwatch.restart();
    f1();
    times[0] = stopwatch.split();
    f2();
    times[1] = stopwatch.split();
    f3();
    times[2] = stopwatch.split();
    f4();
    times[3] = stopwatch.split();
    g1();
    times[4] = stopwatch.split();
    g2();
    times[5] = stopwatch.split();
    try {
    	f2();
    } catch ( StandardException &e ) {
    }
    times[6] = stopwatch.split();
    try {
    	f5();
    } catch ( StandardException &e ) {}
    times[7] = stopwatch.split();
    try {
    	f6();
    } catch ( StandardException &e ) {}
    times[8] = stopwatch.split();

    // Try and negate the effects of caching on early instructions (i.e. get right into cache).
    stopwatch.restart();
    f1();
    times[0] = stopwatch.split();
    f2();
    times[1] = stopwatch.split();
    f3();
    times[2] = stopwatch.split();
    f4();
    times[3] = stopwatch.split();
    g1();
    times[4] = stopwatch.split();
    g2();
    times[5] = stopwatch.split();

    try {
    	f2();
    } catch ( StandardException &e ) {
    }
    times[6] = stopwatch.split();
    try {
    	f5();
    } catch ( StandardException &e ) {}
    times[7] = stopwatch.split();
    try {
    	f6();
    } catch ( StandardException &e ) {}
    times[8] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "         Performance comparison ecl exceptions" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Simple Candidate Function      " << " Time: " << times[0] <<  std::endl;
    std::cout << "  Undeclared exception         " << " Time: " << times[1] <<  std::endl;
    std::cout << "  Unused exception             " << " Time: " << times[2] <<  std::endl;
    std::cout << "  Unused exception w/ string   " << " Time: " << times[3] <<  std::endl;
    std::cout << std::endl;
    std::cout << "Complex Candidate Function     " << " Time: " << times[4] <<  std::endl;
    std::cout << "  Unused exception w/ string   " << " Time: " << times[5] <<  std::endl;
    std::cout << std::endl;
    std::cout << "Try Catch Blocks  " << std::endl;
    std::cout << "  Uncaught exceptions          " << " Time: " << times[6] <<  std::endl;
    std::cout << "  Caught exception             " << " Time: " << times[7] <<  std::endl;
    std::cout << "  Caught exception w/string    " << " Time: " << times[8] <<  std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Conclusions" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "The following conclusions are after testing on a dual core." << std::endl;
    std::cout << std::endl;
    std::cout << " - In simple functions, unused ecl exceptions < 50ns." << std::endl;
    std::cout << " - String messages don't effect the cost of unused ecl exceptions." << std::endl;
    std::cout << " - Try-catch blocks have negligible cost." << std::endl;
    std::cout << " - String messages do impact caught messages ~1us." << std::endl;
    std::cout << std::endl;
    return 0;
}


