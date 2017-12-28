/**
 * @file /src/benchmarks/streams.cpp
 *
 * @brief Benchmark streams to standard output.
 *
 * Benchmark various means of streaming to standard output.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/streams.hpp>
#include <ecl/formatters.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::OConsoleStream;
using ecl::StopWatch;
using ecl::TimeStamp;
using ecl::Format;
using ecl::StandardException;

/*****************************************************************************
** Main
*****************************************************************************/

int main() {
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}

    StopWatch stopwatch;
	TimeStamp times[6];
	unsigned int lines_to_write = 1;
    float f = 33.54235235;
    Format<double> format; format.precision(2); format.width(5);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Printf" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	printf("Heya Dude\n");
    }
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	printf("Heya Dude\n");
    }
    times[0] = stopwatch.split();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	printf("%4.2f \n",f);
    }
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	printf("%4.2f \n",f);
    }
    times[3] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Cout" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	std::cout << "Heya Dude\n";
    }
    std::cout.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	std::cout << "Heya Dude\n";
    }
    std::cout.flush();
    times[1] = stopwatch.split();

    std::cout.precision(4);
    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	std::cout << f << " \n";
    }
    std::cout.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	std::cout << f << " \n";
    }
    std::cout.flush();
    times[4] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Ostream" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    OConsoleStream ostream;
    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	ostream << "Heya Dude\n";
    }
    ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	ostream << "Heya Dude\n";
    }
    ostream.flush();
    times[2] = stopwatch.split();

    for ( unsigned int i = 0; i < lines_to_write; ++i ) { // Avoid speedups from caching affecting times.
    	ostream << format(f) << " \n";
    }
    ostream.flush();
    stopwatch.restart();
    for ( unsigned int i = 0; i < lines_to_write; ++i ) {
    	ostream << format(f) << " \n";
    }
    ostream.flush();
    times[5] = stopwatch.split();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Times" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Writing Char Strings:" << std::endl;
    std::cout << "           printf : " << times[0].nsec() << " ns" << std::endl;
    std::cout << "             cout : " << times[1].nsec() << " ns" << std::endl;
    std::cout << "   OConsoleStream : " << times[2].nsec() << " ns" << std::endl;
    std::cout << "Streaming Floats:" << std::endl;
    std::cout << "           printf : " << times[3].nsec() << " ns" << std::endl;
    std::cout << "             cout : " << times[4].nsec() << " ns" << std::endl;
    std::cout << "   OConsoleStream : " << times[5].nsec() << " ns" << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}
