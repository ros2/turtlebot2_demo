/**
 * @file /src/benchmarks/containers.cpp
 *
 * @brief Benchmarks the performance of various container types.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <algorithm>
#include <iostream>
#include <vector>
#include <ecl/containers/array.hpp>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/timestamp.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using std::vector;
using ecl::Array;
using ecl::RealTimePriority4;
using ecl::StandardException;
using ecl::StopWatch;
using ecl::TimeStamp;

/*****************************************************************************
** Main
*****************************************************************************/


int main()
{
	try {
		ecl::set_priority(RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}
    StopWatch stopwatch;
    TimeStamp timestamp[5];

    Array<int,4> array_tmp;
    array_tmp << 3,4,6,3;
    array_tmp << 3,4,6,3;
    array_tmp << 3,4,6,3;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Constructors" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    stopwatch.restart();
    // Array [manual]
    Array<int,4> array1;
    for ( int i = 0; i < 4; ++i ) {
        array1[i] = 3;
    }
    timestamp[0] = stopwatch.split();

    // Array [comma]
    Array<int,4> array2;
    array2 << 3,3,3,3;
    timestamp[1] = stopwatch.split();

    // Array [blueprint]
    Array<int,4> array3 = Array<int,4>::Constant(3);
    timestamp[2] = stopwatch.split();

    // Vector [manual]
    vector<int> v1(4);
    for ( int i = 0; i < 4; ++i ) {
        v1[i] = 3;
    }
    timestamp[3] = stopwatch.split();

    // Vector [manual]
    int a1[4];
    for ( int i = 0; i < 4; ++i ) {
        a1[i] = 3;
    }
    timestamp[4] = stopwatch.split();

    std::cout << "carray [manual]   : " << timestamp[4] << std::endl;
    std::cout << "Array  [blueprint]: " << timestamp[2] << std::endl;
    std::cout << "Array  [manual]   : " << timestamp[0] << std::endl;
    std::cout << "Array  [comma]    : " << timestamp[1] << std::endl;
    std::cout << "Vector [manual]   : " << timestamp[3] << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Accessors" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    int elements[4];

    stopwatch.restart();
    for (int i = 0; i < 4; ++i ) { elements[i] = array1[i]; }
    timestamp[0] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { elements[i] = array1.at(i); }
    timestamp[1] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { elements[i] = v1[i]; }
    timestamp[2] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { elements[i] = v1.at(i); }
    timestamp[3] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { elements[i] = a1[i]; }
    timestamp[4] = stopwatch.split();

    std::cout << "carray []   : " << timestamp[4] << std::endl;
    std::cout << "Array  []   : " << timestamp[0] << std::endl;
    std::cout << "Array  at   : " << timestamp[1] << std::endl;
    std::cout << "Vector []   : " << timestamp[2] << std::endl;
    std::cout << "Vector at   : " << timestamp[3] << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                    Setters" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    stopwatch.restart();
    for (int i = 0; i < 4; ++i ) { array1[i] = 3; }
    timestamp[0] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { array1 = Array<int,4>::Constant(3); }
    timestamp[1] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { v1[i] = 3; }
    timestamp[2] = stopwatch.split();

    for (int i = 0; i < 4; ++i ) { a1[i] = 3; }
    timestamp[3] = stopwatch.split();

    std::cout << "carray [manual]   : " << timestamp[3] << std::endl;
    std::cout << "Array  [blueprint]: " << timestamp[1] << std::endl;
    std::cout << "Array  [manual]   : " << timestamp[0] << std::endl;
    std::cout << "Vector [manual]   : " << timestamp[2] << std::endl;

return 0;
}

