/**
 * @file /ecl_core_apps/src/benchmarks/eigen3_inverse.cpp
 *
 * @brief Tests eigen3 inverses and decompositions.
 *
 * @date Aug 4, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/linear_algebra.hpp>
#include <ecl/threads/priority.hpp>
#include <ecl/time/stopwatch.hpp>
#include <ecl/time/timestamp.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/formatters.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StandardException;
using ecl::StopWatch;
using ecl::TimeStamp;
using Eigen::Matrix3f;
using Eigen::MatrixXf;

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	const unsigned int repeats = 10;
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}
    StopWatch stopwatch;
    TimeStamp times[9];

    Matrix3f M3 = Matrix3f::Random();
    Matrix3f M3I;
    MatrixXf M100 = MatrixXf::Random(100,100);
    MatrixXf M100I;

    // get rid of caching effects.
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	M3.inverse();
    }

    times[0].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	M3 = Matrix3f::Random();
        stopwatch.restart();
    	M3I = M3.inverse();
        times[0] += stopwatch.split();
    }
    times[1].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	M100 = MatrixXf::Random(100,100);
        stopwatch.restart();
    	M100I = M100.inverse();
        times[1] += stopwatch.split();
    }
    times[2].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	M100 = MatrixXf::Random(100,100);
        stopwatch.restart();
    	M100I = M100.fullPivLu().inverse();
        times[2] += stopwatch.split();
    }

    ecl::Format<double> format(15,9,ecl::RightAlign);
    std::cout << std::endl;
    std::cout << "************** Eigen3 Inverse ***************" << std::endl;
    std::cout << std::endl;
    std::cout << "   3x3   (direct by val) : " << format(static_cast<double>(times[0])/static_cast<double>(repeats)) << std::endl;
    std::cout << " 100x100 (PPivLU by val) : " << format(static_cast<double>(times[1])/static_cast<double>(repeats)) << std::endl;
    std::cout << " 100x100 (FPivLU by val) : " << format(static_cast<double>(times[2])/static_cast<double>(repeats)) << std::endl;
    std::cout << std::endl;

	return 0;
}
