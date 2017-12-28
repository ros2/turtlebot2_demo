/**
 * @file /ecl_core_apps/src/benchmarks/eigen3_decompositions.cpp
 *
 * @brief Benchmark the decompositions in Eigen.
 *
 * @date 05/08/2010
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

	const unsigned int repeats = 5;
	try {
		ecl::set_priority(ecl::RealTimePriority4);
	} catch ( StandardException &e ) {
		// dont worry about it.
	}
    StopWatch stopwatch;
    TimeStamp times[13];

	MatrixXf A = MatrixXf::Random(100,100);
	MatrixXf b = MatrixXf::Identity(100,100);
	MatrixXf x(100,1);

    // get rid of caching effects.
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	x = A.colPivHouseholderQr().solve(b);
    }

    times[0].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.colPivHouseholderQr().solve(b);
        times[0] += stopwatch.split();
    }
    times[1].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.fullPivHouseholderQr().solve(b);
        times[1] += stopwatch.split();
    }
    times[2].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.partialPivLu().solve(b);
        times[2] += stopwatch.split();
    }
    times[3].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.fullPivLu().solve(b);
        times[3] += stopwatch.split();
    }

    ecl::Format<double> format(15,9,ecl::RightAlign);
    std::cout << std::endl;
    std::cout << "************** Eigen3 Decompositions ***************" << std::endl;
    std::cout << std::endl;
    std::cout << "Ax=b where A = Rand(100,100) and b = Identity(100x100)" << std::endl;
    std::cout << "i.e. solving the inverse (results averaged over " << repeats << " runs)" << std::endl;
    std::cout << std::endl;
    std::cout << "ColPivHouseHolder  : " << format(static_cast<double>(times[0])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivHouseHolder : " << format(static_cast<double>(times[1])/static_cast<double>(repeats)) << std::endl;
    std::cout << "PartialPivLu       : " << format(static_cast<double>(times[2])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivLu          : " << format(static_cast<double>(times[3])/static_cast<double>(repeats)) << std::endl;
    std::cout << std::endl;

    b = MatrixXf::Random(100,1);
    times[4].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.colPivHouseholderQr().solve(b);
        times[4] += stopwatch.split();
    }
    times[5].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.fullPivHouseholderQr().solve(b);
        times[5] += stopwatch.split();
    }
    times[6].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.partialPivLu().solve(b);
        times[6] += stopwatch.split();
    }
    times[7].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	A = MatrixXf::Random(100,100);
        stopwatch.restart();
    	x = A.fullPivLu().solve(b);
        times[7] += stopwatch.split();
    }

    std::cout << "Ax=b where A = Rand(100,100) and b = Random(100x1)" << std::endl;
    std::cout << "i.e. solving linear equations (results averaged over " << repeats << " runs)" << std::endl;
    std::cout << std::endl;
    std::cout << "ColPivHouseHolder  : " << format(static_cast<double>(times[4])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivHouseHolder : " << format(static_cast<double>(times[5])/static_cast<double>(repeats)) << std::endl;
    std::cout << "PartialPivLu       : " << format(static_cast<double>(times[6])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivLu          : " << format(static_cast<double>(times[7])/static_cast<double>(repeats)) << std::endl;
    std::cout << std::endl;

    b = MatrixXf::Identity(100,100);
    times[8].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.colPivHouseholderQr().solve(b);
        times[8] += stopwatch.split();
    }
    times[9].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.fullPivHouseholderQr().solve(b);
        times[9] += stopwatch.split();
    }
    times[10].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.partialPivLu().solve(b);
        times[10] += stopwatch.split();
    }
    times[11].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.fullPivLu().solve(b);
        times[11] += stopwatch.split();
    }
    times[12].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.llt().solve(b);
//    	A.llt().solveInPlace(b); // only marginally faster
        times[12] += stopwatch.split();
    }
    times[13].stamp(0,0);
    for ( unsigned int i = 0; i < repeats; ++i ) {
    	MatrixXf D = MatrixXf::Random(100,100); // This is actually a bit dangerous
    	A = D.transpose()*D; // because this will only be positive definite if D is invertible
        stopwatch.restart();
    	x = A.ldlt().solve(b);
//    	A.ldlt().solveInPlace(b); // only marginally faster
        times[13] += stopwatch.split();
    }

    std::cout << "Ax=b where A = RandPosDef(100,100) and b = Identity(100x100)" << std::endl;
    std::cout << "(results averaged over " << repeats << " runs)" << std::endl;
    std::cout << std::endl;
    std::cout << "ColPivHouseHolder  : " << format(static_cast<double>(times[8])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivHouseHolder : " << format(static_cast<double>(times[9])/static_cast<double>(repeats)) << std::endl;
    std::cout << "PartialPivLu       : " << format(static_cast<double>(times[10])/static_cast<double>(repeats)) << std::endl;
    std::cout << "FullPivLu          : " << format(static_cast<double>(times[11])/static_cast<double>(repeats)) << std::endl;
    std::cout << "llt                : " << format(static_cast<double>(times[12])/static_cast<double>(repeats)) << std::endl;
    std::cout << "ldlt               : " << format(static_cast<double>(times[13])/static_cast<double>(repeats)) << std::endl;
    std::cout << std::endl;

	return 0;
}
