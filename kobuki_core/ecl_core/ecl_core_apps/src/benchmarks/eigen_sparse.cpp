/**
 * @file /ecl_core_apps/src/benchmarks/eigen_sparse.cpp
 *
 *
 * @brief Benchmark sparse matrix operations for Eigen (ecl_linear_algebra).
 *
 * @date August 2010
 **/
/*****************************************************************************
** Preprocessing
*****************************************************************************/

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <ecl/time.hpp>
#include <ecl/threads/priority.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/linear_algebra/sparse.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::linear_algebra::SparseMatrix;
using ecl::linear_algebra::DynamicSparseMatrix;
using ecl::linear_algebra::MatrixXd;
using ecl::StopWatch;

/*****************************************************************************
** Main
*****************************************************************************/
#define LM (303)
void test_sparse_matrix( int num )
{
	DynamicSparseMatrix<double> hh(20, LM);
	MatrixXd P(LM,LM);
	hh.setZero();
	P.setZero();
	for( int i=0; i<20; i++ )
		for( int j=0; j<3; j++ )
			hh.coeffRef(i,j) = 1;

	for( int i=0; i<10; i++)
	{
		for( int j=0; j<2; j++ )
		{
			for( int k=0; k<3; k++ )
			{
				hh.coeffRef( i*2+j, (i*4)+k ) = 1;
			}
		}
	}

	SparseMatrix<double> H(hh);
	ecl::StopWatch time;
	for( int i=0; i<num; i++ )
	{
		MatrixXd S = H*P*H.transpose();
		S(4,4)= i;
	}
	std::cout << time.elapsed();
}

void test_dense_matrix( int num )
{
	MatrixXd H(20, LM);
	MatrixXd P(LM,LM);
	H.setZero();
	P.setZero();
	for( int i=0; i<20; i++ )
		for( int j=0; j<3; j++ )
			H(i,j) = 1;

	for( int i=0; i<10; i++)
	{
		for( int j=0; j<2; j++ )
		{
			for( int k=0; k<3; k++ )
			{
				H( i*2+j, (i*4)+k ) = 1;
			}
		}
	}

	ecl::StopWatch time;
	for( int i=0; i<num; i++ )
	{
		MatrixXd S = H*P*H.transpose();
		S(4,4)= i;
	}
	std::cout << time.elapsed();
}

void compare_multiplication()
{
	std::cout << "==========================" << std::endl;
	std::cout << "S=H P T'" << std::endl;
	std::cout << "Start sparse: ";
	test_sparse_matrix(1000);
	std::cout  << std::endl;

	std::cout << "Start dense: ";
	test_dense_matrix(1000);
	std::cout  << std::endl;
	std::cout << "==========================" << std::endl;
}

void fill_sparse( int num )
{
	ecl::StopWatch time;
	DynamicSparseMatrix<double> hh(20, LM);
	for( int i=0; i<num; i++)
	{
		for( int j=0; j<20; j++ )
			for( int k=0; k<LM; k++ )
				hh.coeffRef(j,k) = (double)j*k;
	}

	std::cout << time.elapsed();
}

void fill_dense( int num )
{
	ecl::StopWatch time;
	MatrixXd hh(20, LM);
	for( int i=0; i<num; i++ )
	{
		for( int j=0; j<20; j++ )
			for( int k=0; k<LM; k++ )
				hh(j,k) = (double)j*k;
	}

	std::cout << time.elapsed();
}

void compare_filling()
{
	std::cout << "==========================" << std::endl;
	std::cout << "Filling " << std::endl;
	std::cout << "fill sparse: ";
	fill_sparse(1000);
	std::cout << std::endl;
	std::cout << "fill dense: ";
	fill_dense(1000);
	std::cout << std::endl;
	std::cout << "==========================" << std::endl;
}

int main(int argc, char **argv) {

	try {
	    ecl::set_priority(ecl::RealTimePriority4);
	} catch ( ecl::StandardException &e ) {
		// dont worry about it.
	}

	compare_multiplication();
	compare_filling();

	return 0;
}
