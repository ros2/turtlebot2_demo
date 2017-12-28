/**
 * @file /src/test/fifo.cpp
 *
 * @brief Unit Test for ecl::FiFo containers.
 *
 * @date Aug, 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include "../../include/ecl/containers/array.hpp"
#include "../../include/ecl/containers/fifo.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Array;
using ecl::StandardException;
using ecl::ContainerConcept;
using ecl::FiFo;
/*****************************************************************************
** Tests
*****************************************************************************/
TEST(FiFoTest,constructors)
{
	FiFo<double> fifo(4, 2.0);
	EXPECT_EQ(2.0, fifo[0]);
	EXPECT_EQ(2.0, fifo[1]);
	EXPECT_EQ(2.0, fifo[2]);
	EXPECT_EQ(2.0, fifo[3]);
}

TEST(FiFoTest, poping )
{
	FiFo<double> fifo(4);
	for( int i=0; i<4; i++ )
	{
		fifo.push_back( (double)i );
	}

	EXPECT_EQ(0.0, fifo[0]);
	EXPECT_EQ(1.0, fifo[1]);
	EXPECT_EQ(2.0, fifo[2]);
	EXPECT_EQ(3.0, fifo[3]);
}

/**
 * @cond DO_NOT_DOXYGEN
 */
class movingAvg
{
public:
	movingAvg( const unsigned int widowsSize ) :
		sum(0.0),
		average(0.0),
		windows_size(widowsSize),
		first(true)
	{
		fifo.resize( windows_size );
	}
	void reset()
	{
		fifo.fill(0);
		sum = 0;
		average = 0.0;
	}

	void update( const double & incomingData )
	{
		if( first )
		{
			fifo.fill( incomingData );
			first = false;
		}

		sum -= fifo[0];
		sum += incomingData;
		fifo.push_back( incomingData );
		average = sum/(double)(windows_size);
	}
	double sum;
	double average;
	unsigned int windows_size;
	FiFo<double> fifo;
	bool first;
};

/**
 * @endcond
 */


TEST(FiFoTest, application )
{
	movingAvg mavg( 10 );

	for( int i=0; i<50; i++ )
	{
		mavg.update( (double)i );
//		std::cout << mavg.average << std::endl;
	}

	EXPECT_EQ(44.5, mavg.average );
}

/**
 * @cond DO_NOT_DOXYGEN
 */

/*
 * @class
 * treats differential of Gaussian. Usual objective is to detect the significant change of data incoming.
 * you need just declaration of this class. and then please call update(..) function with your instance incoming data.
 * it will return result of Differential of Gaussian.
 *
 * @note
 * now, it build the kernel in contructor with hard coding. however we can formulize later.
 */
template<typename N, int KernelSize>
class DifferentialOfGaussian1D
{
public:
	DifferentialOfGaussian1D()
	{
		buildKernel();
		invalidate();
	}

	bool updateData( const N & incomingDatum, N & resultOfDOG )
	{
		resultOfDOG = static_cast<N>(0);
		data.push_back( incomingDatum );
		for( int i=0; i<KernelSize; i++ )
		{
			resultOfDOG += (data[i]*kernel[i]);
		}

		if( number_of_incoming < 5 )
		{
			number_of_incoming++;
			return false;
		}

		return true;
	}

	void invalidate()
	{
		number_of_incoming = 0;
	}

private:
	FiFo <float> data;
	Array<N> kernel;
	int number_of_incoming;

	void buildKernel()
	{
		//build kernel
		switch( KernelSize )
		{
		case 5:
			data.resize(KernelSize);
			kernel.resize(KernelSize);
			kernel[0] = 0.1179f;
			kernel[1] = 0.2642f;
			kernel[2] = 0.0f;
			kernel[3] = -0.2642f;
			kernel[4] =  -0.1179f;
			break;

		default:
			std::cout << "class DifferentialOfGaussian1D<N," << KernelSize  << "have no kernel information " << std::endl;
			break;
		}
	}
};

/**
 * @endcond
 */


/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
