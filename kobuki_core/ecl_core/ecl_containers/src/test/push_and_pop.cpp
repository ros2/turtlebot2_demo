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
#include "../../include/ecl/containers/push_and_pop.hpp"

/*****************************************************************************
 ** Using
 *****************************************************************************/

using ecl::Array;
using ecl::StandardException;
using ecl::ContainerConcept;
using ecl::PushAndPop;

/*****************************************************************************
 ** Tests
 *****************************************************************************/

TEST(PushAndPopTests,constructors)
{
  PushAndPop<int> pp_dynamic(4, 2);
  EXPECT_EQ(2, pp_dynamic[0]);
  EXPECT_EQ(2, pp_dynamic[1]);
  EXPECT_EQ(2, pp_dynamic[2]);
  EXPECT_EQ(2, pp_dynamic[3]);

  PushAndPop<int, 4> pp_fixed(2);
  EXPECT_EQ(2, pp_fixed[0]);
  EXPECT_EQ(2, pp_fixed[1]);
  EXPECT_EQ(2, pp_fixed[2]);
  EXPECT_EQ(2, pp_fixed[3]);
}

TEST(PushAndPopTests, pop_and_push )
{
  PushAndPop<double> pp_dynamic(4);
  for (int i = 0; i < 4; i++)
  {
    pp_dynamic.push_back((double)i);
  }

  EXPECT_EQ(0.0, pp_dynamic[0]);
  EXPECT_EQ(1.0, pp_dynamic[1]);
  EXPECT_EQ(2.0, pp_dynamic[2]);
  EXPECT_EQ(3.0, pp_dynamic[3]);

  pp_dynamic.pop_front();
  pp_dynamic.pop_front();
  EXPECT_EQ(2, pp_dynamic.size());

  // [0] always returns first data from the buffer
  EXPECT_EQ(2.0, pp_dynamic[0]);
  EXPECT_EQ(3.0, pp_dynamic[1]);
}

TEST(PushAndPopTests, push_back_only )
{
  PushAndPop<unsigned int, 3> pp;
  for (unsigned int i = 0; i < 4; i++)
  {
    pp.push_back(i);
  }

  EXPECT_EQ( 3, pp.size());

  for (unsigned int i = 4; i < 8; i++)
  {
    pp.push_back(i);
    EXPECT_EQ( 3, pp.size());
  }

  PushAndPop<unsigned int> pp_dynamic(3, 1);
  for (unsigned int i = 0; i < 4; i++)
  {
    pp_dynamic.push_back(i);
  }

  EXPECT_EQ( 3, pp_dynamic.size());

  for (unsigned int i = 4; i < 8; i++)
  {
    pp_dynamic.push_back(i);
    EXPECT_EQ( 3, pp_dynamic.size());
  }
}

TEST(PushAndPopTests, pop_front_only )
{
  // fixed
  PushAndPop<unsigned int, 3> pp;
  for (unsigned int i = 0; i < 2; i++)
  {
    pp.push_back(i);
  }

  EXPECT_EQ( 2, pp.size());

  for (unsigned int i = 0; i < 2; i++)
  {
    pp.pop_front();
    EXPECT_EQ( (2-1-i), pp.size());
  }

  // when you want to check pop-front, please enable below code manually
//	for( unsigned int i=0; i<2; i++ )
//	{
//		pp.pop_front();
//		EXPECT_EQ( 0, pp.size() );
//	}

// dynamic
  PushAndPop<unsigned int> pp_dynamic(3, 1);
  for (unsigned int i = 0; i < 2; i++)
  {
    pp_dynamic.push_back(i);
  }

  EXPECT_EQ( 2, pp_dynamic.size());

  for (unsigned int i = 0; i < 2; i++)
  {
    pp_dynamic.pop_front();
    EXPECT_EQ( (2-1-i), pp_dynamic.size());
  }

  // when you want to check pop-front, please enable below code manually
//	for( unsigned int i=0; i<2; i++ )
//	{
//		pp_dynamic.pop_front();
//		EXPECT_EQ( 0, pp_dynamic.size() );
//	}
}

TEST(PushAndPopTests,concepts)
{
  typedef PushAndPop<unsigned char> UnsignedByteBuffer;
  ecl_compile_time_concept_check (ContainerConcept<UnsignedByteBuffer> );
  typedef PushAndPop<char> ByteBuffer;
  ecl_compile_time_concept_check (ContainerConcept<ByteBuffer> );
  SUCCEED();
  // ecl_compile_time_concept_check(ContainerConcept<Array<char,6>>); // The macro won't let you do this (macro function syntax problem, not a c++ problem)
}


/*****************************************************************************
 ** Main program
 *****************************************************************************/

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
