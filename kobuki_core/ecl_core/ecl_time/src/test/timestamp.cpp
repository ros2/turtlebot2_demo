/**
 * @file /src/test/timestamp.cpp
 *
 * @brief Unit Test for timestamp objects.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstdlib>
#include <gtest/gtest.h>
#include "../../include/ecl/time/timestamp.hpp"

/*****************************************************************************
** Platform Check
*****************************************************************************/

#ifdef ECL_HAS_TIMESTAMP

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::TimeStamp;

/*****************************************************************************
** Variables
*****************************************************************************/

bool verbose = false;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(TimeStampTests,construction) {
    TimeStamp time;
    SUCCEED();
    TimeStamp time_pair(3,123456789L);
    EXPECT_FLOAT_EQ(3.123456789,time_pair);
    TimeStamp time_double(3.00001);
    EXPECT_FLOAT_EQ(3.00001,time_double);
}

TEST(TimeStampTests,timestamps) {
    TimeStamp time;
    time.stamp();
    long t = time.sec();
    t = time.msec();
    t = time.usec();
    t = time.nsec();
    SUCCEED();
    time.stamp(3,1425);
    EXPECT_FLOAT_EQ(3.000001425,time);
    time.stamp(4.00000142);
    EXPECT_FLOAT_EQ(4.00000142,time);
}

TEST(TimeStampTests,copyConstruction) {
    TimeStamp time(3,100);
	TimeStamp time_copy(time);
	EXPECT_EQ(3,time_copy.sec());
	EXPECT_EQ(100,time_copy.nsec());
}

TEST(TimeStampTests,copyAssignment) {
    TimeStamp time(3,100);
	TimeStamp time_copy;
    time_copy = time;
	EXPECT_EQ(3,time_copy.sec());
	EXPECT_EQ(100,time_copy.nsec());
}

TEST(TimeStampTests,comparisonOperators) {
    TimeStamp time(3,100);
	TimeStamp time_copy(time);
	EXPECT_TRUE(time==time_copy);
	EXPECT_FALSE(time!=time_copy);
	EXPECT_TRUE(time<=time_copy);
	EXPECT_TRUE(time>=time_copy);
	EXPECT_FALSE(time<time_copy);
	EXPECT_FALSE(time>time_copy);
    time.stamp();
	EXPECT_FALSE(time==time_copy);
	EXPECT_TRUE(time!=time_copy);
	EXPECT_FALSE(time<=time_copy);
	EXPECT_TRUE(time>=time_copy);
	EXPECT_FALSE(time<time_copy);
	EXPECT_TRUE(time>time_copy);
}

TEST(TimeStampTests,mathematicalOperators) {
	TimeStamp time, time_copy;
    time.stamp(1,100100100L);
    time_copy.stamp(1,900100100L);
    time += time_copy;
    EXPECT_EQ(3,time.sec());
    EXPECT_EQ(200200,time.nsec());
    time.stamp(1,100100100L);
    time = time + time_copy;
    EXPECT_EQ(3,time.sec());
    EXPECT_EQ(200200,time.nsec());
    time.stamp(2,100100100L);
    time -= time_copy;
    EXPECT_EQ(0,time.sec());
    EXPECT_EQ(200000000,time.nsec());
    time.stamp(2,100100100L);
    time = time - time_copy;
    EXPECT_EQ(0,time.sec());
    EXPECT_EQ(200000000,time.nsec());
}

TEST(TimeStampTests,negatives) {
  if ( verbose ) {
    std::cout << "*****************************************************************************" << std::endl;
    std::cout << "* Negatives" << std::endl;
    std::cout << "*****************************************************************************" << std::endl;
  }
  /****************************************
  ** Variables
  ****************************************/
  ecl::TimeStamp time_2_3(2,300000000L), time_n_2_3(-2, -300000000L);
  ecl::TimeStamp time_1_3(1,300000000L), time_n_1_3(-1,-300000000L);
  ecl::TimeStamp time_0_3(0,300000000L), time_n_0_3(0,-300000000L);
  ecl::TimeStamp time_2_7(2,700000000L), time_n_2_7(-2,-700000000L);
  ecl::TimeStamp time_1_7(1,700000000L), time_n_1_7(-1,-700000000L);
  ecl::TimeStamp time_0_7(0,700000000L), time_n_0_7(0,-700000000L);
  /****************************************
  ** Constructor & Operator <<
  ****************************************/
  std::vector<TimeStamp> timestamps = { TimeStamp(1.7),
                                        TimeStamp(0.7),
                                        TimeStamp(-0.7),
                                        TimeStamp(-1.7),
  };
  if ( verbose ) {
    std::cout << "Operator <<\n  ";
    for ( const TimeStamp& timestamp : timestamps) {
      std::cout << timestamp << " ";
    }
    std::cout << std::endl;
  }
  EXPECT_EQ(1, timestamps[0].sec());
  EXPECT_EQ(700000000, timestamps[0].nsec());
  EXPECT_EQ(0, timestamps[1].sec());
  EXPECT_EQ(700000000, timestamps[1].nsec());
  EXPECT_EQ(0, timestamps[2].sec());
  EXPECT_EQ(-700000000, timestamps[2].nsec());
  EXPECT_EQ(-1, timestamps[3].sec());
  EXPECT_EQ(-700000000, timestamps[3].nsec());

  std::vector<float> double_representations;
  for ( const TimeStamp& timestamp : timestamps) {
    double_representations.push_back(timestamp);
  }
  if ( verbose ) {
    std::cout << "Operator double()\n  ";
    for ( const float& d : double_representations ) {
      std::cout << d << " ";
    }
    std::cout << std::endl;
  }
  EXPECT_FLOAT_EQ(1.7, double_representations[0]);
  EXPECT_FLOAT_EQ(0.7, double_representations[1]);
  EXPECT_FLOAT_EQ(-0.7, double_representations[2]);
  EXPECT_FLOAT_EQ(-1.7, double_representations[3]);

  /****************************************
  ** Operator -
  ****************************************/
  double_representations.clear();
  double_representations.push_back(ecl::TimeStamp(1.3) - ecl::TimeStamp(5.1));
  double_representations.push_back(ecl::TimeStamp(1.3) - ecl::TimeStamp(5.8));
  double_representations.push_back(ecl::TimeStamp(1.3) - ecl::TimeStamp(1.5));
  double_representations.push_back(ecl::TimeStamp(-1.3) - ecl::TimeStamp(1.5));
  double_representations.push_back(ecl::TimeStamp(-1.3) - ecl::TimeStamp(-0.8));
  double_representations.push_back(ecl::TimeStamp(-1.3) - ecl::TimeStamp(-5.8));
  if ( verbose ) {
    std::cout << "Operator -\n  ";
    std::cout << (ecl::TimeStamp(1.3) - ecl::TimeStamp(5.1)) << " ";
    std::cout << (ecl::TimeStamp(1.3) - ecl::TimeStamp(5.8)) << " ";
    std::cout << (ecl::TimeStamp(1.3) - ecl::TimeStamp(1.5)) << " ";
    std::cout << (ecl::TimeStamp(-1.3) - ecl::TimeStamp(1.5)) << " ";
    std::cout << (ecl::TimeStamp(-1.3) - ecl::TimeStamp(-0.8)) << " ";
    std::cout << (ecl::TimeStamp(-1.3) - ecl::TimeStamp(-5.8)) << " ";
    std::cout << std::endl;
  }
  EXPECT_FLOAT_EQ(-3.8, double_representations[0]);
  EXPECT_FLOAT_EQ(-4.5, double_representations[1]);
  EXPECT_FLOAT_EQ(-0.2, double_representations[2]);
  EXPECT_FLOAT_EQ(-2.8, double_representations[3]);
  EXPECT_FLOAT_EQ(-0.5, double_representations[4]);
  EXPECT_FLOAT_EQ(4.5, double_representations[5]);

  /****************************************
  ** Operator -=
  ****************************************/
  double_representations.clear();
  ecl::TimeStamp t;
  t = time_1_3; t -= ecl::TimeStamp(5.1);
  double_representations.push_back(t);
  t = time_1_3; t -= ecl::TimeStamp(5.8);
  double_representations.push_back(t);
  t = time_1_3; t -= ecl::TimeStamp(1.5);
  double_representations.push_back(t);
  t = time_n_1_3; t -= ecl::TimeStamp(1.5);
  double_representations.push_back(t);
  t = time_n_1_3; t -= ecl::TimeStamp(-0.8);
  double_representations.push_back(t);
  t = time_n_1_3; t -= ecl::TimeStamp(-5.8);
  double_representations.push_back(t);
  if ( verbose ) {
    std::cout << "Operator -=\n  ";
    for (const double& d : double_representations) {
      std::cout << d << " ";
    }
    std::cout << std::endl;
  }
  EXPECT_FLOAT_EQ(-3.8, double_representations[0]);
  EXPECT_FLOAT_EQ(-4.5, double_representations[1]);
  EXPECT_FLOAT_EQ(-0.2, double_representations[2]);
  EXPECT_FLOAT_EQ(-2.8, double_representations[3]);
  EXPECT_FLOAT_EQ(-0.5, double_representations[4]);
  EXPECT_FLOAT_EQ(4.5, double_representations[5]);
  /****************************************
  ** Operator +
  ****************************************/
  double_representations.clear(); timestamps.clear();
  double_representations.push_back(time_1_3 + time_n_2_3); timestamps.push_back(time_1_3 + time_n_2_3);
  double_representations.push_back(time_1_3 + time_n_2_7); timestamps.push_back(time_1_3 + time_n_2_7);
  double_representations.push_back(time_1_3 + time_n_1_7); timestamps.push_back(time_1_3 + time_n_1_7);
  double_representations.push_back(time_0_3 + time_n_0_7); timestamps.push_back(time_0_3 + time_n_0_7);
  double_representations.push_back(time_1_3 + time_n_0_7); timestamps.push_back(time_1_3 + time_n_0_7);
  double_representations.push_back(time_2_3 + time_n_0_7); timestamps.push_back(time_2_3 + time_n_0_7);
  double_representations.push_back(time_n_1_3 + time_n_1_7); timestamps.push_back(time_n_1_3 + time_n_1_7);
  double_representations.push_back(time_n_1_7 + time_n_1_7); timestamps.push_back(time_n_1_7 + time_n_1_7);
  if ( verbose ) {
    std::cout << "Operator +" << std::endl;
    std::cout << "  1.3 + (-2.3): " << double_representations[0] << std::endl;
    std::cout << "  1.3 + (-2.7): " << double_representations[1] << std::endl;
    std::cout << "  1.3 + (-1.7): " << double_representations[2] << std::endl;
    std::cout << "  0.3 + (-0.7): " << double_representations[3] << std::endl;
    std::cout << "  1.3 + (-0.7): " << double_representations[4] << std::endl;
    std::cout << "  2.3 + (-0.7): " << double_representations[5] << std::endl;
    std::cout << " -1.3 + (-1.7): " << double_representations[6] << std::endl;
    std::cout << " -1.7 + (-1.7): " << double_representations[7] << std::endl;
  }
  EXPECT_FLOAT_EQ(-1.0, double_representations[0]); EXPECT_EQ(0, timestamps[0].nsec());
  EXPECT_FLOAT_EQ(-1.4, double_representations[1]); EXPECT_EQ(-400000000, timestamps[1].nsec());
  EXPECT_FLOAT_EQ(-0.4, double_representations[2]); EXPECT_EQ(-400000000, timestamps[2].nsec());
  EXPECT_FLOAT_EQ(-0.4, double_representations[3]);  EXPECT_EQ(-400000000, timestamps[3].nsec());
  EXPECT_FLOAT_EQ(0.6, double_representations[4]);  EXPECT_EQ(600000000, timestamps[4].nsec());
  EXPECT_FLOAT_EQ(1.6, double_representations[5]);  EXPECT_EQ(600000000, timestamps[5].nsec());
  EXPECT_FLOAT_EQ(-3.0, double_representations[6]); EXPECT_EQ(000000000, timestamps[6].nsec());
  EXPECT_FLOAT_EQ(-3.4, double_representations[7]); EXPECT_EQ(-400000000, timestamps[7].nsec());

  /****************************************
  ** Operator +=
  ****************************************/
  double_representations.clear(); timestamps.clear();
  t = time_1_3; t += time_n_2_3;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_1_3; t += time_n_2_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_1_3; t += time_n_1_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_0_3; t += time_n_0_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_1_3; t += time_n_0_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_2_3; t += time_n_0_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_n_1_3; t += time_n_1_7;
  double_representations.push_back(t); timestamps.push_back(t);
  t = time_n_1_7; t += time_n_1_7;
  double_representations.push_back(t); timestamps.push_back(t);
  if ( verbose ) {
    std::cout << "Operator +=" << std::endl;
    std::cout << "  1.3 + (-2.3): " << double_representations[0] << std::endl;
    std::cout << "  1.3 + (-2.7): " << double_representations[1] << std::endl;
    std::cout << "  1.3 + (-1.7): " << double_representations[2] << std::endl;
    std::cout << "  0.3 + (-0.7): " << double_representations[3] << std::endl;
    std::cout << "  1.3 + (-0.7): " << double_representations[4] << std::endl;
    std::cout << "  2.3 + (-0.7): " << double_representations[5] << std::endl;
    std::cout << " -1.3 + (-1.7): " << double_representations[6] << std::endl;
    std::cout << " -1.7 + (-1.7): " << double_representations[7] << std::endl;
  }
  EXPECT_FLOAT_EQ(-1.0, double_representations[0]); EXPECT_EQ(0, timestamps[0].nsec());
  EXPECT_FLOAT_EQ(-1.4, double_representations[1]); EXPECT_EQ(-400000000, timestamps[1].nsec());
  EXPECT_FLOAT_EQ(-0.4, double_representations[2]); EXPECT_EQ(-400000000, timestamps[2].nsec());
  EXPECT_FLOAT_EQ(-0.4, double_representations[3]);  EXPECT_EQ(-400000000, timestamps[3].nsec());
  EXPECT_FLOAT_EQ(0.6, double_representations[4]);  EXPECT_EQ(600000000, timestamps[4].nsec());
  EXPECT_FLOAT_EQ(1.6, double_representations[5]);  EXPECT_EQ(600000000, timestamps[5].nsec());
  EXPECT_FLOAT_EQ(-3.0, double_representations[6]); EXPECT_EQ(000000000, timestamps[6].nsec());
  EXPECT_FLOAT_EQ(-3.4, double_representations[7]); EXPECT_EQ(-400000000, timestamps[7].nsec());

  /****************************************
  ** Operator <
  ****************************************/
  std::vector<bool> comparisons;
  comparisons.push_back(time_n_1_3 < time_n_2_3);
  comparisons.push_back(time_n_1_3 < time_2_3);
  comparisons.push_back(time_1_3 < time_n_2_3);
  comparisons.push_back(time_1_3 < time_2_3);
  comparisons.push_back(time_n_2_3 < time_n_1_3);
  comparisons.push_back(time_n_2_3 < time_1_3);
  comparisons.push_back(time_2_3 < time_n_1_3);
  comparisons.push_back(time_2_3 < time_1_3);
  comparisons.push_back(time_n_0_3 < time_n_0_7);
  comparisons.push_back(time_n_0_3 < time_0_7);
  comparisons.push_back(time_0_3 < time_n_0_7);
  comparisons.push_back(time_0_3 < time_0_7);
  comparisons.push_back(time_n_0_7 < time_n_0_3);
  comparisons.push_back(time_n_0_7 < time_0_3);
  comparisons.push_back(time_0_7 < time_n_0_3);
  comparisons.push_back(time_0_7 < time_0_3);
  if ( verbose ) {
    std::cout << "Operator <" << std::endl;
    std::cout << "  -1.3 < -2.3: " << (comparisons[0] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -1.3 <  2.3: " << (comparisons[1] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   1.3 < -2.3: " << (comparisons[2] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   1.3 <  2.3: " << (comparisons[3] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -2.3 < -1.3: " << (comparisons[4] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -2.3 <  1.3: " << (comparisons[5] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   2.3 < -1.3: " << (comparisons[6] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   2.3 <  1.3: " << (comparisons[7] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -0.3 < -0.7: " << (comparisons[8] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -0.3 <  0.7: " << (comparisons[9] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   0.3 < -0.7: " << (comparisons[10] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   0.3 <  0.7: " << (comparisons[11] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -0.7 < -0.3: " << (comparisons[12] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "  -0.7 <  0.3: " << (comparisons[13] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   0.7 < -0.3: " << (comparisons[14] ? std::string("true") : std::string("false")) << std::endl;
    std::cout << "   0.7 <  0.3: " << (comparisons[15] ? std::string("true") : std::string("false")) << std::endl;
  }
  std::vector<bool> expected = { false, true, false, true, true, true, false, false, false, true, false, true, true, true, false, false };
  for ( unsigned int i = 0; i < expected.size(); ++i ) {
    if ( expected[i] ) {
      EXPECT_TRUE(comparisons[i]);
    } else {
      EXPECT_FALSE(comparisons[i]);
    }
  }
  // TODO other comparison tests - didn't have to change that code for negative numbers
}

#endif /* ECL_HAS_TIMESTAMP */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
