/**
 * @file /src/test/frequency.cpp
 *
 * @brief Unit Test for the FrequncyMonitor class.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include <iostream>
#include "../../include/ecl/time/frequency.hpp"
#include "../../include/ecl/time/sleep.hpp"
#include "../../include/ecl/time/timestamp.hpp"

/*****************************************************************************
** Platform Check
*****************************************************************************/

#ifdef ECL_HAS_TIMESTAMP

/*****************************************************************************
** Methods
*****************************************************************************/

std::ostream& operator <<( std::ostream& ostream , const ecl::FrequencyDiagnostics& diagnostics )
{
  ostream <<  std::fixed;
  ostream << "hz : " << diagnostics.hz;
  ostream << ", min : " << diagnostics.minimum_interval;
  ostream << ", max : " << diagnostics.maximum_interval;
  ostream << ", last_incoming : " << diagnostics.last_incoming << std::scientific;
  // could print the others, but that would require use of std ios flags to pretty print correctly.
  // let the user do that how they wish if they want to
  return ostream;
}

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(FrequencyMonitorTests,silent) {

  ecl::FrequencyMonitor monitor(0.5, true);
  ecl::FrequencyDiagnostics diagnostics;

  ecl::MilliSleep sleep_ms;
  unsigned int period = 100;

//  std::cout << "\n**********************************************************" << std::endl;
//  std::cout << "* Frequency Monitor Test" << std::endl;
//  std::cout << "**********************************************************\n" << std::endl;
  float epsilon = 0.3;
  for ( unsigned int i = 0; i < 50; ++i ) {
    monitor.update();
    diagnostics = monitor.analyse();
    if ( i == 25 ) {
      period = 200;
      std::cout << diagnostics << std::endl;
      EXPECT_NEAR(diagnostics.hz, 10.0, epsilon);
    }
    sleep_ms(period);
  }
  std::cout << diagnostics << std::endl;
  EXPECT_NEAR(diagnostics.hz, 5.0, epsilon);
  SUCCEED();
}

#endif /* ECL_HAS_TIMESTAMP */

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
