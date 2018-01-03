/**
 * @file /src/test/priorities.cpp
 *
 * @brief Unit Test for priority configuration.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <iostream>
#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <gtest/gtest.h>
#include "../../include/ecl/threads/priority.hpp"
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::set_priority;
using ecl::BackgroundPriority;
using ecl::LowPriority;
using ecl::NormalPriority;
using ecl::HighPriority;
using ecl::CriticalPriority;
using ecl::RealTimePriority4;
using ecl::RealTimePriority3;
using ecl::RealTimePriority2;
using ecl::RealTimePriority1;
using ecl::StandardException;

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(PriorityTest,statistics) {
	std::string diagnostics = ecl::print_priority_diagnostics();
	SUCCEED();
}

TEST(PriorityTest,setPosixPriorities) {
	try {
		set_priority(BackgroundPriority);
		set_priority(LowPriority);
		set_priority(NormalPriority);
		set_priority(HighPriority);
		set_priority(CriticalPriority);
		SUCCEED();
	} catch ( const StandardException &e ) {
        // often dont have permission to higher level priorities so just warn
		// ADD_FAILURE() << "Could not set all the non-real time priorities.";
        std::cout << "Do not have permission for the higher level priorities." << std::endl;
	}
}

TEST(PriorityTest,setPosixRealTimePriorities) {
	try {
		set_priority(RealTimePriority1);
		set_priority(RealTimePriority2);
		set_priority(RealTimePriority3);
		set_priority(RealTimePriority4);
		SUCCEED();
	} catch ( const StandardException &e ) {
		SUCCEED();
            // often dont have permission for real time scheduling, so just warn
            // ADD_FAILURE() << "Could not set all the real time priorities.";
            std::cout << "Do not have permission for real time scheduling priorities." << std::endl;
	}
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}


#else

/*****************************************************************************
** Alternative main
*****************************************************************************/

int main(int argc, char **argv) {

	std::cout << "Currently not supported on your platform." << std::endl;
}

#endif /* ECL_IS_POSIX */

