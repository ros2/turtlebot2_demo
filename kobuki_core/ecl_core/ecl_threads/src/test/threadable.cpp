/**
 * @file /src/test/threadable.cpp
 *
 * @brief Unit Test for the @ref ecl::Threadable "Threadable" class.
 *
 * @date January 2010
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

#include <iostream>
#include <gtest/gtest.h>
#include <ecl/config/ecl.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/threads/threadable.hpp"

/*****************************************************************************
** Doxygen
*****************************************************************************/
/**
 * @cond DO_NOT_DOXYGEN
 */
/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StandardException;
using ecl::Threadable;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace threads {
namespace tests {

/*****************************************************************************
** Classes
*****************************************************************************/

class AThreadable : public Threadable {
public:
	AThreadable(const unsigned int countdown_start = 5) : top(countdown_start) {}

	void runnable() {
		for ( unsigned int i = top; i > 0; --i ) {
//			std::cout << "Counting down...." << i << std::endl;
			sleep(1);
		}
	}

private:
	unsigned int top;
};

} // namespace tests
} // namespace threads
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::threads::tests::AThreadable;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @endcond
 */

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ThreadableTests,runTest) {
    AThreadable a;

    a.start();
    while ( a.isRunning() ) {
    	sleep(1);
    }
    SUCCEED();
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

