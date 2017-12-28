/**
 * @file /src/test/shared_memory.cpp
 *
 * @brief Unit Test for the shared memory classes.
 *
 * @date August 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <gtest/gtest.h>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time/sleep.hpp>
#include "../../include/ecl/ipc/shared_memory.hpp"

#ifdef ECL_HAS_SHARED_MEMORY
// Should check for posix support too because we're using posix fork, but
// posix is all we got for now, so don't care too much.

/*****************************************************************************
** Using
*****************************************************************************/

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using ecl::SharedMemory;
using ecl::Sleep;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace ipc {
namespace tests {

/*****************************************************************************
** Data Storage Class
*****************************************************************************/

class Data {
    public:
        Data(double d1 = 0.0, double d2 = 0.0) {value[0] = d1; value[1] = d2;}
        double value[2];
};

} // namespace tests
} // namespace ipc
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::ipc::tests::Data;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @endcond
 */

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SharedMemoryTests,access) {

	string name("shared_memory");
    Sleep sleep;

    pid_t pID = fork();
    if (pID == 0)
    {
        /*********************************************************************
        ** Child
        *********************************************************************/
        sleep(1);
        try {
            SharedMemory<Data> sm(name.c_str());
           Data *data = sm.data();
            EXPECT_EQ(1.3,data->value[0]);
            EXPECT_EQ(2.3,data->value[1]);
//        cout << "Child read: " << data->value[0] << " " << data->value[1] << endl;
        } catch ( ecl::StandardException &e) {
            // Don't fail the test, hudson doesn't let us have permissions for instance.
            std::cout << e.what() << std::endl;
        }
    }
    else if (pID < 0)            // failed to fork
    {
        cerr << "Failed to fork" << endl;
        exit(1);
    }
    else
    {
        /*********************************************************************
        ** Parent
        *********************************************************************/
        try {
            SharedMemory<Data> sm(name.c_str());
            Data *data = sm.data();
            data->value[0] = 1.3;
            data->value[1] = 2.3;
            EXPECT_EQ(1.3,data->value[0]);
            EXPECT_EQ(2.3,data->value[1]);
//        cout << "Parent wrote: " << data->value[0] << " " << data->value[1] << endl;
            sleep(4);
        } catch ( ecl::StandardException &e) {
            // Don't fail the test, hudson doesn't let us have permissions for instance.
            std::cout << e.what() << std::endl;
        }
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
** Alternative Main
*****************************************************************************/

int main(int argc, char **argv) {
    std::cout << std::endl;
    std::cout << "Shared memory is not supported on this platform (or ecl is just lacking)." << std::endl;
    std::cout << std::endl;
    return 0;
}

#endif /* ECL_HAS_SHARED_MEMORY */



