/**
 * @file /src/test/shared_files.cpp
 *
 * @brief Unit Test for shared files.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <gtest/gtest.h>
#include <ecl/threads/thread.hpp>
#include <ecl/time/timestamp.hpp>
#include "../../include/ecl/devices/shared_file.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::New;
using ecl::SharedFile;
using ecl::Thread;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {
namespace tests {

/*****************************************************************************
** Globals
*****************************************************************************/

void shared_files_f() {
    SharedFile file("shared.txt");
    long n;
    for (unsigned int i = 0; i < 3; ++i ) {
    	n = file.write("Thread\n",7);
    	EXPECT_EQ(7,n);
    }
    file.flush();
}


} // namespace tests
} // namespace devices
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::devices::tests;

TEST(SharedFileTests,allEggsInTheBasket) {

    SharedFile file("shared.txt");
    Thread thread(shared_files_f);
    long n;
    for (unsigned int i = 0; i < 3; ++i ) {
    	n = file.write("Main\n",5);
    	EXPECT_EQ(5,n);
    }
    file.flush();
    thread.join();
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
