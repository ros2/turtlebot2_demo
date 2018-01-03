/**
 * @file /src/test/semaphores_timed.cpp
 *
 * @brief Unit Test for timed semaphores.
 *
 * @date August, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <gtest/gtest.h>
#include <ecl/time/sleep.hpp>
#include <ecl/time/timestamp.hpp>
#include "../../include/ecl/ipc/semaphore.hpp"
#include "../../include/ecl/ipc/shared_memory.hpp"

#ifdef ECL_HAS_SEMAPHORES

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::SharedMemory;
using ecl::Semaphore;
using ecl::Duration;
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

class SemaphoreTimedData {
    public:
        double value[2];
        void initialise() { value[0] = 1.0; value[1] = 2.0; };
};

} // namespace tests
} // namespace ipc
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::ipc::tests::SemaphoreTimedData;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @endcond
 */

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SemaphoreTimedTests,access) {

    string sm_name("shared_memory");
    string sem_name("shared_semaphore");
    Sleep sleep;
    Duration timeout(0,750000000);

    pid_t pID = fork();
    if (pID == 0)
    {
        /*********************************************************************
        ** Child
        *********************************************************************/
//        std::cout << "Child waiting" << std::endl;
        sleep(1);
        try {
            SharedMemory<SemaphoreTimedData> sm(sm_name);
            Semaphore semaphore(sem_name);
//        std::cout << "Child trying the lock for 750ms" << std::endl;
            bool result = semaphore.trylock(timeout);
            EXPECT_FALSE(result);
            if ( result ) {
//        	std::cout << "Child locked semaphore." << std::endl;
            } else {
//        	std::cout << "Child lock attempt timed out." << std::endl;
//            std::cout << "Child trying the lock again for 750ms" << std::endl;
                if ( (result = semaphore.trylock(timeout)) ) {
                   EXPECT_TRUE(result);
//            	   std::cout << "Child locked semaphore." << std::endl;
                } else {
//            	std::cout << "Child lock attempt timed out." << std::endl;
                }
            }
            semaphore.unlock();
        } catch ( ecl::StandardException &e ) {
            // Don't fail the test, hudson doesn't let us have permissions for instance.
            std::cout << e.what() << std::endl;
        }
    }
    else if (pID < 0)            // failed to fork
    {
    	std::cerr << "Failed to fork" << std::endl;
        exit(1);
    }
    else
    {
        /*********************************************************************
        ** Parent
        *********************************************************************/
        try {
            SharedMemory<SemaphoreTimedData> sm(sm_name);
            Semaphore semaphore(sem_name);
            SemaphoreTimedData *data = sm.data();
            semaphore.lock();
            data->value[0] = 1.3;
            data->value[1] = 2.3;
            EXPECT_EQ(1.3,data->value[0]);
            EXPECT_EQ(2.3,data->value[1]);
//        std::cout << "Parent in the shared memory" << std::endl;
//        std::cout << "Parent locking the semaphore." << std::endl;
            sleep(2);
//        std::cout << "Parent unlocking the semaphore." << std::endl;
            semaphore.unlock();
            sleep(2);
        } catch ( ecl::StandardException &e ) {
            // Don't fail the test, hudson doesn't let us have permissions for instance.
            std::cout << e.what() << std::endl;
        }
    }
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    std::cout << std::endl;
    std::cout << "What you should see:" << std::endl;
    std::cout << "  - Process forks" << std::endl;
    std::cout << "  - Parent creates shared memory." << std::endl;
    std::cout << "  - Parent locks a semaphore and enters shared memory for 2 seconds." << std::endl;
    std::cout << "  - Child waits one second and tries lock for 750ms." << std::endl;
    std::cout << "  - Child fails and again tries lock for 750ms." << std::endl;
    std::cout << "  - Child succeeds in locking.." << std::endl;
    std::cout << std::endl;

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

#else

/*****************************************************************************
** Alternative Main
*****************************************************************************/

int main(int argc, char **argv) {
    std::cout << std::endl;
    std::cout << "Semaphores are not supported on this platform (or ecl is just lacking)." << std::endl;
    std::cout << std::endl;
    return 0;
}

#endif /* ECL_HAS_SEMAPHORES */







