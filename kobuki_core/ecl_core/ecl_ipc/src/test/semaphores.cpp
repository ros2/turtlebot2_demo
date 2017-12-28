/**
 * @file /src/test/semaphores.cpp
 *
 * @brief Unit Test for semaphores.
 *
 * @date August 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstdlib>
#include <gtest/gtest.h>
#include <ecl/time/sleep.hpp>
#include "../../include/ecl/ipc/semaphore.hpp"
#include "../../include/ecl/ipc/shared_memory.hpp"

#ifdef ECL_HAS_SEMAPHORES

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::SharedMemory;
using ecl::Semaphore;
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

class SemaphoreTestData {
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

using ecl::ipc::tests::SemaphoreTestData;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @endcond
 */

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SemaphoreTests,access) {

    string sm_name("shared_memory");
    string sem_name("shared_semaphore");
    Sleep sleep;

    pid_t pID = fork();
    if (pID == 0)
    {
        /*********************************************************************
        ** Child
        *********************************************************************/
        sleep(1);
        try {
            SharedMemory<SemaphoreTestData> sm(sm_name);
            Semaphore semaphore(sem_name);
//        std::cout << "Child waiting" << std::endl;
//        std::cout << "Child trying the lock" << std::endl;
            if ( !semaphore.trylock() ) {
//        	std::cout << "Child tried the lock but failed, now hard locking" << std::endl;
                semaphore.lock();
            }
//        std::cout << "Child entering shared memory" << std::endl;
            SemaphoreTestData *data = sm.data();
//        std::cout << "Child: " << data->value[0] << " " << data->value[1] << std::endl;
            EXPECT_EQ(1.3,data->value[0]);
            EXPECT_EQ(2.3,data->value[1]);
            semaphore.unlock();
        } catch ( ecl::StandardException &e) {
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
            SharedMemory<SemaphoreTestData> sm(sm_name);
            Semaphore semaphore(sem_name);
            SemaphoreTestData *data = sm.data();
            semaphore.lock();
//        std::cout << "Parent in the shared memory" << std::endl;
            sleep(2);
            data->value[0] = 1.3;
            data->value[1] = 2.3;
            EXPECT_EQ(1.3,data->value[0]);
            EXPECT_EQ(2.3,data->value[1]);
//        std::cout << "Parent: " << data->value[0] << " " << data->value[1] << std::endl;
//        std::cout << "Parent leaving shared memory" << std::endl;
            semaphore.unlock();
            sleep(3);
        } catch ( ecl::StandardException &e) {
            // Don't fail the test, hudson doesn't let us have permissions for instance.
            std::cout << e.what() << std::endl;
        }
    }
    SUCCEED();

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
    std::cout << "  - Child waits one second and attempts to lock the already locked semaphore." << std::endl;
    std::cout << "  - Parent writes into shared memory and unlocks the semaphore." << std::endl;
    std::cout << "  - Child finally gains access and reads the shared memory." << std::endl;
    std::cout << "  - Parent process sleeps for four seconds while child dies." << std::endl;
    std::cout << "  - Parent dies." << std::endl;
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


