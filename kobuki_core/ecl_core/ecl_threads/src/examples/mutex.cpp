/**
 * @file src/examples/mutex.cpp
 *
 * @brief Demos the mutex functions.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <vector>
#include "../../include/ecl/threads/mutex.hpp"
#include "../../include/ecl/threads/thread.hpp"
#include <ecl/time/sleep.hpp>

int shared_value = 0;
ecl::Mutex mutex;

void f() {
	ecl::Sleep sleep;
	sleep(1);

	std::cout << "Thread: enter blocking" << std::endl;
	mutex.lock();

	shared_value = 10;
	std::cout << "Thread: sets shared_value to 10" << std::endl;

	mutex.unlock();
	std::cout << "Thread: leave blocking" << std::endl;
}

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Mutex" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Lock" << std::endl;
    mutex.lock();

    std::cout << "Unlock" << std::endl;
    mutex.unlock();

    std::cout << "Trylock" << std::endl;
    mutex.trylock();

    std::cout << "Try to lock block already locked" << std::endl;
    if (mutex.trylock()) {
    	std::cout << "Abnormal" << std::endl;
    } else {
    	std::cout << "Failed(Normal)" << std::endl;
    }

    std::cout << "Unlock" << std::endl;
    mutex.unlock();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Synchronization" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ecl::Thread thread_f(f);
	ecl::Sleep sleep;

	std::cout << "Main: enter blocking" << std::endl;
	mutex.lock();
	sleep(3);

	if (shared_value < 5) {
		std::cout << "shared_value(" << shared_value << ") is smaller than 5" << std::endl;
	} else {
		std::cout << "shared_value(" << shared_value << ") is larger than 5" << std::endl;
	}

	mutex.unlock();
	std::cout << "Main: leave blocking" << std::endl;

	thread_f.join();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}
