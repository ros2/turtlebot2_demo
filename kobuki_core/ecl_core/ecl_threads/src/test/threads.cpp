/**
 * @file /src/test/threads.cpp
 *
 * @brief Unit Test for the @ref ecl::Thread "Thread" class.
 *
 * @date June 2009
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
#include <ecl/config/ecl.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <ecl/utilities/references.hpp>
#include "../../include/ecl/threads/thread.hpp"

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
using ecl::Thread;
using ecl::generateFunctionObject;
using ecl::ref;

/*****************************************************************************
** Globals
*****************************************************************************/

// Change this if you want std output.
static const bool output = false;


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace threads {
namespace tests {

/*****************************************************************************
** Classes
*****************************************************************************/

class ThreadMemberFunctions {
public:
	void f() {
		if ( output ) {
			std::cout << "A::f()" << std::endl;
		}
	}
	void g(const int &i) {
		if ( output ) {
			std::cout << "A::g(" << i << ")" << std::endl;
		}
	}
};

class NullaryFunction {
public:
	NullaryFunction() : i(0) {};
	typedef void result_type;
	void operator()() {
		for (int j = 0; j < 3; ++j ) {
			if ( output ) {
				std::cout << "  Nullary Function Object: " << i << std::endl;
			}
			sleep(1);
			++i;
		}
		if ( output ) {
			std::cout << "  Nullary Function Object: finished." << std::endl;
		}
	}
	int i;
};

/*****************************************************************************
** Functions
*****************************************************************************/

void d() {
	for (int i = 0; i < 5; ++i ) {
		sleep(1);
		if ( output ) {
			std::cout << "  d(): " << i << std::endl;
		}
	}
	if ( output ) {
		std::cout << "  d(): finished" << std::endl;
	}
}
void d_cb() {
	if ( output ) {
		std::cout << "  d(): callback function" << std::endl;
	}
}

void f() {
	for (int i = 0; i < 3; ++i) {
		if ( output ) {
			std::cout << "f(): " << i << std::endl;
		}
	}
}

void g(const int &i) {
	for (int j = 0; j < 3; ++j) {
		if ( output ) {
			std::cout << "g(" << i << ")" << std::endl;
		}
	}
}

void deconstructThread() {
	Thread thread_deconstruct(d);
}

} // namespace Tests
} // namespace threads
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::threads::tests;

/*****************************************************************************
** Doxygen
*****************************************************************************/

/**
 * @endcond
 */

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(ThreadTests,threadFunctionTypes) {
    ThreadMemberFunctions a;
    Thread thread_f1(f);
    Thread thread_f2(&ThreadMemberFunctions::f,a);
    Thread thread_f3(generateFunctionObject(f));
    Thread thread_f4(generateFunctionObject(g,1));
    Thread thread_f5(generateFunctionObject(&ThreadMemberFunctions::f,a));
    Thread thread_f6(generateFunctionObject(&ThreadMemberFunctions::g,a,2));
    thread_f1.join();
    thread_f2.join();
    thread_f3.join();
    thread_f4.join();
    thread_f5.join();
    thread_f6.join();
    SUCCEED();
}

TEST(ThreadTests,cancelThread) {
	Thread thread(generateFunctionObject(d));
	sleep(3);
	EXPECT_TRUE(thread.isRunning());
//	std::cout << "Cancelling d() thread (you should see no further output)." << std::endl;
	thread.cancel();
	EXPECT_FALSE(thread.isRunning());
    thread.join();
	EXPECT_FALSE(thread.isRunning());
}

TEST(ThreadTests,deconstruct) {
    deconstructThread();
    sleep(6);
    SUCCEED();
}

TEST(ThreadTests,nullaryFunctionObjects) {
    NullaryFunction function_object;
    Thread thread_f8(function_object); //    Thread thread_f8 = NullaryFunction();
    sleep(4);
    Thread thread_f9(ref(function_object));
	sleep(4);
    thread_f8.join();
    thread_f9.join();
	SUCCEED();
}

TEST(ThreadTests,stackSize) {
    Thread thread(f,ecl::DefaultPriority,1024*1024);
    thread.join();
    SUCCEED();
}

TEST(ThreadTests,delayedStart) {

	NullaryFunction function_object;
	ThreadMemberFunctions a;
	Thread thread1, thread2, thread3, thread4;
	thread1.start(f);
	thread2.start(function_object);
	thread3.start(&ThreadMemberFunctions::f,a);
	thread4.start(generateFunctionObject(&ThreadMemberFunctions::g,a,2));
	thread1.join();
	thread2.join();
	thread3.join();
	thread4.join();
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

