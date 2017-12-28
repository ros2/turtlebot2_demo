/**
 * @file src/examples/thread.cpp
 *
 * @brief Demos the thread functions.
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
#include "../../include/ecl/threads/thread.hpp"
#include <ecl/time/sleep.hpp>

class ThreadMemberFunctions {
public:
	void f() {
		std::cout << "A::f()" << std::endl;
	}
	void g(const int &i) {
		std::cout << "A::g(" << i << ")" << std::endl;
	}
};

class NullaryFunction {
public:
	NullaryFunction() : i(0) {};
	typedef void result_type;
	void operator()() {
		for (int j = 0; j < 3; ++j ) {
			std::cout << "  Nullary Function Object: " << i << std::endl;
			ecl::Sleep sleep;
			sleep(1);
			++i;
		}
		std::cout << "  Nullary Function Object: finished." << std::endl;
	}
	int i;
};

void d() {
	for (int i = 0; i < 5; ++i ) {
		ecl::Sleep sleep;
		sleep(1);
		std::cout << "  d(): " << i << std::endl;
	}
	std::cout << "  d(): finished" << std::endl;
}
void d_cb() {
	std::cout << "  d(): callback function" << std::endl;
}

void f() {
	for (int i = 0; i < 3; ++i) {
		std::cout << "f(): " << i << std::endl;
	}
}

void g(const int &i) {
	for (int j = 0; j < 3; ++j) {
		std::cout << "g(" << i << ")" << std::endl;
	}
}

void deconstructThread() {
	ecl::Thread thread_deconstruct(d);
}

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Thread Function Type" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ThreadMemberFunctions a;
    ecl::Thread thread_f1(f);
    ecl::Thread thread_f2(&ThreadMemberFunctions::f, a);
    ecl::Thread thread_f3(ecl::generateFunctionObject(f));
    ecl::Thread thread_f4(ecl::generateFunctionObject(g,1));
    ecl::Thread thread_f5(ecl::generateFunctionObject(&ThreadMemberFunctions::f,a));
    ecl::Thread thread_f6(ecl::generateFunctionObject(&ThreadMemberFunctions::g,a,2));

    thread_f1.join();
    thread_f2.join();
    thread_f3.join();
    thread_f4.join();
    thread_f5.join();
    thread_f6.join();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Cancel" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	ecl::Thread thread(ecl::generateFunctionObject(d));
	ecl::Sleep sleep;
	sleep(3);
	if (!thread.isRunning()) {
		std::cout << "Abnormal #1" << std::endl;
	}
	thread.cancel();
	if (thread.isRunning()) {
		std::cout << "Abnormal #2" << std::endl;
	}
    thread.join();
	if (thread.isRunning()) {
		std::cout << "Abnormal #3" << std::endl;
	}

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Deconstruct" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    deconstructThread();
    sleep(6);

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Nullary Function Objects" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    NullaryFunction function_object_1;
    ecl::Thread thread_f8(function_object_1);
    sleep(4);
    ecl::Thread thread_f9(ecl::ref(function_object_1));
	sleep(4);
    thread_f8.join();
    thread_f9.join();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Delayed Start" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	NullaryFunction function_object_2;
	ThreadMemberFunctions b;
	ecl::Thread thread1, thread2, thread3, thread4;
	thread1.start(f);
	thread2.start(function_object_2);
	thread3.start(&ThreadMemberFunctions::f,b);
	thread4.start(ecl::generateFunctionObject(&ThreadMemberFunctions::g,b,2));

	thread1.join();
	thread2.join();
	thread3.join();
	thread4.join();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}
