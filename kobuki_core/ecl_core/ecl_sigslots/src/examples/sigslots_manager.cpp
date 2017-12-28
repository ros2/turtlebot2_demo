/**
 * @file /ecl_sigslots/src/examples/sigslots_manager.cpp
 *
 * @brief Demonstration of how to debug with SigSlotsManager.
 *
 * @date March 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/sigslots.hpp"

/*****************************************************************************
** Fuctions
*****************************************************************************/
/**
 * @cond DO_NOT_DOXYGEN
 */

void g() {
	std::cout << "  This is standalone g()" << std::endl;
}
void h(int i) {
	std::cout << "  This is standalone g(int): " << i << std::endl;
}

class A {
public:
	void g() {
		std::cout << "  This is A::g()" << std::endl;
	}
	void h(int i) {
		std::cout << "  This is A::g(int): " << i << std::endl;
	}
};

/**
 * @endcond
 */

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {
	A a;
	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                 1 Signal, 2 Slots" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	ecl::Signal<> sig_void;
	ecl::Slot<> slot_void0(g);
	ecl::Slot<> slot_void1(&A::g,a);

	sig_void.connect("void_test");
	slot_void0.connect("void_test");
	slot_void1.connect("void_test");
	std::cout << "Emitting:" << std::endl;
	sig_void.emit();

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                     Statistics" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	ecl::SigSlotsManager<>::printStatistics();

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                 1 Data Signal, 2 Data Slots" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	ecl::Signal<int> sig_data;
	ecl::Slot<int> slot_data0(h);
	ecl::Slot<int> slot_data1(&A::h,a);

	sig_data.connect("data_test");
	slot_data0.connect("data_test");
	slot_data1.connect("data_test");
	std::cout << "Emitting:" << std::endl;
	sig_data.emit(3);

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                     Statistics" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	ecl::SigSlotsManager<int>::printStatistics();

	return 0;
}

