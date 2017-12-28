/**
 * @file /src/examples/sigslots.cpp
 *
 * @brief Fairly comprehensive demo of the sigslot api.
 *
 * @date May 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/sigslots.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Signal;
using ecl::Slot;
using ecl::SigSlotsManager;

/**
 * @cond DO_NOT_DOXYGEN
 */

/*****************************************************************************
** Globals
*****************************************************************************/

void f() {
	std::cout << " ---> Hey Dude" << std::endl;
}

void g() {
	std::cout << " ---> Hey Chained Dude" << std::endl;
}

void h(const int& i) {
	std::cout << " ---> Hey Data Dude: " << i << std::endl;
}

/*****************************************************************************
** Classes
*****************************************************************************/

class A {
public:
	A() : slot(&A::f,*this), slot_data(&A::g,*this) {
		slot.connect("Dudette");
		slot_data.connect("DataDudette");
	};

	void f() {
		std::cout << " ---> Hey Dudette" << std::endl;
	}
	void g(const int& i) {
		std::cout << " ---> Hey Data Dudette: " << i << std::endl;
	}
	Slot<> slot;
	Slot<const int&> slot_data;
};

/**
 * @endcond
 */
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	std::cout << "Hello dude" << std::endl;

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                     Global Function" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;
	Slot<> slot1(&f);
	Slot<> slot2(&f);
	std::cout << "Connecting 1 signal and two slots to 'Dude'" << std::endl;
	slot1.connect("Dude");
	slot2.connect("Dude");
	Signal<> signal;
	signal.connect("Dude");
	std::cout << "Emitting" << std::endl;
	signal.emit();
	std::cout << std::endl;
	SigSlotsManager<>::printStatistics();
	std::cout << std::endl;

	std::cout << "Connecting a data slot-signal pair to 'DataDude'" << std::endl;
	Slot<const int&> slot_data_1(&h);
	Signal<const int&> sig_data_1;
	slot_data_1.connect("DataDude");
	sig_data_1.connect("DataDude");
	std::cout << "Emitting" << std::endl;
	sig_data_1.emit(2);
	std::cout << std::endl;
	SigSlotsManager<const int&>::printStatistics();

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                     Disconnects" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	std::cout << "Disconnecting second slot" << std::endl;
	slot2.disconnect();
	std::cout << "Emitting" << std::endl;
	signal.emit();
	std::cout << "Disconnecting signal" << std::endl;
	signal.disconnect();
	std::cout << "Emitting (to nowhere)" << std::endl;
	signal.emit();
	std::cout << std::endl;
	SigSlotsManager<>::printStatistics();

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                        Relays" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	std::cout << "Connecting signal and second slot back to 'Dude'" << std::endl;
	slot2.connect("Dude");
	signal.connect("Dude");
	std::cout << std::endl;
	SigSlotsManager<>::printStatistics();
	std::cout << std::endl;

	Signal<> signal_relay;
	std::cout << "Connecting relay as signal to 'ChainedDude'" << std::endl;
	signal_relay.connect("ChainedDude");
	std::cout << "Connecting relay as slot to 'Dude'." << std::endl;
	signal_relay.connectAsSlot("Dude");
	Slot<> slot3(&g);
	std::cout << "Connecting slot to 'ChainedDude'" << std::endl;
	slot3.connect("ChainedDude");
	std::cout << "Emitting" << std::endl;
	signal.emit();
	std::cout << std::endl;
	SigSlotsManager<>::printStatistics();

	std::cout << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << "*                      Member Functions" << std::endl;
	std::cout << "***************************************************************" << std::endl;
	std::cout << std::endl;

	std::cout << "Creating a member slot and connecting to 'Dudette'" << std::endl;
	A a;
	std::cout << "Connecting a signal to 'Dudette'" << std::endl;
	Signal<> signal_dudette;
	signal_dudette.connect("Dudette");
	signal_dudette.emit();
	SigSlotsManager<>::printStatistics();
	std::cout << std::endl;

	std::cout << "Creating a member data slot and connecting to 'DataDudette'" << std::endl;
	std::cout << "Connecting a data signal to 'DataDudette'" << std::endl;
	Signal<const int&> signal_data_dudette;
	signal_data_dudette.connect("DataDudette");
	signal_data_dudette.emit(3);
	std::cout << std::endl;
	SigSlotsManager<const int&>::printStatistics();

	return 0;
}
