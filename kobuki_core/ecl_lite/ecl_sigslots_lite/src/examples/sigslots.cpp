/**
 * @file /ecl_sigslots_lite/src/examples/sigslots.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 17/02/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/sigslots_lite.hpp"

/*****************************************************************************
** Foo
*****************************************************************************/
/**
 * @cond NoDoxygen
 */
class Foo : public ecl::lite::MemberSlots<const char*,Foo>,
			public ecl::lite::MemberSlots<void,Foo,2>
{
public:
	Foo() {}

	void f(const char* str) {
		std::cout << "Foo::f() " << str << std::endl;
	}
	void h() {
		std::cout << "Foo::h()" << std::endl;
	}
};

void f(const char* str) {
	std::cout << "f(char*) " << str << std::endl;
}

void g(const char* str) {
	std::cout << "g(char*) " << str << std::endl;
}

void h() {
	std::cout << "h()" << std::endl;
}

/*****************************************************************************
** Static Variables
*****************************************************************************/

template<>
const unsigned int ecl::lite::GlobalSlots<const char*>::capacity = 4;

template<>
const unsigned int ecl::lite::GlobalSlots<void>::capacity = 2;

/**
 * @endcond NoDoxygen
 */

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	Foo foo;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Data Sigslots" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	ecl::lite::Signal<const char*,2> dual_slot_signal;
	ecl::lite::Signal<const char*> lone_signal1, lone_signal2;
	// koenig lookup means a simple connect(dual_slot_signal,f) also works
	ecl::lite::connect(dual_slot_signal,f);
	ecl::lite::connect(dual_slot_signal,g);
	dual_slot_signal.emit("one signal, two global slots");
	std::cout << std::endl;

	ecl::lite::connect(lone_signal1,&Foo::f,foo);
	lone_signal1.emit("direct member slot connection");
	std::cout << std::endl;

	ecl::lite::connect(lone_signal2,&Foo::f,foo);
	lone_signal1.emit("multiple signals, one slot connection - I");
	lone_signal2.emit("multiple signals, one slot connection - II");
	std::cout << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Void Sigslots" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	ecl::lite::Signal<void> void_signal, member_void_signal;
	connect(member_void_signal,&Foo::h, foo);
	connect(void_signal,h);
	void_signal.emit();
	member_void_signal.emit();

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Storage" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	std::cout << "GlobalSlots<const char*> Stored  : " << ecl::lite::global_slots_stored<const char*>() << std::endl;
    std::cout << "GlobalSlots<const char*> Capacity: " << ecl::lite::global_slots_capacity<const char*>() << std::endl;
    std::cout << std::endl;
	std::cout << "GlobalSlots<void> Stored  : " << ecl::lite::global_slots_stored<void>() << std::endl;
    std::cout << "GlobalSlots<void> Capacity: " << ecl::lite::global_slots_capacity<void>() << std::endl;
    std::cout << std::endl;

	std::cout << "MemberSlots<const char*> Stored(foo)  : " << ecl::lite::member_slots_stored<const char*>(foo) << std::endl;
    std::cout << "MemberSlots<const char*> Capacity(Foo): " << ecl::lite::member_slots_capacity<const char*>(foo) << std::endl;
    std::cout << std::endl;
	std::cout << "MemberSlots<void> Stored(foo)  : " << ecl::lite::member_slots_stored<void>(foo) << std::endl;
    std::cout << "MemberSlots<void> Capacity(Foo): " << ecl::lite::member_slots_capacity<void>(foo) << std::endl;
    std::cout << std::endl;

	ecl::lite::Signal<const char*,2> signal_storage_test;
	std::cout << "Incrementally adding slots to signal<const char*,2>" << std::endl;
	std::cout << "  capacity: " << signal_storage_test.capacity() << std::endl;
	std::cout << "  stored  : " << signal_storage_test.stored() << std::endl;
	connect(signal_storage_test,f);
	std::cout << "  stored  : " << signal_storage_test.stored() << std::endl;
	connect(signal_storage_test,g);
	std::cout << "  stored  : " << signal_storage_test.stored() << std::endl;
    std::cout << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Error Handling" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    ecl::lite::sigslots::Error error;
    std::cout << "Adding two slots to a single slot capacity signal." << std::endl;
    ecl::lite::Signal<const char*> one_shot_signal;
    std::cout << "  Connecting a first slot" << std::endl;
    error = connect(one_shot_signal,f);
    if ( error.flag() != ecl::lite::sigslots::NoError ) {
    	std::cout << "  " << error.what() << std::endl;
    }
    std::cout << "  Connecting a second slot" << std::endl;
    error = connect(one_shot_signal,f);
    if ( error.flag() != ecl::lite::sigslots::NoError ) {
    	std::cout << "  " << error.what() << std::endl;
    }
	std::cout << std::endl;

    std::cout << "Adding two slots to a single slot capacity MemberSlots interface." << std::endl;
    ecl::lite::Signal<const char*> two_shot_signal;
    Foo foo_new; // 0 slots stored in MemberSlots<char*>
    std::cout << "  Connecting a first slot" << std::endl;
    error = connect(two_shot_signal, &Foo::f, foo_new); // 1 slot stored in MemberSlots<char*>
    if ( error.flag() != ecl::lite::sigslots::NoError ) {
    	std::cout << "  " << error.what() << std::endl;
    }
    std::cout << "  Connecting a second slot" << std::endl;
    error = connect(two_shot_signal, &Foo::f, foo_new); // 2nd slot, when only 1 declared, fail!
    if ( error.flag() != ecl::lite::sigslots::NoError ) {
    	std::cout << "  " << error.what() << std::endl;
    }

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	return 0;
}
