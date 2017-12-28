/**
 * @file /ecl_sigslots/src/test/sigslots.cpp
 *
 * @brief Unit test the sigslots api.
 *
 * @date May 16, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/


#include <iostream>
#include <gtest/gtest.h>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/sigslots/signal.hpp"
#include "../../include/ecl/sigslots/slot.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::StandardException;
using ecl::Signal;
using ecl::Slot;

/**
 * @cond DO_NOT_DOXYGEN
 */
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace sigslots {
namespace tests {

/*****************************************************************************
** Classes
*****************************************************************************/

class A {
public:
	A() : f_count(0), g_count(0) {}

	void f(const int &i) {
//	    std::cout << "  Member function with argument: " << i << std::endl;;
		f_count++;
	}
	void g() {
		g_count++;
//	    std::cout << "  Member function" << std::endl;;
	}
	int f_count, g_count;
};

/*****************************************************************************
** Functions
*****************************************************************************/

static int f_count = 0;
static int g_count = 0;

void f(const int &i)
{
	f_count++;
//    std::cout << "  Global function with argument: " << i << std::endl;;
}

void g()
{
	g_count++;
//    std::cout << "  Global function" << std::endl;;
}

} // namespace tests
} // namespace signals
} // namespace ecl

/*****************************************************************************
** Using
*****************************************************************************/

using namespace ecl::sigslots::tests;

/**
 * @endcond
 */
/*****************************************************************************
** Tests
*****************************************************************************/

TEST(SigSlotsTests, voidSigSlots) {

	A a;
    Signal<> sig_void;

    Slot<> slot_void0(g,"void_test");
    Slot<> slot_void1(&A::g,a);

    sig_void.connect("void_test");
//    slot_void0.connect("void_test");
    slot_void1.connect("void_test");

    sig_void.emit();

//    ecl::SigSlotsManager<>::printStatistics();

//    std::cout << "a.g_count: " << a.g_count << std::endl;
//    std::cout << "g_count: " << g_count << std::endl;
    EXPECT_EQ(1,a.g_count);
    EXPECT_EQ(1,g_count);
}

TEST(SigSlotsTests, relay) {

	A a;
    Signal<> sig_starter;
    Signal<> sig_relay;

    Slot<> slot_relayed_goal(g);

    sig_starter.connect("relay");
    sig_relay.connectAsSlot("relay");
    sig_relay.connect("final_goal");
    slot_relayed_goal.connect("final_goal");

    sig_starter.emit();

//    std::cout << "g_count: " << g_count << std::endl;
    EXPECT_EQ(2,g_count);
}

TEST(SigSlotsTests,dataSigSlots) {

	A a;
    Signal<const int&> sig_int;
    Slot<const int&> slot_int0(f);
    Slot<const int&> slot_int1(&A::f,a);

    sig_int.connect("int_test");
    slot_int0.connect("int_test");
    slot_int1.connect("int_test");

    sig_int.emit(4);
//	std::cout << "a.f_count: " << a.f_count << std::endl;
//	std::cout << "f_count: " << f_count << std::endl;
    EXPECT_EQ(1,a.f_count);
    EXPECT_EQ(1,f_count);
}

TEST(SigSlotsTests, disconnect) {

    Signal<> signal;
    Slot<> slot(g);
    signal.connect("disconnect");
    slot.connect("disconnect");
    signal.emit();
    slot.disconnect();
    Signal<const int&> data_signal;
    Slot<const int&> data_slot(f);
    data_signal.connect("disconnect"); // Each data type has its own 'namespace' per se, so this doesn't clash with the above.
    data_slot.connect("disconnect");
    data_signal.emit(1);
    data_slot.disconnect();
    data_signal.emit(1); // Shouldn't increment

    EXPECT_EQ(3,g_count);
    EXPECT_EQ(2,f_count);
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
