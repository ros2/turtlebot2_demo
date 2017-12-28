/**
 * @file /include/ecl/streams/manipulators/manipulator.hpp
 *
 * @brief Parent template definition for stream manipulators.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_MANIPULATOR_HPP_
#define ECL_STREAMS_MANIPULATOR_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/concepts/devices.hpp>
#include "../macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forwarding
*****************************************************************************/

namespace interfaces {
	template <typename Device, bool OutputDevice> class ECL_PUBLIC OutputTextStream;
}

/*****************************************************************************
** Interface [Manipulator]
*****************************************************************************/
/**
 * @brief Parent template for c++ style stream manipulators.
 *
 * This class uses a trick embedded within the curiously recurring template
 * method to allow static inheritance (important in this case because
 * the manipulator function call is template based).
 * Manipulators have an advantage over the usual c++ style manipulators in
 * that they are themselves classes (not functions such as std::endl). This
 * means that they can retain state from one insertion to the next.
 *
 * <b>Usage</b>:
 *
 * The manipulators defined in the ecl export a few global instantiations,
 * namely
 *
 * - ecl::endl
 * - ecl::clrscr
 *
 * Using them follows the same pattern as for standard c++ style cout manipulators.
 *
 * @code
 * using ecl::streams::OConsoleStream;
 * using ecl::streams::endl;
 * int main() {
 *     OConsoleStream ostream;
 *     ostream << clrscr;
 *     ostream << "Dude" << endl;
 *     return;
 * }
 * @endcode
 *
 * <b>Creating your own Manipulators:</b>
 *
 * Any manipulator that you wish to define must inherit from this
 * parent class in the following manner:
 *
 * @code
 * include <ecl/streams/manipulators.hpp>
 *
 * class MyManipulator : public ecl::Manipulator<MyManipulator> {
 *     template <typename OutputStream>
 *     void action (OutputStream& ostream) {
 *         // ...
 *     }
 * };
 * @endcode
 *
 * @tparam Derived : the child manipulator the crtp uses.
 *
 * @sa Manipulator.
 */
template <typename Derived>
class ecl_streams_PUBLIC Manipulator {
public:
	/**
	 * @brief The static crtp virtual parent call.
	 *
	 * This is exactly like a virtual function in a non-template c++
	 * parent class, except it works for templates in the derived type.
	 * This function calls the derived class' action() method with
	 * the same signature.
	 *
	 * @param ostream : the stream to use for manipulation.
	 *
	 * @tparam OutputStream : the output stream type.
	 *
	 * @sa Manipulator.
	 */
	template <typename ODevice>
	void insert (interfaces::OutputTextStream<ODevice,true>& ostream) {
		ecl_compile_time_concept_check(OutputCharDeviceConcept<ODevice>);
		static_cast<Derived*>(this)->action(ostream);
	}

	virtual ~Manipulator() {}
};

} // namespace ecl

#endif /* ECL_STREAMS_MANIPULATOR_HPP_ */
