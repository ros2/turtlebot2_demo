/**
 * @file include/ecl/sigslots_lite/connect.hpp
 *
 * @brief Stores functions.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_CONNECT_HPP_
#define ECL_SIGSLOTS_LITE_CONNECT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "slot.hpp"
#include "signal.hpp"
#include "errors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

/*****************************************************************************
** Privately Used Connects
*****************************************************************************/

namespace sigslots {
/**
 * @brief Internal worker used to connect a signal with a slot.
 *
 * Valid return values:
 *
 * - NoError
 * - OutOfResourcesError : when the capacity of the signal is exceeded.
 *
 * @param signal : the signal for connection.
 * @param slot : the member slot for connection
 */
template <typename Data, unsigned int Capacity, typename FunctionClass>
ecl::lite::sigslots::Error connect(ecl::lite::Signal<Data, Capacity> &signal, MemberSlot<Data,FunctionClass> &slot) {
	return signal.connect(slot);
}

/**
 * @brief Internal worker used to connect a signal with a slot.
 *
 * Valid return values:
 *
 * - NoError
 * - OutOfResourcesError : when the capacity of the signal is exceeded.
 *
 * @param signal : the signal for connection.
 * @param slot : the global slot for connection
 */
template <typename Data, unsigned int Capacity>
ecl::lite::sigslots::Error connect(ecl::lite::Signal<Data, Capacity> &signal, GlobalSlot<Data> &slot) {
	return signal.connect(slot);
}

} // namespace sigslots

/*****************************************************************************
** Publicly Used Connects
*****************************************************************************/

/**
 * @brief Convenience method to connect signal with member function.
 *
 * This simply passes on the work to Signal's connect method.
 *
 * Valid return values:
 *
 * - NoError
 * - OutOfResourcesError : when the capacity of the member slots or the signal is exceeded.
 *
 * @param signal : the signal for connection.
 * @param f : the member function to slot.
 * @param o : the object associated with the member function.
 * @return Error : the sigslots error
 */
template <typename Data, unsigned int Capacity, typename FunctionClass>
sigslots::Error connect(Signal<Data,Capacity> &signal, void(FunctionClass::*f)(Data), FunctionClass &o) {
	// Need to do this just in case o has member slot interfaces for multiple Data types.
	// If we don't it can't resolve the footprint that seems to get lost when passing
	// the function pointers down the line - another way to resolve maybe?
	sigslots::MemberSlotsBase<Data,FunctionClass> &member_slots = o;
	sigslots::MemberSlot<Data,FunctionClass> *slot = member_slots.addSlot(f,o);
	if ( slot != NULL ) {
		return sigslots::connect(signal,*slot); // koenig lookup also works
	} else {
		return sigslots::Error(sigslots::OutOfResourcesError);
	}
}
/**
 * @brief Convenience method to connect signal with global/static function.
 *
 * This simply passes on the work to Signal's connect method.
 *
 * Valid return values:
 *
 * - NoError
 * - OutOfResourcesError : when the capacity of the global slots manager or the signal is exceeded.
 *
 * @param signal : the signal for connection.
 * @param function : the global/static function to slot.
 * @return Error : the sigslots error
 */
template <typename Data, unsigned int Capacity>
sigslots::Error connect(Signal<Data, Capacity> &signal, void (*function)(Data)) {
	sigslots::GlobalSlot<Data> *slot = GlobalSlots<Data>::addSlot(function);
	if ( slot != NULL ) {
		return sigslots::connect(signal,*slot);
	} else {
		return sigslots::Error(sigslots::OutOfResourcesError);
	}

}

/*****************************************************************************
** Publicly Used Connect Overloads
*****************************************************************************/

/**
 * @brief Convenience method to connect signal with member function.
 *
 * This simply passes on the work to Signal's connect method.
 *
 * @param signal : the signal for connection.
 * @param function : the member function to slot.
 * @param o : the object associated with the member function.
 * @return Error : the sigslots error
 */
template <unsigned int Capacity, typename FunctionClass>
sigslots::Error connect(Signal<void, Capacity> &signal, void(FunctionClass::*function)(void), FunctionClass &o) {
	// Need to do this just in case o has member slot interfaces for multiple Data types.
	// If we don't it can't resolve the footprint that seems to get lost when passing
	// the function pointers down the line - another way to resolve maybe?
	sigslots::MemberSlotsBase<void,FunctionClass> &member_slots = o;
	sigslots::MemberSlot<void,FunctionClass> *slot = member_slots.addSlot(function,o);
	if ( slot != NULL ) {
		return sigslots::connect(signal,*slot); // koenig lookup also works
	} else {
		return sigslots::Error(sigslots::OutOfResourcesError);
	}
}
/**
 * @brief Convenience method to connect signal with global/static function.
 *
 * This simply passes on the work to Signal's connect method.
 *
 * @param signal : the signal for connection.
 * @param function : the global/static function to slot.
 * @return Error : the sigslots error
 */
template <unsigned int Capacity>
sigslots::Error connect(Signal<void, Capacity> &signal, void (*function)(void)) {
	sigslots::GlobalSlot<void> *slot = GlobalSlots<void>::addSlot(function);
	if ( slot != NULL ) {
		return sigslots::connect(signal,*slot);
	} else {
		return sigslots::Error(sigslots::OutOfResourcesError);
	}

}

} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_CONNECT_HPP_ */
