/**
 * @file /ecl_sigslots/include/ecl/sigslots/slot.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 13/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_SLOT_HPP_
#define ECL_SIGSLOTS_SLOT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "sigslot.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/utilities/void.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [General]
*****************************************************************************/

/**
 * @brief Function loading component of a callback system.
 *
 * Anywhere that a callback is required can be implemented with a slot. These
 * can be placed anywhere in your code and are initialised with either a free
 * (static or global) function, or a member function. Once initialised, they can
 * be hooked up to a signal.
 *
 * Usage examples are provided in the main page's documentation for this package.
 *
 * @sa Signal<Void>, Slot.
 **/
template <typename Data=Void>
class ECL_PUBLIC Slot {
public:
	/**
	 * @brief Load with a global/static function.
	 *
	 * Note that the function must have exactly one argument and it must
	 * be of the same type as the slot. It also must return void.
	 *
	 * @param f : the global/static function.
	 */
	Slot(void (*f)(Data)) : sigslot(NULL) {
		sigslot = new SigSlot<Data>(f);
	}
	/**
	 * @brief Load with a global/static function and connect.
	 *
	 * Note that the function must have exactly one argument and it must
	 * be of the same type as the slot. It also must return void.
	 *
	 * It additionally connects to the specified topic rapi style.
	 *
	 * @param f : the global/static function.
	 * @param topic : the slot topic name to connect to.
	 */
	Slot(void (*f)(Data), const std::string &topic) : sigslot(NULL) {
		sigslot = new SigSlot<Data>(f);
		connect(topic);
	}

	/**
	 * @brief Load with a member function.
	 *
	 * Note that the function must have exactly one argument and it must
	 * be of the same type as the slot. It also must return void.
	 *
	 * @param f : the member function.
	 * @param c : the class instance.
	 */
	template <typename C>
	Slot(void (C::*f)(Data), C &c) : sigslot(NULL) {
		sigslot = new SigSlot<Data>(f,c);
	}
	/**
	 * @brief Load with a member function and connect.
	 *
	 * Note that the function must have exactly one argument and it must
	 * be of the same type as the slot. It also must return void.
	 *
	 * It additionally connects to the specified topic rapi style.
	 *
	 * @param f : the member function.
	 * @param c : the class instance.
	 * @param topic : the slot topic name to connect to.
	 */
	template <typename C>
	Slot(void (C::*f)(Data), C &c, const std::string& topic) : sigslot(NULL) {
		sigslot = new SigSlot<Data>(f,c);
		connect(topic);
	}

	/**
	 * @brief Copy constructor.
	 *
	 * This is specially designed so that copying a slot is perfectly
	 * acceptable. Copies do not increase the number of callbacks that
	 * are fired - they just preserve the slot when used with
	 * things like stl containers. When the last copy disappears,
	 * the object destroys its connection automagically.
	 *
	 * @param slot : the object to be copied.
	 */
	Slot(const Slot& slot) {
		*this = slot;
		sigslot->incrHandles();
	}

	/**
	 * @brief Default destructor.
	 *
	 * This handles the cleanup operation, first decrementing and then
	 * checking if its the last of its instance. If it is, it cleans
	 * up the sigslot connection with the sigslots manager.
	 */
	~Slot() {
		sigslot->decrHandles();
		if ( sigslot->handles() == 0 ) {
			delete sigslot;
		}
	}
	/**
	 * @brief Lists the topics this slot is connected to.
	 *
	 * Useful for debugging.
	 * @return set<string> : a set of topic names this slot is listening to.
	 */
	const std::set<std::string>& connections() { return sigslot->subscribedTopics(); }
	/**
	 * @brief Make a connection to the specified topic.
	 *
	 * This contacts the sigslots manager to connect the signal to the
	 * specified topic - creating the topic if it is not yet existing.
	 *
	 * @param topic : the topic to connect to.
	 */
	void connect(const std::string& topic) {	sigslot->connectSlot(topic); }
	/**
	 * @brief Disconnect the slot from all topics.
	 *
	 * This completely disconnects the slot.
	 */
	void disconnect() { sigslot->disconnect(); }

private:
	SigSlot<Data>* sigslot;
};

/*****************************************************************************
** Interface [Void]
*****************************************************************************/
/**
 * @brief Specialised slot that handles void callbacks.
 *
 * Specialised signal that works with void function callbacks.
 *
 * Usage examples are provided in the main page's documentation for this package.
 *
 * @sa Signal, Slot.
 **/
template<>
class ECL_PUBLIC Slot<Void> {
public:
	/**
	 * @brief Load with a global/static function.
	 *
	 * Note that the function must have zero arguments and it must
	 * return void.
	 *
	 * @param f : the global/static function.
	 */
	Slot(VoidFunction f) : sigslot(NULL) {
		sigslot = new SigSlot<Void>(f);
	}
	/**
	 * @brief Load with a global/static function and connect.
	 *
	 * Note that the function must have zero arguments and it must
	 * return void.
	 *
	 * @param f : the global/static function.
	 * @param topic : the slot topic name to connect to.
	 */
	Slot(VoidFunction f, const std::string &topic) : sigslot(NULL) {
		sigslot = new SigSlot<Void>(f);
		connect(topic);
	}
	/**
	 * @brief Load with a member function.
	 *
	 * Note that the function must have zero arguments and it must
	 * return void.
	 *
	 * @param f : the member function.
	 * @param c : the class instance.
	 */
	template <typename C>
	Slot(void (C::*f)(void), C &c) : sigslot(NULL) {
		sigslot = new SigSlot<Void>(f,c);
	}
	/**
	 * @brief Load with a member function and connect.
	 *
	 * Note that the function must have zero arguments and it must
	 * return void.
	 *
	 * @param f : the member function.
	 * @param c : the class instance.
	 * @param topic : the slot topic name to connect to.
	 */
	template <typename C>
	Slot(void (C::*f)(void), C &c, const std::string &topic) : sigslot(NULL) {
		sigslot = new SigSlot<Void>(f,c);
		connect(topic);
	}
	/**
	 * @brief Copy constructor.
	 *
	 * This is specially designed so that copying a slot is perfectly
	 * acceptable. Copies do not increase the number of callbacks that
	 * are fired - they just preserve the slot when used with
	 * things like stl containers. When the last copy disappears,
	 * the object destroys its connection automagically.
	 *
	 * @param slot : the object to be copied.
	 */
	Slot(const Slot& slot) {
		*this = slot;
		sigslot->incrHandles();
	}
	/**
	 * @brief Default destructor.
	 *
	 * This handles the cleanup operation, first decrementing and then
	 * checking if its the last of its instance. If it is, it cleans
	 * up the sigslot connection with the sigslots manager.
	 */
	~Slot() {
		sigslot->decrHandles();
		if ( sigslot->handles() == 0 ) {
			delete sigslot;
		}
	}
	/**
	 * @brief Make a connection to the specified topic.
	 *
	 * This contacts the sigslots manager to connect the signal to the
	 * specified topic - creating the topic if it is not yet existing.
	 *
	 * @param topic : the topic to connect to.
	 */
	void connect(const std::string& topic) {	sigslot->connectSlot(topic); }
	/**
	 * @brief Disconnect the slot from all topics.
	 *
	 * This completely disconnects the slot.
	 */
	void disconnect() { sigslot->disconnect(); }

private:
	SigSlot<Void>* sigslot;

// unused:
//	  unsigned int copies() { return sigslot->handles(); }
};

} // namespace ecl

#endif /* ECL_SIGSLOTS_SLOT_HPP_ */
