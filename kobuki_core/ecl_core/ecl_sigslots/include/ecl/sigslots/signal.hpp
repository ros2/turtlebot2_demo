/**
 * @file /ecl_sigslots/include/ecl/sigslots/signal.hpp
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

#ifndef ECL_SIGSLOTS_SIGNAL_HPP_
#define ECL_SIGSLOTS_SIGNAL_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "sigslot.hpp"
#include <ecl/utilities/void.hpp>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [General]
*****************************************************************************/
/**
 * @brief Signalling component of a callback system.
 *
 * Anywhere that triggers an event requiring a callback to be
 * executed can be implemented with a signal. These can be placed anywhere in your
 * code and with slots, provide a many to many callback solution.
 *
 * Usage examples are provided in the main page's documentation for this package.
 *
 * @sa Signal<Void>, Slot.
 **/

template <typename Data=Void>
class ECL_PUBLIC Signal {
public:
	/**
	 * @brief Default constructor.
	 *
	 * This creates a signal with no connections yet established.
	 * Use with connect().
	 */
	Signal() : sigslot(NULL) {
		sigslot = new SigSlot<Data>();
	}
	/**
	 * @brief Creates a signal and connects.
	 */
	Signal(const std::string &topic) : sigslot(NULL) {
		sigslot = new SigSlot<Data>();
		connect(topic);
	}

	/**
	 * @brief Copy constructor.
	 *
	 * This is specially designed so that copying a signal is perfectly
	 * acceptable. Copies do not increase the number of emits that
	 * are fired - they just preserve the signal when used with
	 * things like stl containers. When the last copy disappears,
	 * the object destroys its connection automagically.
	 *
	 * @param signal : the object to be copied.
	 */
	Signal(const Signal& signal) {
		*this = signal;
		sigslot->incrHandles();
	}

	/**
	 * @brief Default destructor.
	 *
	 * This handles the cleanup operation, first decrementing and then
	 * checking if its the last of its instance. If it is, it cleans
	 * up the sigslot connection with the sigslots manager.
	 */
	~Signal() {
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
	void connect(const std::string& topic) { sigslot->connectSignal(topic); }
	/**
	 * @brief Connect as a slot, with the emit function loaded.
	 *
	 * This allows the signal to act as a relaying signal (effectively
	 * a slot with the emit() function loaded.
	 *
	 * @param topic : the topic to connect to.
	 */
	void connectAsSlot(const std::string& topic) { sigslot->connectSlot(topic); }
	/**
	 * @brief Disconnect the signal from all topics.
	 *
	 * This completely disconnects the signal.
	 */
	void disconnect() { sigslot->disconnect(); }

	/**
	 * @brief The primary purpose of the signal, to emit!
	 *
	 * Emits a signal with the specified data.
	 *
	 * @param data : the data to emit.
	 */
	void emit(Data data) { sigslot->emit(data); }

private:
	SigSlot<Data>* sigslot;
};

/*****************************************************************************
** Interface [Void]
*****************************************************************************/
/**
 * @brief Specialised signal that emits without passing data.
 *
 * Specialised signal that only emits a signal but passes no data to their
 * respective slots. This is the default template construction for a
 * signal, so you need only use Signal<> to represent the type.
 *
 * Usage examples are provided in the main page's documentation for this package.
 *
 * @sa Signal, Slot.
**/
template<>
class ECL_PUBLIC Signal<Void> {
public:
	/**
	 * @brief Default constructor.
	 *
	 * This creates a signal but with no default connection. Use
	 * connect() to establish a connection.
	 */
	Signal() : sigslot(NULL) {
		sigslot = new SigSlot<Void>();
	}
	/**
	 * @brief Creates a signal and connects.
	 */
	Signal(const std::string &topic) : sigslot(NULL) {
		sigslot = new SigSlot<Void>();
		connect(topic);
	}
	/**
	 * @brief Copy constructor.
	 *
	 * This is specially designed so that copying a signal is perfectly
	 * acceptable. Copies do not increase the number of emits that
	 * are fired - they just preserve the signal when used with
	 * things like stl containers. When the last copy disappears,
	 * the object destroys its connection automagically.
	 *
	 * @param signal : the object to be copied.
	 */
	Signal(const Signal& signal) {
		*this = signal;
		sigslot->incrHandles();
	}
	/**
	 * @brief Default destructor.
	 *
	 * This handles the cleanup operation, first decrementing and then
	 * checking if its the last of its instance. If it is, it cleans
	 * up the sigslot connection with the sigslots manager.
	 */
	~Signal() {
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
	void connect(const std::string& topic) { sigslot->connectSignal(topic); }
	/**
	 * @brief Connect as a slot, with the emit function loaded.
	 *
	 * This allows the signal to act as a relaying signal (effectively
	 * a slot with the emit() function loaded.
	 *
	 * @param topic : the topic to connect to.
	 */
	void connectAsSlot(const std::string& topic) { sigslot->connectSlot(topic); }
	/**
	 * @brief Disconnect the signal from all topics.
	 *
	 * This completely disconnects the signal.
	 */
	void disconnect() { sigslot->disconnect(); }

	/**
	 * @brief The primary purpose of the signal, to emit!
	 *
	 * Emits a signal with no data.
	 */
	void emit() { sigslot->emit(); }

private:
	SigSlot<Void>* sigslot;

};

} // namespace ecl

#endif /* ECL_SIGSLOTS_SIGNAL_HPP_ */
