/**
 * @file /ecl_sigslots_lite/include/ecl/sigslots_lite/signal.hpp
 *
 * @brief Simple signal interface.
 *
 * Could probably set up a signal base for these, but its ok as is.
 *
 * @date Feb, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_SIGNAL_HPP_
#define ECL_SIGSLOTS_LITE_SIGNAL_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "slot.hpp"
#include "errors.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief A simple signal class.
 *
 * This class, once instantiated and connected is a fixed entity.
 * Memory reserved for slots is fixed and disconnects are not
 * possible This allows us to avoid the use of new and in most
 * low level control programs, such disconnects are rarely used
 * anyway.
 *
 * To reserve the capacity (i.e. no. of slots that can be attached)
 * configure the second template parameter upon instantiation. Note
 * that the default is 1.
 *
 * @code
 * void f(int i) {
 *   std::cout << i << std::endl;
 * }
 * int main() {
 *
 *     Signal<int,1> signal; // Signal<int> provides a default of 1
 *     connect(signal,f)
 *     signal.emit(3);
 *     // ...
 * @endcode
 *
 * @tparam Data : the data type to emit.
 * @tparam Capacity : the number of slot connections to reserve.
 */
template <typename Data=void, unsigned int Capacity = 1>
class Signal {
public:
	/**
	 * @brief Initialise the storage.
	 */
	Signal() {
		for ( unsigned int i = 0; i < Capacity; ++i) {
			slots[i] = NULL;
		}
	}
	/**
	 * @brief Connect the signal to the specified slot.
	 *
	 * This will attach the slot (note that it is a
	 * permanent attachment) so long as the reserved
	 * capacity isn't already fully utilised.
	 *
	 * Valid return values:
	 *
	 * - NoError
	 * - OutOfResourcesError
	 *
	 * @param slot : the slot to connect.
	 * @return Error : the sigslots error
	 */
	sigslots::Error connect(sigslots::SlotBase<Data> &slot) {
		for (unsigned int i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				slots[i] = &slot;
				return sigslots::Error(sigslots::NoError);
			}
		}
		// if we reach here, we've run out of resources.
		return sigslots::Error(sigslots::OutOfResourcesError);
	}
	/**
	 * @brief The reserved capacity for this signaller.
	 *
	 * @return unsigned int : the capacity.
	 */
	unsigned int capacity() const { return Capacity; }
	/**
	 * @brief The current number of connections stored.
	 *
	 * @return unsigned int : no. of connections.
	 */
	unsigned int stored() const {
		unsigned int i;
		for (i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				break;
			}
		}
		return i;

	}

	/**
	 * @brief Signal slots with the specified data.
	 *
	 * @param data : the data to send to the slots.
	 */
	void emit(Data data) const {
		for (unsigned int i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				break;
			} else {
				slots[i]->execute(data);
			}
		}
	}

private:
	sigslots::SlotBase<Data> *slots[Capacity];
};

/**
 * @brief Specialisation of the signal for data-less sigslots.
 *
 * Specialises the signal class for use with void callbacks. In
 * this situation, it is purely a signaller, nothing else.
 *
 * @tparam Capacity : the number of slot connections to reserve.
 */
template <unsigned int Capacity>
class Signal<void,Capacity> {
public:
	/**
	 * @brief Initialise the storage.
	 */
	Signal() {
		for ( unsigned int i = 0; i < Capacity; ++i) {
			slots[i] = NULL;
		}
	}
	/**
	 * @brief Connect the signal to the specified slot.
	 *
	 * This will attach the slot (note that it is a
	 * permanent attachment) so long as the reserved
	 * capacity isn't already fully utilised.
	 *
	 * Valid return values:
	 *
	 * - NoError
	 * - OutOfResourcesError
	 *
	 * @param slot : the slot to connect.
	 * @return Error : the sigslots error
	 */
	sigslots::Error connect(sigslots::SlotBase<void> &slot) {
		for (unsigned int i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				slots[i] = &slot;
				return sigslots::Error(sigslots::NoError);
			}
		}
		// if we reach here, we've run out of resources.
		return sigslots::Error(sigslots::OutOfResourcesError);
	}
	/**
	 * @brief The reserved capacity for this signaller.
	 *
	 * @return unsigned int : the capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief The current number of connections stored.
	 *
	 * @return unsigned int : no. of connections.
	 */
	unsigned int stored() const {
		unsigned int i;
		for (i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				break;
			}
		}
		return i;

	}
	/**
	 * @brief Signal slots with the specified data.
	 */
	void emit() const {
		for (unsigned int i = 0; i < Capacity; ++i ) {
			if ( slots[i] == NULL ) {
				break;
			} else {
				slots[i]->execute();
			}
		}
	}

private:
	sigslots::SlotBase<void> *slots[Capacity];
};


} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_SIGNAL_HPP_ */
