/**
 * @file /include/ecl/sigslots_lite/managers.hpp
 *
 * @brief Management classes for sigslots lite.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_MANAGERS_HPP_
#define ECL_SIGSLOTS_LITE_MANAGERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "slot.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

template <typename Data, unsigned int Capacity> class Signal;

/*****************************************************************************
** Managers
*****************************************************************************/

namespace sigslots {
/**
 * @brief Used internally to retrieve info about members lots.
 */
template <typename Data, typename FunctionClass>
class MemberSlotsBase {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::MemberSlot<Data,FunctionClass>* addSlot(void (FunctionClass::*func)(Data), FunctionClass &instance) = 0;
};

/**
 * @brief Used internally to retrieve info about void member slots.
 */
template <typename FunctionClass>
class MemberSlotsBase<void,FunctionClass> {
public:
	virtual unsigned int stored() const { return 0; }
	virtual unsigned int capacity() const { return 0; }
	virtual sigslots::MemberSlot<void,FunctionClass>* addSlot(void (FunctionClass::*func)(), FunctionClass &instance) = 0;
};

} // namespace sigslots

/******************************************
** Member Manager
*******************************************/
/**
 * @brief This is the member slot interface, inheritable by classes.
 *
 * To enable slots for your class, simply inherit this interface.
 *
 * @code
 * class Foo : public ecl::lite::MemberSlots<const char*,Foo> {
 * public:
 * 	   void f(const char* str) {
 * 		   std::cout << "Foo::f() " << str << std::endl;
 * 	   }
 * };
 *
 * int main() {
 *     Foo foo;
 *     ecl::lite::Signal<const char*> signal;
 *     connect(signal,&Foo::f,foo);
 *     signal.emit("Dude");
 * @endcode
 *
 * @tparam Data : the footprint for the member function to slot.
 * @tparam FunctionClass : the class it will provide slots for.
 * @tparam Capacity : the number of functions from FunctionClass you'll slot.
 */
template <typename Data, typename FunctionClass, unsigned int Capacity = 1>
class MemberSlots : public sigslots::MemberSlotsBase<Data,FunctionClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <typename Data_, unsigned int Capacity_, typename FunctionClass_>
	friend sigslots::Error connect(Signal<Data_,Capacity_> &signal, void(FunctionClass_::*f)(Data_), FunctionClass_ &o);

	// needs access to stored()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_stored(const FunctionClass_ &object);

	// needs access to capacity()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_capacity(const FunctionClass_ &object);


protected:
	MemberSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }
	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return MemberSlot* : a pointer to a member slot.
	 */
	sigslots::MemberSlot<Data,FunctionClass>* addSlot(void (FunctionClass::*func)(Data), FunctionClass &instance) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].member_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::MemberSlot<Data,FunctionClass>(func,instance);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::MemberSlot<Data,FunctionClass> slots[Capacity];
};

/******************************************
** Global Manager
*******************************************/

/**
 * @brief This is the global slot interface.
 *
 * You'll need to preconfigure the capacity for this slot
 * interface. Note that the capacity corresponds to the
 * number of global functions with the specified footprint
 * (Data arg), not the total number of slottable global
 * functions overall.
 *
 * @code
 * // reserve the number of global slots that you'll use.
 * template<>
 * const unsigned int ecl::lite::GlobalSlots<const char*>::capacity = 2;
 *
 * void f(const char* str) {
 *     std::cout << "f() " << str << std::endl;
 * }
 *
 * void g(const char* str) {
 * 	   std::cout << "g() " << str << std::endl;
 * }
 *
 * int main() {
 *     ecl::lite::Signal<const char*,2> signal;
 *     connect(signal,f);
 *     connect(signal,g);
 *     signal.emit("Dude");
 * @endcode
 *
 * @tparam Data : the footprint for the type of global functions to slot.
 * @tparam Dummy : dummy argument to enable capacity to be a const expr for array bounds
 */
template <typename Data, typename Dummy = int>
class GlobalSlots {
public:
	static const unsigned int capacity;/**< @brief Number of global functions of this type that can be slotted. **/

	/*********************
	** Friends
	**********************/
	// needs access to addSlot()
	template <typename Data_, unsigned int Capacity_>
	friend sigslots::Error connect(Signal<Data_, Capacity_> &signal, void (*function)(Data_));

	// needs access to stored()
	template <typename Data_>
	friend unsigned int global_slots_stored();

private:
	/**
	 * @brief Number of slots currently stored.
	 *
	 * This is used for two purposes, 1) for recall and 2) to increment
	 * since it holds a static variable internally. This is triggered
	 * by the boolean argument which has a default value that can be hidden
	 * when doing 1).
	 *
	 * @param increment : increment the storage value (called from addSlot)
	 * @return : number of slots currently stored.
	 */
	static unsigned int stored(const bool increment = false) {
		static unsigned int stored_slots = 0;
		if ( increment ) { ++stored_slots; }
		return stored_slots;
	}
	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @return GlobalSlot* : a pointer to a global slot.
	 */
	static sigslots::GlobalSlot<Data>* addSlot(void (*func)(Data)) {
		unsigned int size = stored();
		static sigslots::GlobalSlot<Data> slots[capacity];
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].global_function ) {
				return &(slots[i]);
			}
		}
		if ( size < capacity ) {
			slots[size] = sigslots::GlobalSlot<Data>(func);
			bool inc = true;
			stored(inc); // increment the number of stored slots variable
			return &(slots[size]);
		} else {
			return NULL;
		}
	}
};

/*****************************************************************************
** Specialisations
*****************************************************************************/
/**
 * @brief Specialisation for void member slots management.
 */
template <typename FunctionClass, unsigned int Capacity>
class MemberSlots<void, FunctionClass, Capacity> : public sigslots::MemberSlotsBase<void,FunctionClass> {
public:
	/*********************
	** Friends
	**********************/
	// allow connect to use addSlot
	template <unsigned int Capacity_, typename FunctionClass_>
	friend sigslots::Error connect(Signal<void,Capacity_> &signal, void(FunctionClass_::*f)(void), FunctionClass_ &o);
	// needs access to stored()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_stored(const FunctionClass_ &object);

	// needs access to capacity()
	template <typename Data_, typename FunctionClass_>
	friend unsigned int member_slots_capacity(const FunctionClass_ &object);

protected:
	MemberSlots() : size(0) {};

private:
	/**
	 * @brief The number of slots stored.
	 *
	 * @return unsigned int : storage size.
	 */
	unsigned int stored() const { return size; }

	/**
	 * @brief The number of slots that can be attached to member functions.
	 *
	 * @return unsigned int : the maximum capacity.
	 */
	unsigned int capacity() const { return Capacity; }

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @param instance : the class instance associated with the function.
	 * @return MemberSlot* : a pointer to a member slot.
	 */
	sigslots::MemberSlot<void,FunctionClass>* addSlot(void (FunctionClass::*func)(void), FunctionClass &instance) {
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].member_function ) {
				return &(slots[i]);
			}
		}
		if ( size < Capacity ) {
			slots[size] = sigslots::MemberSlot<void,FunctionClass>(func,instance);
			++size;
			return &(slots[size-1]);
		} else {
			return NULL;
		}
	}

	unsigned int size;
	sigslots::MemberSlot<void,FunctionClass> slots[Capacity];
};

/**
 * @brief Specialisation for void global slots management.
 *
 * @tparam Dummy : dummy argument to enable capacity to be a const expr for array bounds
 */
template <typename Dummy>
class GlobalSlots<void, Dummy> {
public:
	static const unsigned int capacity; /**< @brief Number of global void functions that can be slotted. **/


	/*********************
	** Friends
	**********************/
	// needs access to addSlot()
	template <unsigned int Capacity_>
	friend sigslots::Error connect(Signal<void,Capacity_> &signal, void (*function)(void));

	// needs access to stored()
	template <typename Data> friend unsigned int global_slots_stored();

private:
	/**
	 * @brief Number of slots currently stored.
	 *
	 * This is used for two purposes, 1) for recall and 2) to increment
	 * since it holds a static variable internally. This is triggered
	 * by the boolean argument which has a default value that can be hidden
	 * when doing 1).
	 *
	 * @param increment : increment the storage value (called from addSlot)
	 * @return : number of slots currently stored.
	 */
	static unsigned int stored(const bool increment = false) {
		static unsigned int stored_slots = 0;
		if ( increment ) { ++stored_slots; }
		return stored_slots;
	}

	/**
	 * @brief Add a slot.
	 *
	 * This is used 'under the hood' by the connectors.
	 *
	 * @param func : the function to slot.
	 * @return GlobalSlot* : a pointer to a global slot.
	 */
	static sigslots::GlobalSlot<void>* addSlot(void (*func)(void)) {
		unsigned int size = stored();
		static sigslots::GlobalSlot<void> slots[capacity];
		for ( unsigned int i = 0; i < size; ++i ) {
			if ( func == slots[i].global_function ) {
				return &(slots[i]);
			}
		}
		if ( size < capacity ) {
			slots[size] = sigslots::GlobalSlot<void>(func);
			bool inc = true;
			stored(inc); // increment the number of stored slots variable
			return &(slots[size]);
		} else {
			return NULL;
		}
	}
};


} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_MANAGERS_HPP_ */
