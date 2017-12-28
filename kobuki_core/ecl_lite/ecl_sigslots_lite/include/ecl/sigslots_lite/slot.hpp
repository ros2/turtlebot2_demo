/**
 * @file /ecl_sigslots_lite/include/ecl/sigslots_lite/slot.hpp
 *
 * @brief Simple slots - these are actually all hidden from the user.
 *
 * @date Feb, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_SLOT_HPP_
#define ECL_SIGSLOTS_LITE_SLOT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstddef> // defines NULL

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {

/*****************************************************************************
** Slots
*****************************************************************************/

namespace sigslots {
/**
 * @brief Parent slot class with the common, publicly exposed interface.
 *
 * This allows member and global slots to be used as one and the same.
 *
 * @tparam Data : the function callback arg type (footprint).
 */
template <typename Data>
class SlotBase {
public:
	virtual void execute(Data data) = 0;
};
/**
 * @brief A slot with member callback function.
 *
 * @tparam Data : the argument type.
 * @tparam FunctionClass : the type of the class the function belongs too.
 */
template <typename Data, typename FunctionClass>
class MemberSlot : public SlotBase<Data> {
public:
	MemberSlot() {}
	MemberSlot(void (FunctionClass::*func)(Data), FunctionClass &instance) :
		object(&instance),
		member_function(func)
	{}

	void execute(Data data) {
		(*object.*member_function)(data);
	}
	FunctionClass *object;
	void (FunctionClass::*member_function)(Data);
private:
};


/**
 * @brief A slot with global/static callback function.
 *
 * @tparam Data : the argument type.
 */
template <typename Data>
class GlobalSlot : public SlotBase<Data> {
public:
	GlobalSlot() : global_function(NULL) {}
	GlobalSlot(void (*func)(Data)) : global_function(func) {}

	void execute(Data data) {
		if ( global_function != NULL ) {
			(*global_function)(data);
		}
	}
	void (*global_function)(Data); /**< @brief Slotted function. **/
};

/*****************************************************************************
** Slot Specialisations
*****************************************************************************/

/**
 * @brief Parent slot class with the common, publicly exposed interface.
 *
 * This allows member and global void slots to be used as one and the same.
 */
template <>
class SlotBase<void> {
public:
	virtual void execute() = 0;
};

/**
 * @brief A slot with member callback function.
 *
 * @tparam Data : the argument type.
 * @tparam FunctionClass : the type of the class the function belongs too.
 */
template <typename FunctionClass>
class MemberSlot<void,FunctionClass> : public SlotBase<void> {
public:
	MemberSlot() {}
	MemberSlot(void (FunctionClass::*func)(void), FunctionClass &instance) :
		object(&instance),
		member_function(func)
	{}

	void execute() {
		(*object.*member_function)();
	}
	FunctionClass *object;
	void (FunctionClass::*member_function)(void);
private:
};

/**
 * @brief A slot with global/static callback function.
 *
 * @tparam Data : the argument type.
 */
template <>
class GlobalSlot<void> : public SlotBase<void> {
public:
	GlobalSlot() : global_function(NULL) {}
	GlobalSlot(void (*func)(void)) : global_function(func) {}

	void execute() {
		if ( global_function != NULL ) {
			(*global_function)();
		}
	}
	void (*global_function)(void); /**< @brief Slotted function. **/
};

} // namespace sigslots
} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_SLOT_HPP_ */
