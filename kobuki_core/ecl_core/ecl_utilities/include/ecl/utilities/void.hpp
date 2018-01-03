/**
 * @file /include/ecl/utilities/void.hpp
 *
 * @brief Void/null object and function definitions.
 *
 * A void (null) object that is occasionally useful, particularly as a
 * null argument to template specialisations. Also typedefs the void
 * function pointer for convenience.
 *
 * @sa Void.
 *
 * @date August 2007
 *
 **/
/*****************************************************************************
** Defines
*****************************************************************************/

#ifndef ECL_UTILITIES_VOID_HPP_
#define ECL_UTILITIES_VOID_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Typedefs
*****************************************************************************/
/**
 * @brief Convenient type definition for empty functions (void pointers).
 *
 * Convenient type definition for empty functions (void pointers). Wherever
 * you would normally use
 * @code
 * void(*)()
 * @endcode
 * you can instead replace with the more convenient
 * @code
 * VoidFunction
 * @endcode
 **/
typedef void (*VoidFunction)();

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief A void (null) object.
 *
 * A void (null) object that is occasionally useful for template instantiations.
 * When you want to instantiate a template class without a type, just set
 * its default type to this void object and create a specialisation of that
 * template class.
 **/
class Void {
public:
	template <typename OutputStream>
	friend OutputStream& operator << (OutputStream& ostream, const Void void_object);

	virtual ~Void() {}
};

/**
 * @brief Output stream operator for Void objects.
 *
 * This function not serve any purpose except to allow the stream to move
 * on to the next output object.
 *
 * @param ostream : the output stream.
 * @param void_object : the void object to ignore.
 * @return OutputStream : return the output stream as is.
 **/
template <typename OutputStream>
OutputStream& operator << (OutputStream& ostream, const Void void_object) { return ostream; }

}; // Namespace ecl



#endif /*ECL_UTILITIES_VOID_HPP_*/
