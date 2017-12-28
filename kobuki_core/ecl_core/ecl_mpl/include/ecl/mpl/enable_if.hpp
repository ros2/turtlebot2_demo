/**
 * @file /ecl_mpl/include/ecl/mpl/enable_if.hpp
 *
 * @brief C++0x's enable_if extensions (fill-in till they're available).
 *
 * @date 29/07/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MPL_ENABLE_HPP_
#define ECL_MPL_ENABLE_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Conditional class for (true) implementation of enable_if.
 *
 * This is not for direct use, it just provides implementation.
 */
template <bool B, class T = void>
struct enable_if_c
{
	typedef T type;
};

/**
 * @brief Conditional class for (false) implementation of enable_if.
 *
 * This is not for direct use, it just provides implementation.
 */
template <class T>
struct enable_if_c<false, T>
{};

/**
 * @brief Enables the SFINAE concept.
 *
 * This is the same as the boost and future C++0x implementations.
 * Use this to do things like helping to instantiate families
 * of specialisations. For example:
 *
 * @code
#include <ecl/type_traits/fundamental_types.hpp> // for is_float

// This will instantiate if it is anything except float or double.
template <typename T, typename Enable = void>
class TestObject {
public:
    bool isFloatSpecialisation() { return false; }
};

// This specialisation will instantiate it T is float or double.
template <typename T>
class TestObject< T, typename enable_if< is_float<T> >::type > {
public:
    bool isFloatSpecialisation() { return true; }
};
 * @endcode
 */
template <class Condition, class T = void>
struct enable_if : public enable_if_c<Condition::value, T>
{};

} //namespace ecl

#endif /* ECL_MPL_ENABLE_HPP_ */
