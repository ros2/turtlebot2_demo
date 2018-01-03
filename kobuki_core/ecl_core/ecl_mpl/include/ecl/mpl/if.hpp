/**
 * @file /include/ecl/mpl/bool.hpp
 *
 * @brief Defines the metaprogamming equivalent of the boolean type.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MPL_IF_HPP_
#define ECL_MPL_IF_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief The metaprogramming equivalent of the 'if' function.
 *
 * This defines a compile time metafunctional version of the 'if' function. It
 * configures the default response (that for when the boolean condition is true).
 * The alternative (false) is configured in a specialisation.
 *
 * @tparam Condition : the logical condition to test.
 * @tparam T1 : the return type if true.
 * @tparam T2 : the return type if false.
 *
 * @sa if_c<false,T1,T2>.
 */
template <bool Condition, typename T1, typename T2>
struct if_c {
	typedef T1 type;
};

/**
 * @brief Specialisation of the metaprogrammed 'if' statement when false.
 *
 * This defines the negative (false) response for the compile time metafunctional
 * version of the 'if' function.
 *
 * @tparam T1 : the return type if true.
 * @tparam T2 : the return type if false.
 *
 * @sa if_c<Condition,T1,T2>.
 */
template <typename T1, typename T2>
struct if_c<false,T1,T2> {
	typedef T2 type;
};

} // namespace ecl

#endif /* ECL_MPL_IF_HPP_ */
