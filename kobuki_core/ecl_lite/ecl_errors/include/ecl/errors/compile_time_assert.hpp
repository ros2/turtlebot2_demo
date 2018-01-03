/**
 * @file /include/ecl/errors/compile_time_assert.hpp
 *
 * @brief Compile time checks.
 *
 * Macros and classes that allow compile time checking of conditional checks.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ERRORS_COMPILE_TIME_ASSERT_HPP_
#define ECL_ERRORS_COMPILE_TIME_ASSERT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {


/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @cond DOXYGEN_IGNORE_THIS
 */

template <bool> struct COMPILE_TIME_FAILURE;

template <> struct COMPILE_TIME_FAILURE<true> {};

template <int x> struct static_assert_test {};

}; // namespace ecl
/**
 * @endcond
 */

/**
 * @brief Compile time equivalent of the usual assert (ecl/c) functions.
 *
 * @ingroup Macros
 *
 * This macro tests logical conditions at compile time, useful for template classes.
 * When it fails, the compile aborts and flashes an eye catching message including
 * COMPILE_TIME_FAILURE in the output along with the line of code where the test fails.
 *
 * The idea for this comes from boost where a similar implementation is used in
 * the unit testing facility.
 */
#define ecl_compile_time_assert( logical_expression ) \
   typedef ecl::static_assert_test<sizeof(ecl::COMPILE_TIME_FAILURE< static_cast<bool>(logical_expression) >)> JOIN(compile_time_check,__LINE__)

/**
 * @brief Verbose compile time assert.
 *
 * @ingroup Macros
 *
 * This is similar to compile_time_assert, but simply adds a string message to
 * its list of arguments. Note - this string must not contain spaces - the
 * best way to use it is to use caps and underscores, then the message output
 * is in sync with the COMPILE_TIME_FAILURE that will be emitted when the
 * assertion fails.
 *
 * Recognition for this code must go to the author of the stones of nvwa (zlib license).
 *
 * http://sourceforge.net/projects/nvwa
 */
// If the (void)ERROR_##message is absent, we get unused variable warnings.
#define ecl_verbose_compile_time_assert( logical_expression, message ) \
	{ \
		ecl::COMPILE_TIME_FAILURE<( (logical_expression) != 0)> ERROR_##message; \
	    (void)ERROR_##message; \
	}

#endif /* ECL_ERRORS_COMPILE_TIME_ASSERT_HPP_ */
