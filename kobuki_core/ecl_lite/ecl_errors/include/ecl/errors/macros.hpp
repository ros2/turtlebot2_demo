/**
 * @file /include/ecl/errors/macros.hpp
 *
 * @brief Macros for error-checking and program termination.
 *
 * Macros for error-checking and program termination.
 *
 * @date April 2013.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ERRORS_MACROS_HPP_
#define ECL_ERRORS_MACROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>

/*****************************************************************************
** PreProcessing
*****************************************************************************/
/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef ecl_errors_EXPORTS // we are building a shared lib/dll
    #define ecl_errors_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_errors_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_errors_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_errors_PUBLIC
  #define ecl_errors_LOCAL
#endif

/*
 * Convert a variable name to a string.
 */
#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

/*
 * Joints two arguments together.
 * The following piece of macro magic joins the two
 * arguments together, even when one of the arguments is
 * itself a macro (see 16.3.1 in C++ standard).  The key
 * is that macro expansion of macro arguments does not
 * occur in BOOST_DO_JOIN2 but does in BOOST_DO_JOIN.
 */
#define JOIN( X, Y ) DO_JOIN( X, Y )
#define DO_JOIN( X, Y ) DO_JOIN2(X,Y)
#define DO_JOIN2( X, Y ) X##Y

/**
 * @ingroup Macros
 *
 * @brief Stringify the line of code you are at.
 *
 * This uses a couple of simpler string macros to indicate in which file and at what
 * line this piece of code is at.
 */
#define LOC __FILE__ ":" TOSTRING(__LINE__) " "


#endif /* ECL_ERRORS_MACROS_HPP_*/
