/**
 * @file /include/ecl/formatters/macros.hpp
 *
 * @brief Macros for ecl formatters.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FORMATTERS_MACROS_HPP_
#define ECL_FORMATTERS_MACROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Declspecs
*****************************************************************************/

/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef ecl_formatters_EXPORTS // we are building a shared lib/dll
    #define ecl_formatters_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_formatters_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_formatters_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_formatters_PUBLIC
  #define ecl_formatters_LOCAL
#endif

#endif /* ECL_FORMATTERS_MACROS_HPP_ */
