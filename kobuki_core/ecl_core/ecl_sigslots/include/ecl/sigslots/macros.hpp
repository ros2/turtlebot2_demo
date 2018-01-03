/**
 * @file /include/ecl/sigslot/macros.hpp
 *
 * @brief Macros for ecl sigslots.
 *
 * @date May, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_MACROS_HPP_
#define ECL_SIGSLOTS_MACROS_HPP_

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
  #ifdef ecl_sigslots_EXPORTS // we are building a shared lib/dll
    #define ecl_sigslots_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_sigslots_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_sigslots_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_sigslots_PUBLIC
  #define ecl_sigslots_LOCAL
#endif

#endif /* ECL_SIGSLOTS_MACROS_HPP_ */
