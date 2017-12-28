/**
 * @file /include/ecl/filesystem/macros.hpp
 *
 * @brief Macros for public, local export handling.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FILESYSTEM_MACROS_HPP_
#define ECL_FILESYSTEM_MACROS_HPP_

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
  #ifdef ecl_filesystem_EXPORTS // we are building a shared lib/dll
    #define ecl_filesystem_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_filesystem_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_filesystem_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_filesystem_PUBLIC
  #define ecl_filesystem_LOCAL
#endif


#endif /* ECL_FILESYSTEM_MACROS_HPP_*/
