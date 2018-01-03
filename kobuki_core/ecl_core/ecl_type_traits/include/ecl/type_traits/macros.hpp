/**
 * @file /include/ecl/type_traits/macros.hpp
 *
 * @brief Macros for windows dlls.
 *
 * @date March 2013.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TYPE_TRAITS_MACROS_HPP_
#define ECL_TYPE_TRAITS_MACROS_HPP_

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
  #ifdef ecl_type_traits_EXPORTS // we are building a shared lib/dll
    #define ecl_type_traits_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_type_traits_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_type_traits_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_type_traits_PUBLIC
  #define ecl_type_traits_LOCAL
#endif


#endif /* ECL_TYPE_TRAITS_MACROS_HPP_*/
