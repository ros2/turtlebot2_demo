/**
 * @file /include/ecl/geometry/macros.hpp
 *
 * @brief Macros for ecl geometry.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_MACROS_HPP_
#define ECL_GEOMETRY_MACROS_HPP_

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
  #ifdef ecl_geometry_EXPORTS // we are building a shared lib/dll
    #define ecl_geometry_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_geometry_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_geometry_LOCAL ECL_HELPER_LOCAL
#else // ecl is being built around static libraries
  #define ecl_geometry_PUBLIC
  #define ecl_geometry_LOCAL
#endif

#endif /* ECL_GEOMETRY_MACROS_HPP_ */
