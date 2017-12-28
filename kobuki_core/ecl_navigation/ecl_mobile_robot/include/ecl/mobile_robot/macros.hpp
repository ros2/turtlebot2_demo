/**
 * @file /include/ecl/mobile_robot/macros.hpp
 *
 * @brief Macros for ecl mobile robot.
 *
 * @date April, 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MOBILE_ROBOT_MACROS_HPP_
#define ECL_MOBILE_ROBOT_MACROS_HPP_

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
  #ifdef ecl_mobile_robot_EXPORTS // we are building a shared lib/dll
    #define ecl_mobile_robot_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_mobile_robot_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_mobile_robot_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_mobile_robot_PUBLIC
  #define ecl_mobile_robot_LOCAL
#endif

#endif /* ECL_DEVICES_MACROS_HPP_ */
