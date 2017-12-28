/**
 * @file /include/ecl/time_lite/types.hpp
 *
 * @brief Cross platform definition for time types.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_TYPES_HPP_
#define ECL_TIME_LITE_TYPES_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/time_lite/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#if defined(ECL_HAS_WIN_TIMERS)
  #include "types_win.hpp"
#elif defined(ECL_HAS_MACH_TIMERS) || defined(ECL_HAS_POSIX_TIMERS) || defined(ECL_HAS_RT_TIMERS) // should probably get cmake to check for timespec.
  #include "types_pos.hpp"
#else
  #error("There is not a supporting time implementation on this platform (possibly needs extended ecl support).")
#endif

#endif /* ECL_TIME_LITE_TYPES_HPP_ */
