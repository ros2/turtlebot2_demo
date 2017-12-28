/**
 * @file /include/ecl/time_lite/functions.hpp
 *
 * @brief Platform specific incantations of the time functions.
 *
 * Platform specific incantations of the time functions. These
 * are notoriously different depending on the platform - linux has librt,
 * mac osx the mach functions and windoze ???.
 *
 * These are only for use internally by the ecl time classes - a user should
 * never need to know them.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_FUNCTIONS_HPP_
#define ECL_TIME_LITE_FUNCTIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime>
#include <ecl/config/macros.hpp>
#include <ecl/time_lite/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#if defined(ECL_HAS_MACH_TIMERS)
  #include "functions_mac.hpp"
#elif defined(ECL_HAS_RT_TIMERS)
  // monotonic clock -> clock_gettime; clock_selection -> clock_nanosleep
  // #if defined(_POSIX_MONOTONIC_CLOCK) && (_POSIX_MONOTONIC_CLOCK) >= 0L && defined(_POSIX_CLOCK_SELECTION) && (_POSIX_CLOCK_SELECTION) >= 0L
  #include "functions_rt.hpp"
#elif defined(ECL_HAS_POSIX_TIMERS)
    #include "functions_pos.hpp"
#elif defined(ECL_HAS_WIN_TIMERS)
  // basic system time functions (horrid resolution and little functionality!)
  #include "functions_win.hpp"
#else
  #error("There is not a supporting time implementation on this platform (possibly needs extended ecl support).")
#endif

#endif /* ECL_TIME_LITE_FUNCTIONS_HPP_ */
