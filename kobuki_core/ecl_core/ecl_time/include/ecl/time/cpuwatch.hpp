/**
 * @file /ecl_time/include/ecl/time/cpuwatch.hpp
 *
 * @brief Platform switcher for the cpu watch interface.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_CPUWATCH_HPP_
#define ECL_TIME_CPUWATCH_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config.hpp>

// Currently the cpuwatch timers are only defined for rt timers, so
// the cpuwatch class is similarly defined.
#if defined(ECL_IS_POSIX)
  // monotonic clock, cpu clock -> clock_gettime; clock_selection -> clock_nanosleep
  #if defined(_POSIX_MONOTONIC_CLOCK) && (_POSIX_MONOTONIC_CLOCK) >= 0L && defined(_POSIX_CLOCK_SELECTION) && (_POSIX_CLOCK_SELECTION) >= 0L
    #include "cpuwatch_rt.hpp"
  #else
    // No fallback available
  #endif
#endif

#endif /* ECL_TIME_CPUWATCH_HPP_ */
