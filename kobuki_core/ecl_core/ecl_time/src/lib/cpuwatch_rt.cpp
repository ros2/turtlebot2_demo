/**
 * @file /src/lib/cpuwatch_rt.cpp
 *
 * @brief Posix rt-timer implementation of the cpuwatch class.
 *
 * @date June, 2009
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

#if defined(ECL_IS_POSIX)
  // monotonic clock, cpu clock -> clock_gettime; clock_selection -> clock_nanosleep
  #if defined(_POSIX_MONOTONIC_CLOCK) && (_POSIX_MONOTONIC_CLOCK) >= 0L && defined(_POSIX_CLOCK_SELECTION) && (_POSIX_CLOCK_SELECTION) >= 0L


/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time/cpuwatch_rt.hpp"
#include <ecl/time_lite/cpu_time.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [CpuWatch]
*****************************************************************************/

CpuWatch::CpuWatch() {
	cpu_time(tmp);
	start_time.stamp(tmp.tv_sec, tmp.tv_nsec);
	split_time = start_time;
};

/*****************************************************************************
** Implementation
*****************************************************************************/

void CpuWatch::restart()
{
	cpu_time(tmp);
	start_time.stamp(tmp.tv_sec, tmp.tv_nsec);
    split_time = start_time;
}

TimeStamp CpuWatch::elapsed()
{
	cpu_time(tmp);
    TimeStamp current_time(tmp.tv_sec, tmp.tv_nsec);
    return ( current_time - start_time );
}

TimeStamp CpuWatch::split()
{
    TimeStamp last_time = split_time;
	cpu_time(tmp);
    split_time.stamp(tmp.tv_sec,tmp.tv_nsec);
    return (split_time - last_time);
}

}; // namespace ecl

#endif /* MANY POSIX TIME REQ'MENTS */
#endif /* ECL_IS_POSIX */

