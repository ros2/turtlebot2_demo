/**
 * @file /src/lib/functions_rt.cpp
 *
 * @brief Real-time (librt) implementation of ecl time functions.
 *
 * @date January 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time_lite/functions_rt.hpp"
#include <errno.h>

/*****************************************************************************
** Macros
*****************************************************************************/

#if defined(ECL_HAS_RT_TIMERS)

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

TimeError epoch_time(TimeStructure &time) {
	// _POSIX_CLOCK_MONOTONIC should have gotten us this far,
	// just check if we have CLOCK_MONOTONIC and if not, fallback to
	// CLOCK_REALTIME.
#ifdef ECL_HAS_CLOCK_MONOTONIC
	int result = clock_gettime(CLOCK_MONOTONIC,&time);
#else
	int result = clock_gettime(CLOCK_REALTIME,&time);
#endif
	switch (result) {
		case(0) : { return TimeError(NoError); }
		case(EFAULT) : { return TimeError(MemoryError); }          // time was not in addressable memory space
		case(EINVAL) : { return TimeError(ArgNotSupportedError); } // clock id is not supported (actually impossible if cmake detects)
		case(EPERM) : { return TimeError(PermissionsError); }      // user does not have permissions to use the clock.
		default : { return TimeError(UnknownError); }
	}
}

TimeError realtime_epoch_time(TimeStructure &time) {
        int result = clock_gettime(CLOCK_REALTIME,&time);
        switch (result) {
                case(0) : { return TimeError(NoError); }
                case(EFAULT) : { return TimeError(MemoryError); }          // time was not in addressable memory space
                case(EINVAL) : { return TimeError(ArgNotSupportedError); } // clock id is not supported (actually impossible if cmake detects)
                case(EPERM) : { return TimeError(PermissionsError); }      // user does not have permissions to use the clock.
                default : { return TimeError(UnknownError); }
        }
}

TimeError sleep_until(const TimeStructure &time) {
    // Last arg is to catch remaining time if interrupted by a signal, not necessary here.
#ifdef ECL_HAS_CLOCK_MONOTONIC
	int result = clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&time,NULL);
#else
	int result =  clock_nanosleep(CLOCK_REALTIME,TIMER_ABSTIME,&time,NULL);
#endif
	switch (result) {
		case(0) : { return TimeError(NoError); }
		case(EFAULT) : { return TimeError(MemoryError); }     // time was not in addressable memory space
		case(EINTR) : { return TimeError(InterruptedError); } // interrupted by a signal
		case(EINVAL) : { return TimeError(OutOfRangeError); } // sec/nsec pair specified was out of range
		default : { return TimeError(UnknownError); }
	}
}
TimeError sleep(const TimeStructure &time) {
	int result = nanosleep(&time, NULL);
	switch (result) {
		case(0) : { return TimeError(NoError); }
		case(EFAULT) : { return TimeError(MemoryError); }     // some memory error copying information around
		case(EINTR) : { return TimeError(InterruptedError); } // interrupted by a signal
		case(EINVAL) : { return TimeError(OutOfRangeError); } // sec/nsec pair specified was out of range
		default : { return TimeError(UnknownError); }
	}
}

} // namespace ecl

#endif
