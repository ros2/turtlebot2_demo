/**
 * @file /src/lib/functions_mac.cpp
 *
 * @brief Mac implementation of ecl time functions.
 *
 * @date January 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time_lite/functions_mac.hpp"
#include <errno.h>

/*****************************************************************************
** Macros
*****************************************************************************/

#if defined(ECL_HAS_MACH_TIMERS)

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

TimeError epoch_time(TimeStructure &time) {
	struct timeval tv;
	int result = gettimeofday(&tv, NULL);
    if ( result == 0 ) {
	    time.tv_sec = tv.tv_sec;
	    time.tv_nsec = tv.tv_usec*1000;
	    return TimeError(NoError);
    } else {
    	switch (errno) {
			case(EFAULT) : { return TimeError(MemoryError); }          // time was not in addressable memory space
			case(EINVAL) : { return TimeError(ArgNotSupportedError); } // timezone (or something else) is invalid
			default : { return TimeError(UnknownError); }
		}
    }
}

TimeError sleep_until(const TimeStructure &time) {
	TimeStructure current_time, sleep_time;

	TimeError error = epoch_time(current_time);
	if ( error.flag() != NoError ) { return error; }

	/*********************
	** current > time?
	**********************/
	if ( current_time.tv_sec > time.tv_sec ) {
		return TimeError(NoError); // return immediately
	} else if ( current_time.tv_sec == time.tv_sec ) {
		if ( current_time.tv_nsec > time.tv_nsec ) {
			return TimeError(NoError);  // return immediately
		}
	}
	sleep_time.tv_sec = time.tv_sec - current_time.tv_sec;
	if ( current_time.tv_nsec <= time.tv_nsec ) {
		sleep_time.tv_nsec = time.tv_nsec - current_time.tv_nsec;
	} else {
		sleep_time.tv_sec -= 1;
		sleep_time.tv_nsec = 1000000000L - current_time.tv_nsec + time.tv_nsec;
	}
	int result = nanosleep(&sleep_time, NULL);
	switch (result) {
		case(0) : { return TimeError(NoError); }
		case(EFAULT) : { return TimeError(MemoryError); }     // some memory error copying information around
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
