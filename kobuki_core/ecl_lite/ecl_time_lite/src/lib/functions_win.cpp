/**
 * @file /src/lib/functions_win.cpp
 *
 * @brief Windows implementation of ecl time functions.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/windows.hpp>
#include <ecl/config/portable_types.hpp>
#include "../../include/ecl/time_lite/functions_win.hpp"

/*****************************************************************************
** Macros
*****************************************************************************/

#if defined(ECL_HAS_WIN_TIMERS)

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Notes
*****************************************************************************/

// This is defined by windows and looks like this:
//typedef union _LARGE_INTEGER {
//	struct {
//		DWORD LowPart;
//		LONG HighPart;
//	};
//	LONGLONG QuadPart;
//} LARGE_INTEGER;
//typedef LARGE_INTEGER TimeStructure; /**< @brief The underlying type used to store time structures. **/

/*****************************************************************************
** Private functions
*****************************************************************************/

static double cpu_frequency() {
	static double frequency = 0.0;

	if ( frequency == 0.0 ) { // need to initialise
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		frequency = static_cast<double>(freq.QuadPart);
	}
	return frequency;
}

/*****************************************************************************
** Public functions
*****************************************************************************/

TimeError epoch_time(TimeStructure &time) {
	// Get time
    LARGE_INTEGER stamp;
    QueryPerformanceCounter(&stamp);
	double t = static_cast<double>(stamp.QuadPart)/cpu_frequency();
	time.tv_sec = static_cast<long>(t);
	time.tv_nsec = ((t - static_cast<double>(time.tv_sec))*1000000000.0);
	return TimeError(NoError);
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
	    	  return TimeError(NoError);
		}
	}
	sleep_time.tv_sec = time.tv_sec - current_time.tv_sec;
	if ( current_time.tv_nsec <= time.tv_nsec ) {
		sleep_time.tv_nsec = time.tv_nsec - current_time.tv_nsec;
	} else {
		sleep_time.tv_sec -= 1;
		sleep_time.tv_nsec = 1000000000L - current_time.tv_nsec + time.tv_nsec;
	}
	return ecl::sleep(sleep_time);
}

TimeError sleep(const TimeStructure &time) {
    HANDLE timer = NULL;
    LARGE_INTEGER sleep_time;

    sleep_time.QuadPart = -
            static_cast<ecl::uint64>(time.tv_sec)*10000000LL -
            static_cast<ecl::uint64>(time.tv_nsec) / 100LL;

      if ( (timer = CreateWaitableTimer(NULL, TRUE, NULL)) == NULL ) {
    	  return TimeError(UnknownError);
      }
      if (!SetWaitableTimer (timer, &sleep_time, 0, NULL, NULL, 0)) {
    	  return TimeError(UnknownError);
      }

      if (WaitForSingleObject (timer, INFINITE) != WAIT_OBJECT_0) {
    	  return TimeError(UnknownError);
      }
      return TimeError(NoError);
}


} // namespace ecl

#endif
