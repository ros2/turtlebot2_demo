/**
 * @file /ecl_time_lite/src/lib/cpu_time.cpp
 *
 * @brief Implemntation of the cpu timer.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time_lite/cpu_time.hpp"
#include <errno.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

#ifdef ECL_HAS_CPUTIME
TimeError cpu_time(TimeStructure &time) {
	// On the man page, it says that this function should need _POSIX_CPUTIME
	// defined, but in practice I'm a bit unusre about this. On one system,
	// I've had _POSIX_CPUTIME defined to 0, but this function would still work.
	// Just assuming that you need clock_gettime for now.
	int result = clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&time);
	switch (result) {
		case(0) : { return TimeError(NoError); }
		case(EFAULT) : { return TimeError(MemoryError); }          // time was not in addressable memory space
		case(EINVAL) : { return TimeError(ArgNotSupportedError); } // clock id is not supported (actually impossible if cmake detects)
		case(EPERM) : { return TimeError(PermissionsError); }      // user does not have permissions to use the clock.
		default : { return TimeError(UnknownError); }
	}
}
#endif

} // namespace ecl
