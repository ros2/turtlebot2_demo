/**
 * @file /src/lib/timestamp_pos.cpp
 *
 * @brief Posix rt-timer implementation of the timestamp class.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <errno.h>
#include <sys/time.h>
#include <ecl/config/ecl.hpp>
#include <ecl/time_lite/functions.hpp>
#include "../../include/ecl/time/timestamp_pos.hpp"
#include <ecl/exceptions/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [TimeStamp]
*****************************************************************************/

TimeStamp::TimeStamp() ecl_debug_throw_decl(StandardException) {
	stamp();
}

TimeStamp::TimeStamp (const double& decimal_time_value) ecl_assert_throw_decl(StandardException) :
	TimeStampBase(decimal_time_value)
{}

TimeStamp::TimeStamp (const time_t& seconds, const long& nanoseconds) ecl_assert_throw_decl(StandardException) :
	TimeStampBase(seconds, nanoseconds)
{}

TimeStamp::TimeStamp(const TimeStampBase& base) : TimeStampBase(base) {}

/*****************************************************************************
** Implementation [Stamps]
*****************************************************************************/

const TimeStamp& TimeStamp::stamp() ecl_debug_throw_decl(StandardException) {
	if ( epoch_time(time).flag() != NoError ) {
		ecl_debug_throw(time::throwTimeStampException(LOC));
	}
    return (*this);
}

#if defined(ECL_HAS_RT_TIMERS)
TimeStamp TimeStamp::realtime_now() ecl_debug_throw_decl(StandardException) {
        TimeStructure time;
        if ( realtime_epoch_time(time).flag() != NoError ) {
                ecl_debug_throw(time::throwTimeStampException(LOC));
        }
        return TimeStamp(time.tv_sec, time.tv_nsec);
}
#endif
}; // namespace ecl

/*****************************************************************************
** Implementation [Exceptions]
*****************************************************************************/

#ifdef ECL_HAS_EXCEPTIONS
namespace ecl {
namespace time {

StandardException throwTimeStampException(const char* loc) {
	int error_result = errno;
    switch (error_result) {
        case ( EINVAL ) : return ecl::StandardException(loc, ecl::NotSupportedError, "The requested clock is not supported on this system.");
        case ( EFAULT ) : return ecl::StandardException(loc, ecl::OutOfRangeError, "The timespec pointer points outside the address space.");
        default         :
        {
			std::ostringstream ostream;
			ostream << "Unknown posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, ecl::UnknownError, ostream.str());
		}
    }
}

}; // namespace time
}; // namespace ecl

#endif /* ECL_HAS_EXCEPTIONS */
#endif /* ECL_IS_POSIX */
