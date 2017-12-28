/**
 * @file /ecl_time/src/lib/timestamp_base.cpp
 *
 * @brief Implementation of the common cross-functionality in timestamp.
 *
 * @date May 24, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/time/timestamp_base.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

TimeStampBase::TimeStampBase (const double& decimal_time_value) ecl_assert_throw_decl(StandardException)
{
    time.tv_sec = static_cast<long>(decimal_time_value);
    time.tv_nsec = (decimal_time_value - static_cast<long>(decimal_time_value))*1000000000L;
}

TimeStampBase::TimeStampBase (const time_t& seconds, const long& nanoseconds) ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( (seconds > 0) && (nanoseconds >= 0) ) ||
                      ( (seconds < 0) && (nanoseconds <= 0) ) ||
                      ( (seconds == 0) ),
                      StandardException(LOC,InvalidInputError,"Positive timestamps require positive nanoseconds, negative timestamps require negative nanoseconds.") );
    time.tv_sec = seconds;
    time.tv_nsec = nanoseconds;
}

/*****************************************************************************
** Implementation [Stamps]
*****************************************************************************/

const TimeStampBase& TimeStampBase::stamp (const double& decimal_time_value) ecl_assert_throw_decl(StandardException) {
    time.tv_sec = static_cast<time_t>(decimal_time_value);
    time.tv_nsec = (decimal_time_value - static_cast<long>(decimal_time_value))*1000000000L;
    return (*this);
}

const TimeStampBase& TimeStampBase::stamp (const time_t& seconds, const long& nanoseconds) ecl_assert_throw_decl(StandardException) {
  ecl_assert_throw( ( (seconds > 0) && (nanoseconds >= 0) ) ||
                    ( (seconds < 0) && (nanoseconds <= 0) ) ||
                    ( (seconds == 0) ),
                    StandardException(LOC,InvalidInputError,"Positive timestamps require positive nanoseconds, negative timestamps require negative nanoseconds.") );
    time.tv_sec = seconds;
    time.tv_nsec = nanoseconds;
    return (*this);
}

/*****************************************************************************
** Implementation [Comparison Operators]
*****************************************************************************/

bool TimeStampBase::operator==(const TimeStampBase& time_stamp ) {
    if( ( time_stamp.time.tv_sec == time.tv_sec ) && ( time_stamp.time.tv_nsec == time.tv_nsec ) ) {
        return true;
    } else {
        return false;
    }
}

bool TimeStampBase::operator!=(const TimeStampBase& time_stamp ) {
    if( ( time_stamp.time.tv_sec == time.tv_sec ) && ( time_stamp.time.tv_nsec == time.tv_nsec ) ) {
        return false;
    } else {
        return true;
    }
}

bool TimeStampBase::operator<=(const TimeStampBase& time_stamp ) {
    if( time.tv_sec > time_stamp.time.tv_sec ) {
        return false;
    } else if ( time.tv_sec == time_stamp.time.tv_sec ) {
        if ( time.tv_nsec > time_stamp.time.tv_nsec ) {
            return false;
        }
    }
    return true;
}

bool TimeStampBase::operator>=(const TimeStampBase& time_stamp ) {
    if( time.tv_sec < time_stamp.time.tv_sec ) {
        return false;
    } else if ( time.tv_sec == time_stamp.time.tv_sec ) {
        if ( time.tv_nsec < time_stamp.time.tv_nsec ) {
            return false;
        }
    }
    return true;
}

bool TimeStampBase::operator<(const TimeStampBase& time_stamp ) {
    if( time.tv_sec > time_stamp.time.tv_sec ) {
        return false;
    } else if ( time.tv_sec == time_stamp.time.tv_sec ) {
        if ( time.tv_nsec >= time_stamp.time.tv_nsec ) {
            return false;
        }
    }
    return true;
}

bool TimeStampBase::operator>(const TimeStampBase& time_stamp ) {
    if( time.tv_sec < time_stamp.time.tv_sec ) {
        return false;
    } else if ( time.tv_sec == time_stamp.time.tv_sec ) {
        if ( time.tv_nsec <= time_stamp.time.tv_nsec ) {
            return false;
        }
    }
    return true;
}
/*****************************************************************************
** Implementation [TimeStamp][Mathematical Operators]
*****************************************************************************/

void TimeStampBase::operator+=(const TimeStampBase& time_stamp ) {
    time.tv_sec += time_stamp.time.tv_sec;
    int64 nsec = time.tv_nsec + time_stamp.time.tv_nsec;
    if ( nsec >= 1000000000L ) {
      time.tv_sec += 1;
      nsec -= 1000000000L;
    } else if ( nsec <= -1000000000L ) {
      time.tv_sec -= 1;
      nsec += 1000000000L;
    }
    if ( ( time.tv_sec > 0 ) && ( nsec < 0 ) ) {
      time.tv_sec -= 1;
      nsec = 1000000000L + nsec;
    }
    if ( ( time.tv_sec < 0 ) && ( nsec > 0 ) ) {
      time.tv_sec += 1;
      nsec = nsec - 1000000000L;
    }
    time.tv_nsec = nsec;
}

TimeStampBase TimeStampBase::operator+(const TimeStampBase& time_stamp ) {
    long sec = time.tv_sec + time_stamp.time.tv_sec;
    int64 nsec = time.tv_nsec + time_stamp.time.tv_nsec;
    if ( nsec >= 1000000000L ) {
      sec += 1;
      nsec -= 1000000000L;
    } else if ( nsec <= -1000000000L ) {
      sec -= 1;
      nsec += 1000000000L;
    }
    if ( ( sec > 0 ) && ( nsec < 0 ) ) {
      sec -= 1;
      nsec = 1000000000L + nsec;
    }
    if ( ( sec < 0 ) && ( nsec > 0 ) ) {
      sec += 1;
      nsec = nsec - 1000000000L;
    }
    return TimeStampBase(sec,nsec);
}

void TimeStampBase::operator-=(const TimeStampBase& time_stamp ) ecl_assert_throw_decl(StandardException) {
    time_t sec = time.tv_sec - time_stamp.time.tv_sec;
    long nsec = time.tv_nsec - time_stamp.time.tv_nsec;

    if ( (sec > 0) && ( nsec < 0 )) {
        sec -= 1;
        nsec += 1000000000L;
    } else if ( (sec < 0) && ( nsec > 0 )) {
      sec += 1;
      nsec -= 1000000000L;
    }
    time.tv_sec = sec;
    time.tv_nsec = nsec;
}

TimeStampBase TimeStampBase::operator-(const TimeStampBase& time_stamp ) ecl_assert_throw_decl(StandardException) {
    time_t sec = time.tv_sec - time_stamp.time.tv_sec;
    long nsec = time.tv_nsec - time_stamp.time.tv_nsec;

    if ( (sec > 0) && ( nsec < 0 )) {
        sec -= 1;
        nsec += 1000000000L;
    } else if ( (sec < 0) && ( nsec > 0 )) {
      sec += 1;
      nsec -= 1000000000L;
    }
    return TimeStampBase(sec,nsec);
}



} // namespace ecl

