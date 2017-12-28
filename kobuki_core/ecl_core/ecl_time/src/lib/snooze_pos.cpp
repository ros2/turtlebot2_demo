/**
 * @file /src/lib/snooze_pos.cpp
 *
 * @brief Implementation of periodic timers.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/time_lite/functions.hpp>
#include "../../include/ecl/time/snooze_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [Snooze]
*****************************************************************************/

Snooze::Snooze() {
	period(Duration(0,10000000));
}

Snooze::Snooze(const Duration &time, const bool& validate) { period(time, validate); }

void Snooze::period(const Duration &time, const bool& validate) {
	validate_times = validate;
	time_period.tv_sec = time.sec();
	time_period.tv_nsec = time.nsec();
    wrap_value_ns = 1000000000L-time_period.tv_nsec;
    initialise();
}

void Snooze::initialise() {
	epoch_time(time_value);
}

void Snooze::operator()() {

    add_period();
    validate();
    sleep_until(time_value);
}

void Snooze::add_period() {

	// Nanoseconds
    if ( time_value.tv_nsec > wrap_value_ns ) { // wrap
        time_value.tv_nsec = time_value.tv_nsec - wrap_value_ns;
        time_value.tv_sec += 1;
    } else {
        time_value.tv_nsec += time_period.tv_nsec;
    }
    // Seconds
    time_value.tv_sec += time_period.tv_sec;

    if ( validate_times ) {
    	validate();
    }
}

void Snooze::validate() {

	TimeStructure time_current;
    epoch_time(time_current);

    if ( time_current.tv_sec > time_value.tv_sec ) {
    	// Revalidate
        time_value.tv_sec = time_current.tv_sec;
        time_value.tv_nsec = time_current.tv_nsec;
        add_period();
    }  else if ( time_current.tv_sec == time_value.tv_sec ) {
        if ( time_current.tv_nsec > time_value.tv_nsec ) {
        	// Revalidate
            time_value.tv_sec = time_current.tv_sec;
            time_value.tv_nsec = time_current.tv_nsec;
            add_period();
        }
    }
}

} // namespace ecl

#endif /* ECL_IS_POSIX */
