/**
 * @file /include/ecl/time.hpp
 *
 * @brief Various classes for timing activities.
 *
 * @date May, 2009.
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_HPP_
#define ECL_TIME_HPP_


/*****************************************************************************
** Includes
*****************************************************************************/
#include <time.h>


#ifdef emit
    #undef emit
    #define replace_qt_emit
#endif

#include "time/duration.hpp"
#include <ecl/time/frequency.hpp>
#include "time/random_number_generator.hpp"
#include "time/sleep.hpp" // Posix rt-timers only
#include "time/snooze.hpp" // Posix rt-timers only
#include "time/stopwatch.hpp" // Posix rt-timers only
#include "time/time_data.hpp"
#include "time/timestamp.hpp"

#ifdef replace_qt_emit
    #define emit
#endif


#endif /*ECL_TIME_HPP_*/
