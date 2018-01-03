/**
 * @file /src/lib/stopwatch.cpp
 *
 * @brief Posix rt-timer implementation of the stopwatch class.
 *
 * @date June, 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time/timestamp.hpp"
#include "../../include/ecl/time/stopwatch.hpp"
#ifdef ECL_HAS_TIMESTAMP

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [StopWatch]
*****************************************************************************/
StopWatch::StopWatch() {
    // The variable start_time will automatically initialise with the current
    // system time.
};

/*****************************************************************************
** Implementation
*****************************************************************************/

void StopWatch::restart()
{
    start_time.stamp();
    split_time = start_time;
}

TimeStamp StopWatch::elapsed()
{
    TimeStamp current_time; // Automatically fetches the current system time.
    return ( current_time - start_time );
}

TimeStamp StopWatch::split()
{
    TimeStamp last_time = split_time;
    split_time.stamp();
    return (split_time - last_time);
}

}; // namespace ecl

#endif /* ECL_HAS_TIMESTAMP */

