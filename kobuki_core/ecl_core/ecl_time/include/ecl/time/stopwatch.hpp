/**
 * @file /include/ecl/time/stopwatch.hpp
 *
 * @brief Cross-platform header inclusions for the stopwatch class.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STOPWATCH_HPP_
#define ECL_STOPWATCH_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include "timestamp.hpp"
#include "macros.hpp"

#ifdef ECL_HAS_TIMESTAMP

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief A timepiece that emulates the functionality of a stopwatch.
 *
 * This uses the TimeStamp class as a means of recording time, splits and
 * elapsed time. Its operation should be intuitive.
 *
 * This is only valid on platforms with a supporting timestamp implementation.
 *
 * Note that the stopwatch starts automatically, just use restart() if you
 * wish to reset and start again.
 *
 * <b>Usage:</b>
 * @code
 * StopWatch stopwatch;
 * TimeStamp time;
 * time = stopwatch.split()
 * cout << time << endl;
 * time = stopwatch.elapsed()
 * cout << time << endl;
 * stopwatch.restart();
 * @endcode
 **/
class ecl_time_PUBLIC StopWatch
{
    public:
        /**
         * Default constructor that initialises the stopwatch with the current
         * system time to use as a point of reference.
         **/
        StopWatch();

        virtual ~StopWatch() {}
        /**
         * @brief Restarts the stopwatch.
         *
         * Restarts the stopwatch (i.e. re-initialises the stopwatch with the
         * current system time as its point of reference).
         **/
        void restart();

        /**
         * @brief Calculates the total elapsed time.
         *
         * Calculates the total elapsed time since the stopwatch was started.
         * @return TimeStamp : the total elapsed time since (re)started.
         **/
        TimeStamp elapsed();
        /**
         * @brief Calculates the current split.
         *
         * Calculates the elapsed time since the last split.
         * @return TimeStamp : the elapsed time since the last split.
         **/
        TimeStamp split();

    private:
        TimeStamp start_time, split_time;
};


}; // namespace ecl

#endif /* ECL_HAS_TIMESTAMP */
#endif /* ECL_STOPWATCH_HPP_ */
