/**
 * @file /ecl_time/include/ecl/time/cpuwatch_rt.hpp
 *
 * @brief This class measures cpu time for a process.
 *
 * Currently it is only valid for systems with rt timers, i.e. clock_gettime.
 * This is constrained by the selection made in cpuwatch.hpp.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_CPUWATCH_POS_HPP_
#define ECL_TIME_CPUWATCH_POS_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>

#if defined(ECL_IS_POSIX)
  // monotonic clock, cpu clock -> clock_gettime; clock_selection -> clock_nanosleep
  #if defined(_POSIX_MONOTONIC_CLOCK) && (_POSIX_MONOTONIC_CLOCK) >= 0L && defined(_POSIX_CLOCK_SELECTION) && (_POSIX_CLOCK_SELECTION) >= 0L

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include "timestamp.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Classes
*****************************************************************************/
/**
 * @brief A stopwatch that runs on a process's cpu time.
 *
 * This measures the time spent by the process executing on the cpu itself.
 * A good stopwatch for benchmarking tests.
 *
 * Functionally, it uses the TimeStamp class as a means of recording time,
 * splits and elapsed time. Its operation should be intuitive.
 *
 * This is only valid on platforms with a supporting timestamp
 * implementation and rt timers (needs clock_gettime with
 * CLOCK_PROCESS_CPUTIME_ID.
 *
 * Note that the cpuwatch starts automatically, just use restart() if you
 * wish to reset and start again.
 *
 * <b>Usage:</b>
 * @code
 * CpuWatch cpuwatch;
 * TimeStamp time;
 * time = cpuwatch.split()
 * cout << time << endl;
 * time = cpuwatch.elapsed()
 * cout << time << endl;
 * cpuwatch.restart();
 * @endcode
 **/
class ecl_time_PUBLIC CpuWatch
{
    public:
        /**
         * Default constructor that initialises the cpuwatch with the current
         * system time to use as a point of reference.
         **/
        CpuWatch();

        virtual ~CpuWatch() {}
        /**
         * @brief Restarts the cpuwatch.
         *
         * Restarts the cpuwatch (i.e. re-initialises the cpuwatch with the
         * current system time as its point of reference).
         **/
        void restart();

        /**
         * @brief Calculates the total elapsed time.
         *
         * Calculates the total elapsed time since the cpuwatch was started.
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
        timespec tmp; // use for temporary cacluations.
};

}; // namespace ecl

#endif /* MANY POSIX TIME REQ'MENTS */
#endif /* ECL_IS_POSIX */
#endif /* ECL_TIME_CPUWATCH_POS_HPP_ */
