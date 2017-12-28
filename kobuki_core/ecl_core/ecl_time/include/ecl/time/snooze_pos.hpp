/**
 * @file /include/ecl/time/snooze_pos.hpp
 *
 * @brief Periodic loop timers via the rt library.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_SNOOZE_POS_HPP_
#define ECL_TIME_SNOOZE_POS_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include "duration.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/time_lite/types.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Classes [Snooze]
*****************************************************************************/

/**
 * @brief  Implements periodic loop timing without the use of system timers.
 *
 * Intelligent use of the system's absolute time to do periodic timing
 * without using up the system's timer/signal resources. It also manages
 * to escape the effects of the following construct:
 * \arg start loop
 * \arg do work
 * \arg get time
 * \arg calculate remaining time
 * \arg sleep remaining time
 * \arg end loop
 * The problem with this is a small amount of time drift creeps in. Additional
 * latency may also occur between getting the time and sleeping due to
 * scheduling of other processes. This class avoids this problem by sleeping
 * until the next 'absolute time' which means the latency is only
 * present on the very first loop. After that, only the effects of
 * latency 'noise' will affect the looping.
 *
 * The only thing to be careful with this class is that the periodic
 * timestamp does not get behind the current time.
 * (this can happen because it was not initialised, or there was a burst of
 * lag that causes the snooze class to not sleep at all for
 * several loops until the timestamp catches up with the current time).
 *
 * By default the class is configured to validate the periodic timestamp
 * every time the operator() is called. This will resync it with the
 * current time if it gets behind.
 * This however brings in a little latency as the snooze class has
 * to make a call to check the current time. If this is an issue,
 * you can set the validate variable to false in either the constructor
 * or period() method.
 * This will remove the validation, but ONLY do this if you are certain
 * your loops will always be consistent (e.g. when scheduling the
 * process/thread with SCHED_RR or SCHED_FIFO).
 *
 * Note that this is only really suitable for loops down to 1ms (best
 * around 5-10ms or upwards). If you wish finer granularity, then you
 * really need to consider a RTOS.
 *
 * <b>Usage:</b>
 *
 * @code
 * Snooze snooze(Duration(0,20000000); // 20ms snooze
 * // Some preliminaries
 * snooze.initialise(); // make sure the snoozer is sync'ed with the current time.
 * while (1) {
 *   // do some work
 *   snooze();
 * }
 * @endcode
 **/
class ecl_time_PUBLIC Snooze {
public:
	/*********************
	** C&D's
	**********************/
	/**
	 * @brief Initialises the snoozer with a default period of 10ms.
	 *
	 * Default initialisation for the snooze class - 10ms periods and no validation.
	 * Use the period() method to reconfigure if desired.
	 */
	Snooze();
	/**
	 * @brief Initialises the snoozer RAII style with the specified period.
	 *
	 * Initialises the snoozer RAII style with the specified period. It also
	 * optionally allows the user to specify whether they want to validate
	 * each snooze timestamp so that it doesn't get behind the current time.
	 * Doing so increases the work the class needs to do though. By default
	 * the class does not validate so as to keep latency to a minimum,
	 * but use it like this with care.
	 *
	 * @param time : the period of the loop that will be used.
	 * @param validate : flag that enables validation of the latency at each loop.
	 **/
	Snooze (const Duration &time, const bool& validate = false );
	virtual ~Snooze() {}

	/*********************
	** Usage Methods
	**********************/
	/**
	 * @brief Reconfigures the snoozer with the specified period.
	 *
	 * Reinitialises the snoozer with the specified period. It also
	 * optionally allows the user to specify whether they want to validate
	 * each snooze timestamp so that it doesn't get behind the current time.
	 * Doing so increases the work the class needs to do though. By default
	 * the class does not validate so as to keep latency to a minimum,
	 * but use it like this with care.
	 *
	 * @param time : the period of the loop that will be used.
	 * @param validate : flag that enables validation of the latency at each loop.
	 **/
	void period(const Duration &time, const bool& validate = false);
	/**
	 * @brief Returns the currently configured period setting.
	 *
	 * Returns the currently configured period setting.
	 *
	 * @return Duration : the current time period duration.
	 **/
	Duration period() { return Duration(time_period.tv_sec,time_period.tv_nsec); }
	/**
	 * @brief Restarts the snoozer (syncs it with the current time).
	 *
	 * Use this immediately before starting to loop (that is, if you haven't
	 * instantiated the class or called the period() function directly
	 * beforehand).
	 */
	void initialise();
	/**
	 * @brief Puts the thread/process to sleep until the next period is reached.
	 *
	 * Sleeps until the next periodic timestamp has been reached. If the
	 * timestamp has fallen behind the current time, it will react in one
	 * of two ways:
	 *
	 * - If validation checks were requested, it will resync with the current time.
	 * - Otherwise, it will simply return immediately.
	 */
	void operator()();

protected:
	/*********************
	** Internal Methods
	**********************/
	/**
	 * @brief Adds a period to the currently stored 'alarm clock' setting.
	 *
	 * Internally used to add the period to the time structure without causing
	 * problems while wrapping around.
	 */
	void add_period();
	/**
	 * @brief Validates that the snooze timestamp hasn't fallen behind the current time.
	 *
	 * Validate that the snooze timestamp has not fallen behind the current time.
	 * If it has, then update the snooze time value to the current time value and proceed.
	 */
	void validate();

	/*********************
	** Internal Variables
	**********************/
	TimeStructure time_value;
	TimeStructure time_period;
	long wrap_value_ns;
	bool validate_times;

};

} // namespace ecl

#endif /* ECL_IS_POSIX */
#endif /* ECL_TIME_SNOOZE_POS_HPP_ */
