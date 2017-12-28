/**
 * @file /include/ecl/time/sleep_win.hpp
 *
 * @brief Interface for the sleep classes utilising posix timers.
 *
 * @date April 2013
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_SLEEP_WIN_HPP_
#define ECL_TIME_SLEEP_WIN_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_WIN32)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <time.h>
#include <errno.h>
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time_lite.hpp>
#include "duration.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [Sleep]
*****************************************************************************/
/**
 * @brief General purpose sleeper.
 *
 * General purpose sleeper. This works with the Duration structure which
 * allows time setting of any resolution. Alternatively, a simple
 * 'seconds' resolution operator is available for convenience.
 *
 * <b>Error Handling</b>
 *
 * Will throw exceptions in debug mode if the timer doesn't initialise
 * properly (unlikely) or you exceed the resolution of a time unit (more likely).
 * In release mode they do not come with any error handling. These functions
 * need to remain fast, like containers, and since they're not usually
 * destructive, such bells and whistles are omitted.
 *
 * @sa Duration, MilliSleep, MicroSleep, NanoSleep.
 */
class ecl_time_PUBLIC Sleep {
public:
	/**
	 * @brief Preconfigures the sleep functor with a specified duration.
	 *
	 * Preconfigures the sleep functor with a specified duration. Note that the
	 * preconfigured value may be changed later via operator()(const Duration &duration)
	 * or operator()(const unsigned long &seconds).
	 *
	 * @param duration : preconfigured duration.
	 */
	Sleep(const Duration &duration);
	/**
	 * @brief Preconfigures the sleep functor with a specified length (in secs).
	 *
	 * Preconfigures the sleep functor with a specified length (secs). Note that the
	 * preconfigured value may be changed later via operator()(const Duration &duration)
	 * or operator()(const unsigned long &seconds).
	 *
	 * @param seconds : preconfigured sleep time (secs).
	 */
	Sleep(const unsigned long &seconds = 0);
	virtual ~Sleep() {}

	/**
	 * @brief Executes the sleeper for the currently configured sleep time.
	 *
	 * Uses the last configured sleep time for the sleeper and executes.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Returns the currently configured sleep time for this sleeper.
	 *
	 * Returns the currently configured sleep time for this sleeper.
	 *
	 * @return Duration : the current sleep time.
	 */
	Duration duration() { return Duration(required.tv_sec,required.tv_nsec); }

	/**
	 * @brief Sleeper functor method for seconds resolution sleeps.
	 *
	 * Sleeper functor method for seconds resolution sleeps.
	 *
	 * @param seconds : number of seconds to sleep for.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()(const unsigned long &seconds) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Sleeper functor method for sleeping at any resolution.
	 *
	 * Sleeper functor method for sleeping at any resolution.
	 *
	 * @param duration : the sleep duration.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()(const Duration &duration) ecl_assert_throw_decl(StandardException);

private:
    TimeStructure required, remaining;
};

/**
 * @brief Milliseconds resolution sleeper.
 *
 * This sleeper works on the milliseconds range, [0-1000].
 *
 * <b>Error Handling</b>
 *
 * Will throw exceptions in debug mode if the timer doesn't initialise
 * properly (unlikely) or you exceed the resolution of a time unit (more likely).
 * In release mode they do not come with any error handling. These functions
 * need to remain fast, like containers, and since they're not usually
 * destructive, such bells and whistles are omitted.
 *
 * @sa Duration, Sleep, MicroSleep, NanoSleep.
 */
class ecl_time_PUBLIC MilliSleep {
public:
	/**
	 * @brief Preconfigures the sleep functor with a specified length (msecs).
	 *
	 * Preconfigures the sleep functor with a specified length (msecs). Note that the
	 * preconfigured value may be changed later via
	 * operator()(const unsigned long &milliseconds).
	 *
	 * @param milliseconds : preconfigured sleep time (msecs).
	 */
	MilliSleep(const unsigned long &milliseconds = 0);

	virtual ~MilliSleep() {}

	/**
	 * @brief Executes the sleeper for the currently configured sleep time.
	 *
	 * Uses the last configured sleep time for the sleeper and executes.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Returns the currently configured sleep time for this sleeper.
	 *
	 * Returns the currently configured sleep time for this sleeper.
	 *
	 * @return Duration : the current sleep time.
	 */
	Duration duration() { return Duration(required.tv_sec,required.tv_nsec); }

	/**
	 * @brief Sleeper functor method for milliseconds resolution sleeps.
	 *
	 * Sleeper functor method for milliseconds resolution sleeps.
	 *
	 * @param milliseconds : number of milli_seconds to sleep for [0-1000].
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()(const unsigned long &milliseconds) ecl_assert_throw_decl(StandardException);
private:
    TimeStructure required, remaining;
};

/**
 * @brief Microseconds resolution sleeper.
 *
 * This sleeper works on the microseconds range, [0-1000000].
 *
 * <b>Error Handling</b>
 *
 * Will throw exceptions in debug mode if the timer doesn't initialise
 * properly (unlikely) or you exceed the resolution of a time unit (more likely).
 * In release mode they do not come with any error handling. These functions
 * need to remain fast, like containers, and since they're not usually
 * destructive, such bells and whistles are omitted.
 *
 * @sa Duration, Sleep, MilliSleep, NanoSleep.
 */
class ecl_time_PUBLIC MicroSleep {
public:
	/**
	 * @brief Preconfigures the sleep functor with a specified length (usecs).
	 *
	 * Preconfigures the sleep functor with a specified length (usecs).
	 * Note that the preconfigured value may be changed later via
	 * operator()(const unsigned long &microseconds).
	 *
	 * @param microseconds : preconfigured sleep time (usecs).
	 */
	MicroSleep(const unsigned long &microseconds = 0);

	virtual ~MicroSleep() {}


	/**
	 * @brief Executes the sleeper for the currently configured sleep time.
	 *
	 * Uses the last configured sleep time for the sleeper and executes.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Returns the currently configured sleep time for this sleeper.
	 *
	 * Returns the currently configured sleep time for this sleeper.
	 *
	 * @return Duration : the current sleep time.
	 */
	Duration duration() { return Duration(required.tv_sec,required.tv_nsec); }
	/**
	 * @brief Sleeper functor method for microseconds resolution sleeps.
	 *
	 * Sleeper functor method for microseconds resolution sleeps.
	 *
	 * @param micro_seconds : number of micro_seconds to sleep for [0-1000000].
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()(const unsigned long &micro_seconds) ecl_assert_throw_decl(StandardException);
private:
    TimeStructure required, remaining;
};

/**
 * @brief Nanoseconds resolution sleeper.
 *
 * This sleeper works on the nanoseconds range, [0-1000000000].
 *
 * <b>Error Handling</b>
 *
 * Will throw exceptions in debug mode if the timer doesn't initialise
 * properly (unlikely) or you exceed the resolution of a time unit (more likely).
 * In release mode they do not come with any error handling. These functions
 * need to remain fast, like containers, and since they're not usually
 * destructive, such bells and whistles are omitted.
 *
 * @sa Duration, Sleep, MilliSleep, MicroSleep.
 */
class ecl_time_PUBLIC NanoSleep {
public:
	/**
	 * @brief Preconfigures the sleep functor with a specified length (nsecs).
	 *
	 * Preconfigures the sleep functor with a specified length (nsecs).
	 * Note that the preconfigured value may be changed later via
	 * operator()(const unsigned long &nanoseconds).
	 *
	 * @param nanoseconds : preconfigured sleep time (nsecs).
	 */
	NanoSleep(const unsigned long &nanoseconds = 0);

	virtual ~NanoSleep() {}


	/**
	 * @brief Executes the sleeper for the currently configured sleep time.
	 *
	 * Uses the last configured sleep time for the sleeper and executes.
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Returns the currently configured sleep time for this sleeper.
	 *
	 * Returns the currently configured sleep time for this sleeper.
	 *
	 * @return Duration : the current sleep time.
	 */
	Duration duration() { return Duration(required.tv_sec,required.tv_nsec); }
	/**
	 * @brief Sleeper functor method for nanoseconds resolution sleeps.
	 *
	 * Sleeper functor method for nanoseconds resolution sleeps.
	 *
	 * @param nanoseconds : number of nano_seconds to sleep for [0-1000000000].
	 *
	 * @exception StandardException : throws if the sleep function fails [debug mode only].
	 */
	void operator()(const unsigned long &nanoseconds) ecl_assert_throw_decl(StandardException);
private:
    TimeStructure required, remaining;
};


} // namespace ecl

/*****************************************************************************
** Interface [Exceptions][Sleeper Classes]
*****************************************************************************/

#if defined(ECL_HAS_EXCEPTIONS)
namespace ecl {
namespace time {


/*****************************************************************************
** Interface [Sleep Exceptions]
*****************************************************************************/
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>usleep and nanosleep</i> posix
 * functions used in the sleeper classes.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 */
inline ecl::StandardException throwSleepException(const char* loc ) {
	int error_result = errno;
	switch (error_result) {
		case ( EINTR  ) : return StandardException(loc, ecl::InterruptedError, "Interrupted the sleep.");
		case ( EINVAL ) : return StandardException(loc, ecl::InvalidInputError, "Specified value was negative or exceeded resolution range.\n\n            Sleep: [N/A]\n            MilliSleep: [0-1000]\n            MicroSleep: [0-1x10^6]\n            NanoSleep: [0-1x10^9]\n");
		case ( EFAULT ) : return StandardException(loc, ecl::MemoryError, "Memory error.");
		default         :
		{
			std::ostringstream ostream;
			ostream << "Unknown error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}

}


}; // namespace time
} // namespace ecl
#endif

#endif /* ECL_IS_WIN32 */
#endif /* ECL_TIME_SLEEP_WIN_HPP_ */
