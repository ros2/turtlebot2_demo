/**
 * @file /include/ecl/time_lite/functions_pos.hpp
 *
 * @brief Simple posix implementation of ecl time functions.
 *
 * This implementation has its problems - most notably that you cannot get
 * monotonic functionality.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_FUNCTIONS_POS_HPP_
#define ECL_TIME_LITE_FUNCTIONS_POS_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>
#include "macros.hpp"

#if defined(ECL_HAS_POSIX_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime> // time structs, nanosleep
#include <sys/time.h> // gettimeofday
#include <ecl/config/macros.hpp>
#include "errors.hpp"
#include "types_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementations
*****************************************************************************/
/**
 * @brief Posix fallback function for gettime when better functions unavailable.
 *
 * This is a simple implementation of the gettime and is used within the ecl
 * as a fallback for posix systems which don't have better timers. The main
 * drawback with this one is it is not monotonic and can suffer from wrap
 * around problems.
 *
 * Valid return values:
 *
 * - NoError,
 * - MemoryError,
 * - ArgNotSupported,
 * - PermissionsError.
 *
 * @param time : time structure.
 * @return TimeError : error result.
 */
ecl_time_lite_PUBLIC  TimeError epoch_time(TimeStructure &time);

/**
 * @brief Fallback approximation to an absolute timed sleep.
 *
 * If librt is unavailable we have to use this approximation to its
 * absolutely timed nanosleep function. If used in a control loop,
 * there will be some drift.
 *
 * Valid return values:
 *
 * - NoError,
 * - MemoryError,
 * - OutOfRangeError,
 * - InterruptedError.
 *
 * @param time : the time to sleep until.
 * @return TimeError : error result.
 */
ecl_time_lite_PUBLIC  TimeError sleep_until(const TimeStructure &time);

/**
 * @brief A regular sleep function that operates relative to 'now'.
 *
 * This has high resolution, but its accuracy will depend on your
 * scheduler.
 *
 * Valid return values:
 *
 * - NoError,
 * - MemoryError,
 * - OutOfRangeError,
 * - InterruptedError.
 *
 * @param time : the period to sleep for.
 * @return TimeError : error result.
 */
ecl_time_lite_PUBLIC  TimeError sleep(const TimeStructure &time);

} // namespace ecl

#endif
#endif /* ECL_TIME_LITE_FUNCTIONS_POS_HPP_ */
