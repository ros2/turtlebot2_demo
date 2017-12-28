/**
 * @file /include/ecl/time_lite/functions_rt.hpp
 *
 * @brief Real-time implementation of ecl time functions.
 *
 * Real-time (librt) implementation of ecl time functions.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_FUNCTIONS_RT_HPP_
#define ECL_TIME_LITE_FUNCTIONS_RT_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>
#include "macros.hpp"

#if defined(ECL_HAS_RT_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime>
#include <ecl/config/macros.hpp>
#include "errors.hpp"
#include "types.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
/**
 * @brief Monotonic gettime function provided by librt.
 *
 * This is the best case scenario - it's high resolution and its monotonic.
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
 * @brief Realtime gettime function provided by librt.
 *
 * For those rare cases when you absolutely have to fallback to realtime
 * instead of using the monotone clock.
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
ecl_time_lite_PUBLIC  TimeError realtime_epoch_time(TimeStructure &time);

/**
 * @brief Monotonic absolute timed sleep function provided by librt.
 *
 * This is the best implementation of a sleeper which does not
 * cause drift in a periodic control loop. It tunes in on absolute
 * times rather than relative times.
 *
 * Valid return values:
 *
 * - NoError,
 * - MemoryError,
 * - OutOfRangeError,
 * - InterruptedError.
 *
 * @param time : the absolute time to sleep until.
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
#endif /* ECL_TIME_LITE_FUNCTIONS_RT_HPP_ */
