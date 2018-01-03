/**
 * @file /include/ecl/time_lite/functions_mac.hpp
 *
 * @brief Mach implementation of ecl time functions.
 *
 * This is to make up for the fact that macosx does not have librt.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_FUNCTIONS_MAC_HPP_
#define ECL_TIME_LITE_FUNCTIONS_MAC_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>
#include "macros.hpp"

#if defined(ECL_HAS_MACH_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime>
#include <sys/time.h> // gettimeofday
#include <ecl/config/macros.hpp>
#include "errors.hpp"
#include "types.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

/**
 * @brief Mach implementation of gettime on a macosx.
 *
 * This is better than the fallback posix implementation, but still doesn't
 * quite implement monotonic time. Mac has a better timer function, but
 * I've not investigated it yet.
 *
 * Valid return values:
 *
 * - NoError,
 * - UnknownError.
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
 * - UnknownError.
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
 * - UnknownError.
 *
 * @param time : the period to sleep for.
 * @return TimeError : error result.
 */
ecl_time_lite_PUBLIC  TimeError sleep(const TimeStructure &time);

} // namespace ecl

#endif
#endif /* ECL_TIME_LITE_FUNCTIONS_MAC_HPP_ */
