/**
 * @file /include/ecl/time_lite/functions_win.hpp
 *
 * @brief Windows interfaces for ecl time functions.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_FUNCTIONS_WIN_HPP_
#define ECL_TIME_LITE_FUNCTIONS_WIN_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>
#include "macros.hpp"

#if defined(ECL_HAS_WIN_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime>
#include <ecl/config/macros.hpp>
#include "types_win.hpp"
#include "errors.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Functions
*****************************************************************************/

/**
 * @brief Gets the time in seconds/nanoseconds from some landmark point.
 *
 * This is not actually epoch time, but its good enough. It pulls the
 * amount of time since the computer has actually been turned on. To get the epoch
 * time you'll need a bit more machinery - you can find an example in ros's
 * rostime/src/time.cpp.
 *
 * Valid return values:
 *
 * - NoError,
 * - UnknownError.
 *
 * @param time : time structure.
 * @return TimeError :  error result.
 */
ecl_time_lite_PUBLIC  TimeError epoch_time(TimeStructure &time);

/**
 * @brief Fallback to an approximation of an absolute sleep function.
 *
 * This imitates rt's absolute sleeper which will sleep exactly
 * until the specified time. This one instead will just generate
 * some time differences which will, if used in a control loop,
 * generate some drift.
 *
 * Valid return values:
 *
 * - NoError,
 * - UnknownError.
 *
 * @param time : the absolute time to sleep until.
 * @return TimeError :  error result.
 */
ecl_time_lite_PUBLIC  TimeError sleep_until(const TimeStructure &time);

/**
 * @brief A regular sleep function that operates relative to 'now'.
 *
 * This is the windows implementation and it doesn't have a great
 * deal of resolution.
 *
 * Valid return values:
 *
 * - NoError,
 * - UnknownError.
 *
 * @param time : the period to sleep for.
 * @return TimeError :  error result.
 */
ecl_time_lite_PUBLIC  TimeError sleep(const TimeStructure &time);

} // namespace ecl

#endif
#endif /* ECL_TIME_LITE_FUNCTIONS_WIN_HPP_ */
