/**
 * @file /include/ecl/time_lite/date.hpp
 *
 * @brief Date manipulations.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_DATE_HPP_
#define ECL_TIME_LITE_DATE_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>

#if defined(ECL_HAS_POSIX_TIMERS) || defined (ECL_HAS_RT_TIMERS) || defined(ECL_HAS_MACH_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementations
*****************************************************************************/

/**
 * @brief Get the date/time as a string.
 *
 * Simple function to generate a usable string for uniquely creating
 * things like filenames.
 */
ecl_time_lite_PUBLIC std::string get_date_string();

} // namespace ecl

#endif  // POSIX or RT TIMERS

#endif // ECL_TIME_LITE_DATE_HPP_
