/**
 * @file /include/ecl/time_lite/types_pos.hpp
 *
 * @brief Posix definition for time types.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_TYPES_POS_HPP_
#define ECL_TIME_LITE_TYPES_POS_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>

#if defined(ECL_HAS_MACH_TIMERS) || defined(ECL_HAS_POSIX_TIMERS) || defined(ECL_HAS_RT_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ctime>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Standard cross platform time structure.
 *
 * This is already supplied by posix systems as timespec.
 */
typedef timespec TimeStructure;

} // namespace ecl

#endif
#endif /* ECL_TIME_LITE_TYPES_POS_HPP_ */
