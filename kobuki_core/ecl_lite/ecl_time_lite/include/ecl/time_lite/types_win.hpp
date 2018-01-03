/**
 * @file /include/ecl/time_lite/types_win.hpp
 *
 * @brief Cross platform definition for time types.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_TYPES_WIN_HPP_
#define ECL_TIME_LITE_TYPES_WIN_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>
#include "macros.hpp"

#if defined(ECL_HAS_WIN_TIMERS)

/*****************************************************************************
** Include
*****************************************************************************/

#include <ctime>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Structures
*****************************************************************************/

/**
 * @brief Standard cross platform time structure.
 *
 * Simply replicates the posix timespec structure.
 */
struct ecl_time_lite_PUBLIC TimeStructure {
	time_t tv_sec;
	long tv_nsec;
};

} // namespace ecl
#endif
#endif /* ECL_TIME_LITE_TYPES_WIN_HPP_ */
