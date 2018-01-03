/**
 * @file /ecl_time_lite/include/ecl/time_lite/cpu_time.hpp
 *
 * @brief Timer for measuring time spent sitting on the cpu.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_CPU_TIME_HPP_
#define ECL_TIME_LITE_CPU_TIME_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "errors.hpp"
#include "types.hpp"
#include <ecl/time_lite/config.hpp>
#include <ecl/config/macros.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

#ifdef ECL_HAS_CPUTIME
/**
 * @brief Cpu time function provided by librt.
 *
 * This provides the time spent by the process executing on the cpu.
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
ecl_time_lite_PUBLIC  TimeError cpu_time(TimeStructure &time);
#endif

} // namespace ecl

#endif /* ECL_TIME_LITE_CPU_TIME_HPP_ */
