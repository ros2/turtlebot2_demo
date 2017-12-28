/**
 * @file /include/ecl/time/duration.hpp
 *
 * @brief Cross-platform header inclusions for the duration typedefs.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_DURATION_HPP_
#define ECL_TIME_DURATION_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#if defined(ECL_IS_POSIX)
    #include "timestamp_pos.hpp"
#elif defined(ECL_IS_WIN32)
    #include "timestamp_win.hpp"
#endif

namespace ecl {

	/**
	 * @brief Convenience typedef to associate timestamps with the concept of durations.
	 *
	 * Both timestamps and durations utilise similar functionality under the hood,
	 * even though they are conceptually different. The only thing I might consider
	 * if creating a separate class for durations is to introduce negativity.
	 * However, I've yet to find a real use for it.
	 */
	typedef TimeStamp Duration;

} // namespace ecl

#endif /* ECL_TIME_DURATION_HPP */
