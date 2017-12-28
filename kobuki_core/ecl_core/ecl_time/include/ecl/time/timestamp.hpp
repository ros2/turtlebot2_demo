/**
 * @file /include/ecl/time/timestamp.hpp
 *
 * @brief Cross-platform header inclusions for the timestamp class.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_TIME_STAMP_HPP_
#define ECL_TIME_TIME_STAMP_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

// Use the ecl cross platform time functions.
#include <ecl/time_lite/functions.hpp>

#if defined(ECL_IS_POSIX)
  #include "timestamp_pos.hpp"
#elif defined(ECL_IS_WIN32)
  #include "timestamp_win.hpp"
#endif

#endif /* ECL_TIME_TIME_STAMP_HPP_ */
