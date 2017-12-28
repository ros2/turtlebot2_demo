/**
 * @file /include/ecl/time/sleep.hpp
 *
 * @brief Cross-platform header inclusions for the sleep class.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_SLEEP_HPP_
#define ECL_TIME_SLEEP_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#if defined(ECL_IS_POSIX)
  #include "sleep_pos.hpp"
#elif defined(ECL_IS_WIN32)
  #include "sleep_win.hpp"
#endif

#endif /* ECL_TIME_SLEEP_HPP_ */
