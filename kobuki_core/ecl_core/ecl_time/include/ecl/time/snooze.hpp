/**
 * @file /include/ecl/time/snooze.hpp
 *
 * @brief Implements periodic loop timing via absolute time lookups.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_SNOOZE_HPP_
#define ECL_TIME_SNOOZE_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#if defined(ECL_IS_POSIX)
  #include "snooze_pos.hpp"
#else
  #include "snooze_win.hpp"
#endif

#endif /* ECL_TIME_SNOOZE_HPP_ */
