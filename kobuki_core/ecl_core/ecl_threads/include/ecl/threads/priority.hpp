/**
 * @file /include/ecl/threads/priority.hpp
 *
 * @brief Priority scheduling for threads.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_PRIORITY_HPP_
#define ECL_THREADS_PRIORITY_HPP_

/*****************************************************************************
** Cross Platform Implementation
*****************************************************************************/

#include <ecl/config/ecl.hpp> // ECL_ macros

#include "priority_common.hpp"
#if defined(ECL_IS_POSIX)
  #include "priority_pos.hpp"
#elif defined(ECL_HAS_WIN32_THREADS)
  #include "priority_win.hpp"
#endif


#endif /*ECL_THREADS_PRIORITY_HPP_*/
