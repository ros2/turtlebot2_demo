/**
 * @file /include/ecl/threads/thread.hpp
 *
 * @brief Thread interfaces.
 *
 * Cross-platform thread functionality to be used as a composited variable (i.e,
 * not an inheritable interface).
 *
 * @date June 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_THREAD_HPP_
#define ECL_THREADS_THREAD_HPP_

/*************************************************************************
 * Includes
 ************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#if defined(ECL_HAS_POSIX_THREADS)
  #include "thread_pos.hpp"
#elif defined(ECL_HAS_WIN32_THREADS)
  #include "thread_win.hpp"
#endif


#endif /* ECL_THREADS_THREAD_HPP_*/
