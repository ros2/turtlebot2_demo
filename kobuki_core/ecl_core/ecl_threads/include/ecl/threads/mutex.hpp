/**
 * @file /include/ecl/threads/mutex.hpp
 *
 * @brief Mutex interface.
 *
 * Cross-platform mutex for locking/unlocking shared memory in a
 * multi-threaded application. In linux it uses pthreads, in windows it
 * uses critical sections.
 *
 * @date June 2009
 **/
#ifndef ECL_ECL_THREADS_MUTEX_HPP_
#define ECL_ECL_THREADS_MUTEX_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*************************************************************************
 * Includes
 ************************************************************************/

#if defined(ECL_HAS_POSIX_THREADS)
  #include "mutex_pos.hpp"
#elif defined(ECL_HAS_WIN32_THREADS)
  #include "mutex_w32.hpp"
#endif


#endif /* ECL_ECL_THREADS_MUTEX_HPP_*/
