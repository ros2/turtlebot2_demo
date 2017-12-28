/**
 * @file /src/lib/mutex_pos.cpp
 *
 * @brief Posix mutex implementation.
 *
 * @date June 2009
 **/
/*****************************************************************************
 ** Platform Check
 *****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <errno.h>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/threads/mutex.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl {

/*****************************************************************************
 * Mutex Class Methods
 *****************************************************************************/

Mutex::Mutex(const bool locked) ecl_assert_throw_decl(StandardException) :
    number_locks(0)
{

  pthread_mutexattr_t attr;
  int result;

  result = pthread_mutexattr_init(&attr);
  ecl_assert_throw(result == 0, threads::throwMutexAttrException(LOC,result));

  #if defined(NDEBUG) || defined(ECL_NDEBUG)
    result = pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_NORMAL);
  #else
    result = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
  #endif
  ecl_assert_throw(result == 0, threads::throwMutexAttrException(LOC,result));

  if (result == 0) {
    result = pthread_mutex_init(&mutex, &attr);
  }
  ecl_assert_throw(result == 0, threads::throwMutexInitException(LOC,result));
  result = pthread_mutexattr_destroy(&attr);
  ecl_assert_throw(result == 0, threads::throwMutexAttrException(LOC,result));

  if (locked) {
    this->lock();
  }
}
;

Mutex::~Mutex()
{
  pthread_mutex_destroy(&mutex);
  // Spank! Destructor exceptions are bad.
  // ecl_assert_throw( result == 0, threads::throwMutexDestroyException(LOC,result));
}

void Mutex::lock() ecl_assert_throw_decl(StandardException)
{
  ++number_locks;
  int result = pthread_mutex_lock(&mutex);
  ecl_assert_throw(result == 0, threads::throwMutexLockException(LOC,result));
}
;

bool Mutex::trylock(Duration &duration) ecl_assert_throw_decl(StandardException)
{
  #if defined(_POSIX_TIMEOUTS) && (_POSIX_TIMEOUTS - 200112L) >= 0L
    timespec timeout;
    timeout.tv_sec = duration.sec();
    timeout.tv_nsec = duration.nsec();
    int result = pthread_mutex_timedlock(&mutex, &timeout);
    if (result == ETIMEDOUT) {
      return false;
    }
    ecl_assert_throw(result == 0, threads::throwMutexTimedLockException(LOC,result));
    ++number_locks;
  #else
    return trylock(); // fallback option
  #endif
  return true;
}
;

bool Mutex::trylock() ecl_assert_throw_decl(StandardException)
{
  int result = pthread_mutex_trylock(&mutex);

  // result will typically be EBUSY if already locked, so filter it from the assert check.
  if (result == EBUSY) {
    return false;
  }

  ecl_assert_throw(result == 0, threads::throwMutexLockException(LOC,result));

  // If we made it here, it means the attempt locked the mutex.
  ++number_locks;
  return true;
}
;

void Mutex::unlock() ecl_assert_throw_decl(StandardException)
{
  --number_locks;
  int result = pthread_mutex_unlock(&mutex);
  ecl_assert_throw(result == 0, threads::throwMutexUnLockException(LOC,result));
}
;

}
;
// namespace ecl

#endif /* ECL_IS_POSIX */
