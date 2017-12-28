/**
 * @file /ecl_threads/include/ecl/threads/mutex_w32.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 27/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_MUTEX_W32_HPP_
#define ECL_THREADS_MUTEX_W32_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_WIN32)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <windows.h>
#include <ecl/time/duration.hpp>
#include "macros.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef CRITICAL_SECTION RawMutex; /**< @brief Abstraction representing the fundamental mutex type. **/

/*****************************************************************************
** Class Mutex
*****************************************************************************/
/**
 * @brief Cross-platformable lock for shared memory access in a single application..
 *
 * This is a very simple implementation of the win32 mutex using
 * CRITICAL_SECTION objects. It slightly lacks in functionality compared
 * to the posix mutex's - there's no debug deadlock checking mode and there
 * is no timed trylock functionality either. However, it works and the same
 * interface is used across platforms (the timed trylock defaults to an
 * ordinary trylock).
 **/
class ecl_threads_PUBLIC Mutex {
public:
	/**
	 * @brief Initialises the mutex.
	 *
	 * Assigns the required resources for the mutex.
	 *
	 * @param locked : optionally lock the mutex upon creation (default is unlocked).
	 */
	Mutex(const bool locked = false);
	/**
	 * @brief Destroys the mutex.
	 *
	 * De-allocates the resources allocated to the mutex.
	 */
	virtual ~Mutex();

	/**
	 * @brief Unlocks the mutex.
	 *
	 * Unlocks the mutex.
	 */
	void unlock();
	/**
	 * @brief Locks the mutex.
	 *
	 * Locks the mutex.
	 */
	void lock();
	/**
	 * @brief Attempts to lock the mutex, but will timeout if it fails for a certain duration.
	 *
	 * Attempts to lock the mutex, but will timeout if it fails for a certain duration.
	 *
	 * Note: this function is not available on win32 systems.
	 * In order to maintain a standard interface, it will simply default
	 * its behaviour to that of a regular trylock().
	 *
	 * @return bool : success or failure of the attempt to lock the mutex.
	 */
	bool trylock(Duration &duration);
	/**
	 * @brief Tries to lock, but returns immediately if it can't.
	 *
	 * Like its counterpart lock(), but it won't wait around if the
	 * mutex is already locked.
	 *
	 * @return bool : success or failure of the attempt to lock the mutex.
	 */
	bool trylock();
	/**
	 * @brief Number of threads that have this mutex currently locked.
	 *
	 * This represents the number of threads currently associated with
	 * this mutex. There can only be at most 1 accessing, with the rest
	 * are currently waiting for their turn.
	 *
	 * @return unsigned int : the number of locks currently associated with this mutex.
	 **/
	unsigned int locks() { return number_locks; }

	/**
	 * @brief Accesses the system-dependent fundamental mutex type.
	 *
	 * Returns a reference to the underlying mutex type.
	 * @return RawMutex& : a reference to the underlying mutex type.
	 */
	RawMutex& rawType() { return mutex; }

private:
	RawMutex mutex;
	unsigned int number_locks;

};

}; // namespace ecl


#endif /* ECL_IS_WIN32 */
#endif /* ECL_THREADS_MUTEX_W32_HPP_ */
