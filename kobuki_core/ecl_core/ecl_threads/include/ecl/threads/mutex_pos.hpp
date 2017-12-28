/**
 * @file /include/ecl/threads/mutex_pos.hpp
 *
 * @brief Posix interface for a mutex.
 *
 * @date June 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_MUTEX_POS_HPP_
#define ECL_THREADS_MUTEX_POS_HPP_

/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <errno.h>
#include <cstring> // strerror function
#include <string>
#include <pthread.h>
#include <sstream>
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time/duration.hpp>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef pthread_mutex_t RawMutex; /**< @brief Abstraction representing the fundamental mutex type. **/

/*****************************************************************************
** Class Mutex
*****************************************************************************/
/**
 * @brief Cross-platformable lock for shared memory access in a single application..
 *
 * This class provides an object that handles locking/unlocking memory between
 * threads in the same process. It uses the posix mutex which can be configured
 * in a variety of ways. For convenience, this mutex class is configured so
 * that it uses exception handling for errors (including deadlock checking!) when -DNDEBUG is not
 * present. At some stage later I may set global/local mechanisms for enabling
 * deadlock checking on mutex's if it proves to be a neccessary feature, but for
 * now, its kept simple and as automatic as possible.
 *
 * One other feature of posix mutexes is that you can set priorities for them,
 * but I have not done that here.
 *
 * On linux they are handled in the kernel, so they are fast
 * and schedulers can be set up to optimise how they are handled.
 *
 * <b>Error handling</b>
 *
 * In debug mode, all of the mutex's operations will throw if an error occurs. In
 * addition, posix will attempt to detect deadlocks (and ecl will subsequently throw
 * an exception) while in debug mode.
 *
 * For release mode, there is no flag to check if a mutex is running (generally this
 * is pretty foolproof *or* you have a deadlock and can't do anything anyway. I could
 * provide a graceful fallback with an error flag, but will only do so if really
 * necessary.
 **/
class Mutex {
public:
	/**
	 * @brief Initialises the mutex.
	 *
	 * Assigns the required resources for the mutex. This also automatically
	 * configures the mutex to check for deadlocks if NDEBUG is not defined.
	 *
	 * @param locked : optionally lock the mutex upon creation (default is unlocked).
	 *
	 * @exception StandardException : throws if mutex initialisation fails [debug mode only].
	 */
	Mutex(const bool locked = false) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Destroys the mutex.
	 *
	 * De-allocates the resources allocated to the mutex.
	 */
	virtual ~Mutex();

	/**
	 * @brief Unlocks the mutex.
	 *
	 * @exception StandardException : throws if mutex unlocking fails [debug mode only].
	 */
	void unlock() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Locks the mutex.
	 *
	 * @exception StandardException : throws if mutex locking fails [debug mode only].
	 */
	void lock() ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Attempts to lock the mutex, but will timeout if it fails for a certain duration.
	 *
	 * Attempts to lock the mutex, but will timeout if it fails for a certain duration.
	 *
	 * Note: this function is not always available on posix systems (e.g. macosx).
	 * When it is not present, it will simply default its behaviour to that of a
	 * regular trylock().
	 *
	 * @return bool : success or failure of the attempt to lock the mutex.
	 *
	 * @exception StandardException : throws if mutex timed locking fails [debug mode only].
	 */
	bool trylock(Duration &duration) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Tries to lock, but returns immediately if it can't.
	 *
	 * Like its counterpart lock(), but it won't wait around if the
	 * mutex is already locked.
	 *
	 * @return bool : success or failure of the attempt to lock the mutex.
	 *
	 * @exception StandardException : throws if mutex trylock recorded an error other than an already busy mutex [debug mode only].
	 */
	bool trylock() ecl_assert_throw_decl(StandardException);
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

/*****************************************************************************
** Interface [Exceptions]
*****************************************************************************/

#if defined(ECL_HAS_EXCEPTIONS)
namespace ecl {
namespace threads {

/*****************************************************************************
** Interface [Mutex Exceptions]
*****************************************************************************/

/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutexattr_init</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexAttrException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EINVAL ) : return StandardException(loc, InvalidInputError, "The specified mutex attribute was invalid.");
		case ( ENOMEM ) : return StandardException(loc, MemoryError, "There is insufficient memory for initialisation of the mutex attribute.");
		default         :
		{
			std::ostringstream ostream;
			ostream << "Unknown posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutex_init</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexInitException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EINVAL ) : return StandardException(loc, InvalidInputError, "The specified mutex was invalid.");
		case ( EBUSY )  : return StandardException(loc, InvalidInputError, "The mutex object has already been initialised and not yet destroyed.");
		case ( EAGAIN ) : return StandardException(loc, MemoryError, "The mutex object has already been initialised and not yet destroyed.");
		case ( ENOMEM ) : return StandardException(loc, MemoryError, "There is insufficient memory for initialisation of the mutex.");
		case ( EPERM )  : return StandardException(loc, PermissionsError, "The user does not have the privilege to perform the operation.");
		default         :
		{
			std::ostringstream ostream;
			ostream << "Unknown posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutex_destroy</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexDestroyException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EINVAL ) : return StandardException(loc, DestructorError, "The specified mutex is invalid (for some reason or other).");
		case ( EBUSY )  : return StandardException(loc, DestructorError, "Attempted to destroy the mutex while it was locked.");
		default         : return StandardException(loc, UnknownError, "Unknown error.");
	}
}
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutex_timedlock</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexTimedLockException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EDEADLK ): return StandardException(loc, UsageError, "DEADLOCK! The current thread already owns the mutex.");
		// These aren't relevant to us (EBUSY is normal operation for trylock, EINVAL can't happen because of RAII, and EAGAIN because our mutex isn't RECURSIVE)
		case ( EINVAL ) : return StandardException(loc, UsageError, "The mutex is not initialised or it is priority protected and the calling thread's priority is higher than the mutex' current priority ceiling.");
		case ( EAGAIN ) : return StandardException(loc, OutOfRangeError, "The mutex could not be acquired because the maximum number of recursive locks for the mutex has been exceeded.");
		default         : return StandardException(loc, UnknownError, "Unknown error.");
	}
}
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutex_lock/trylock</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexLockException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EDEADLK ): return StandardException(loc, UsageError, "DEADLOCK! The mutex has already been locked by this thread, it now has to wait on itself.");
		// These aren't relevant to us (EBUSY is normal operation for trylock, EINVAL can't happen because of RAII, and EAGAIN because our mutex isn't RECURSIVE)
		case ( EBUSY )  : return StandardException(loc, ConfigurationError, "The try lock failed because it was already locked (normal operation really, not really an error).");
		case ( EINVAL ) : return StandardException(loc, InvalidInputError, "The mutex does not refer to an initialised mutex.");
		case ( EAGAIN ) : return StandardException(loc, OutOfRangeError, "The mutex could not be acquired because the maximum number of recursive locks for the mutex has been exceeded.");
		default         : return StandardException(loc, PosixError, "Unknown error.");
	}
}
/**
 * This function generates a custom StandardException response
 * for posix error numbers generated by <i>pthread_mutex_unock</i> calls within
 * the Mutex class.
 * @param loc : use with the LOC macro, identifies the line and file of the code.
 * @param error_result : mutex functions do not use errno, so we must pass this function result directly to the handler.
 * @return StandardException : the execption to throw.
 */
inline StandardException ECL_LOCAL throwMutexUnLockException(const char* loc, int error_result) {
	switch (error_result) {
		case ( EINVAL ) : return StandardException(loc, InvalidInputError, "The mutex does not refer to an initialised mutex.");
		case ( EAGAIN ) : return StandardException(loc, OutOfRangeError, "The mutex could not be acquired because the maximum number of recursive locks for the mutex has been exceeded.");
		case ( EPERM )  : return StandardException(loc, PermissionsError, "The user does not have the privilege to perform the operation.");
		default         : return StandardException(loc, UnknownError, "Unknown error.");
	}
}


}; // namespace threads
} // namespace ecl

#endif /* ECL_HAS_EXCEPTIONS */
#endif /* ECL_IS_POSIX */
#endif /* ECL_THREADS_MUTEX_POS_HPP_ */
