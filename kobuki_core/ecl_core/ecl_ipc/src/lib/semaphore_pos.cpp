/**
 * @file /src/lib/semaphore_pos.cpp
 *
 * @brief Posix semaphore implementation.
 *
 * @date August 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/ipc/semaphore_pos.hpp"

#ifdef ECL_HAS_POSIX_SEMAPHORES

#include <errno.h>
#include <sstream>
#include <ecl/errors/handlers.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/time_lite/functions.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using std::ostringstream;

/*****************************************************************************
** Implementation [Semaphore]
*****************************************************************************/

Semaphore::Semaphore() throw(StandardException) {
	// Never use this constructor
	throw StandardException(LOC, RaiiError);
}

Semaphore::Semaphore(const std::string& string_id) ecl_assert_throw_decl(StandardException):
	name(string("/"+string_id)),
	semaphore(NULL)
{
    /*********************
     * Open Flags
     *********************/
    /*
     * O_CREAT    : it will try and create a semaphore or just open if it already exists
     * O_EXCL     : when used with O_CREAT fails to open if the semaphore already exists.
     *
     * Other flags are unnecessary.
     */
    static const int open_flags = O_CREAT;
    /*********************
     * Permissions
     *********************/
    /*
     * Permissions. Must be specified when O_CREAT is used, otherwise it is ignored. It
     * is combined with the process umask (permissions & ~umask).
     * S_IRWXU [00700] - read, write & exec by user
     * S_IRUSR [00400] - read by the user who owns it
     * S_IWUSR [00200] - write by user
     * S_IXUSR [00100] - exec by user
     * S_IRWXG [00070] - read, write & exec by group
     * S_IRGRP [00040] - read by group
     * S_IWGRP [00020] - write by group
     * S_IXGRP [00010] - exe by group
     * S_IRWXG [00007] - read, write & exec by other
     * S_IROTH [00004] - read by other
     * S_IWOTH [00002] - write by other
     * S_IXOTH [00001] - exe by other
     *
     * Semaphores just have access, or they do not (p.137 of the posix bible). Just
     * set rwx on whichever subset of users applies. As we're not worried about
     * security here, just open them up.
    */
    static const int permissions = S_IRWXU|S_IRWXG|S_IRWXO;
    /*
     * The initial value of the semaphore. For mutex style semaphores, set the
     * initial value to be unlocked.
     */
    static const int initial_value_unlocked = 1;

    /*********************
    ** Open
    **********************/
    semaphore = sem_open(name.c_str(),open_flags,permissions,initial_value_unlocked);
    ecl_assert_throw(semaphore != SEM_FAILED, ipc::openSemaphoreException(LOC));
}

Semaphore::~Semaphore()
{
	// Should check the error, but only fails if the semaphore is invalid (checked by constructor).
    sem_close(semaphore);
	// Should check the error, but only fails with noncritical errors.
    sem_unlink(name.c_str());
}

void Semaphore::lock()
{
	sem_wait(semaphore);
}

int Semaphore::count()
{
	// Should check the error, but only fails if the semaphore is invalid (checked by constructor).
    int semaphore_count;
    sem_getvalue(semaphore,&semaphore_count);
    return semaphore_count;
}

void Semaphore::unlock()
{
	// Should check the error, but only fails if the semaphore is invalid (checked by constructor).
	// The other error is an overflow error, which is handled by constraining our count to 1.
    if ( count() == 1 ) { // Already unlocked
        return;
    } else {
        sem_post(semaphore);
    }
}

bool Semaphore::trylock()
{
	// 0 is a success
	if (  sem_trywait( semaphore ) == 0 ) {
		return true;
	} else {
		// Should check errno = EAGAIN for implication that a locked semaphore caused it to fail
		// and not something else
		return false;
	}
}

bool Semaphore::trylock( const Duration &timeout ) ecl_debug_throw_decl(StandardException) {

    #if defined(_POSIX_TIMEOUTS) && (_POSIX_TIMEOUTS - 200112L) >= 0L
		long tnsec;
		timespec ctime;

		if ( epoch_time(ctime).flag() != NoError ) { return false; }

		ctime.tv_sec += timeout.sec();

		tnsec = ctime.tv_nsec + timeout.nsec();
		if(tnsec >=  999999999 ) { ctime.tv_sec += 1; }
		tnsec %= 1000000000;
		ctime.tv_nsec = tnsec;

		// We already guarantee timeout is the right structure, so no
		// need to check for EINVAL, other errors should be almost
		// irrelevant for us too.
		int result = sem_timedwait( semaphore, &ctime );
		if ( result != 0 ) {
			if ( errno == ETIMEDOUT ) {
				// The timeout kicked in
				return false;
			} else {
				ecl_debug_throw( ipc::tryLockSemaphoreException(LOC));
				// either a signal interrupt EINTR
				// or a blocking (when does this occur? EAGAIN
				return false;
			}
		}
	#else
		return trylock(); // fallback to standard trylock
	#endif
	return true;
}

/*****************************************************************************
** Exception Handlers
*****************************************************************************/

#ifdef ECL_HAS_EXCEPTIONS

namespace ipc {

ecl::StandardException openSemaphoreException(const char* loc) {
	int error_result = errno;
    switch ( error_result ) {
        case ( EACCES ) : {
            return StandardException(LOC,PermissionsError,"The semaphore exists, but permission to open has been denied.");
        }
        case ( EEXIST ) : {
            return StandardException(LOC,PermissionsError,"The semaphore already exists, so your request to explicitly create was denied.");
        }
        case ( ENOENT ) : {
            return StandardException(LOC,ConfigurationError,"The semaphore requested doesn't already exist (you specifically requested it to just open, not create).");
        }
        case ( ENOMEM ) : {
            return StandardException(LOC,MemoryError,"Insufficient memory.");
        }
        case ( EINVAL ) : {
            return StandardException(LOC,InvalidArgError,"Name was empty (i.e. '/'). Can also be the maximum number of semaphores has already been exceeded.");
        }
        case ( EMFILE ) : {
        	return StandardException(LOC,OutOfResourcesError,"This process has already exceeded the number of files/pseudofiles it is permitted to open.");
		}
        case ( ENFILE ) : {
        	return StandardException(LOC,OutOfResourcesError,"This system has already exceeded the number of files/pseudofiles it is permitted to open.");
		}
        case ( ENAMETOOLONG ) : {
        	return StandardException(LOC,InvalidArgError,"The semaphore name was too long.");
		}
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

ecl::StandardException tryLockSemaphoreException(const char* loc) {
	int error_result = errno;
    switch ( error_result ) {
        case ( EINTR ) : {
            return StandardException(LOC,InterruptedError,"Waiting for the semaphore lock was interrupted by a system signal.");
        }
        case ( EINVAL ) : {
            return StandardException(LOC,InvalidArgError,"The semaphore was invalid or the timeout structure specified was invalid.");
        }
        case ( EAGAIN ) : {
            return StandardException(LOC,BlockingError,"The waiting operation could not be performed without blocking???");
        }
		default         :
		{
			ostringstream ostream;
			ostream << "Posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

} // namespace ipc

#endif /* ECL_HAS_EXCEPTIONS */

}; // namespace ecl

#endif /* ECL_HAS_POSIX_SEMAPHORES */
