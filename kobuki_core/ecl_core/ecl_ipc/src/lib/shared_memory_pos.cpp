/**
 * @file /ecl_ipc/src/lib/shared_memory_pos.cpp
 *
 * @brief Magic trick to make sure the library is called for shared memory.
 *
 * If using --as-needed and shared memory objects, this library will often
 * not get called (as the shared memory class is 100% headers). This will
 * result in sometimes (if nothing else calls it) -lrt not being called
 * and ultimately you will get undefined references to shm_unlink and co.
 *
 * This is some magic to make sure that you always bring in the library
 * when using the shared memory class.
 *
 * @date January 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/


#include "../../include/ecl/ipc/shared_memory_pos.hpp"

#ifdef ECL_HAS_POSIX_SHARED_MEMORY

#include <string>
#include <sys/mman.h>        /* For shm_open() */
#include <fcntl.h>           /* For O_* constants */
#include <errno.h>
#include <ecl/exceptions/macros.hpp>
#include <iostream>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace ipc {

void SharedMemoryBase::unlink() {
	shm_unlink(name.c_str());
}

int SharedMemoryBase::open() {
    /*********************
     * Open Flags
     *********************/
    /*
     * O_CREAT    : it will try and create some memory or just open if it already exists
     *              if the shared memory object already exists.
     * O_EXCL     : when used with O_CREAT fails to open if the message queue already exists.
     *            : if neither, then it will only open an existing memory.
     * O_WRONLY   : write only.
     * O_RDONLY   : read only.
     * O_RDWR     : read-write.
     * O_NONBLOCK : normally it will block if sending to a full queue. Disable this.
     */
    static const int open_flags = O_RDWR;
    static const int create_flags = O_CREAT|O_RDWR|O_EXCL;
    /*********************
     * Permissions
     *********************/
    /*
     * Permissions. Must be specified when O_CRELOC is used, otherwise it is ignored. It
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
    */
    static const int permissions = S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH;
    /*************************************************************************
    ** Name
    *************************************************************************/
    /*
     * The name is like a filename. To be compatible on all systems, always
     * begin with a leading '/' and include no other '/'s.
     * In debug mode, might be worth parsing the name here.
     */
    /*************************************************************************
    ** Open
    *************************************************************************/
    int shm_descriptor = shm_open(name.c_str(),create_flags,permissions);
    if ( ( shm_descriptor == -1 ) && ( errno == EEXIST ) ) {
        // Maybe its already open, try opening it normally
        shm_descriptor = shm_open(name.c_str(),open_flags,permissions);
    } else {
    	shared_memory_manager = true;
    }
    if ( shm_descriptor == -1) {
    }
	return shm_descriptor;
}


/*****************************************************************************
** Exception Handlers
*****************************************************************************/

#ifndef ECL_DISABLE_EXCEPTIONS

ecl::StandardException openSharedSectionException(const char* loc) {
	int error_result = errno;
	switch (error_result) {
		case ( EACCES ) : {
			throw StandardException(LOC,PermissionsError,"Opening shared memory failed - permission denied.");
			break;
		}
	//            case ( EEXIST ) : { // If using O_EXCL, need this, otherwise no.
	//                // pathname exists, assume its a fifo and continue
	//                break;
	//            }
		case ( EMFILE ) : case ( ENFILE ) : {
			throw StandardException(LOC,OutOfResourcesError,"Opening shared memory failed - too many file connections already open.");
			break;
		}
		case ( ENOENT ) : case ( ENAMETOOLONG ) : case ( EINVAL ) : {
			throw StandardException(LOC,InvalidArgError,"Opening shared memory failed - pathname problem.");
			break;
		}
		case ( ENOSYS ) : {
			throw StandardException(LOC,NotSupportedError,"Opening shared memory failed - kernel system functions are not available (remake the kernel).");
			break;
		}
		default         :
		{
			std::ostringstream ostream;
			ostream << "Posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}

ecl::StandardException memoryMapException(const char* loc) {
	int error_result = errno;
	switch (error_result) {
		case ( EACCES ) : {
			return StandardException(LOC,PermissionsError,"Shared mapping failed - permission problems (see man mmap).");
		}
		case ( EAGAIN ) : {
			return StandardException(LOC,MemoryError,"Shared mapping failed - file locked or too much memory has been locked.");
		}
		case ( EBADF  ) : {
			return StandardException(LOC,InvalidArgError,"Shared mapping failed - not a valid file descriptor (see man mmap).");
		}
		case ( EINVAL ) : {
			return StandardException(LOC,InvalidArgError,"Shared mapping failed - start, length or offset were invalid or MAP_PRIVLOCE and MAP_SHARED were either both present or both obso (see man mmap).");
		}
		case ( ENFILE ) : {
			return StandardException(LOC,OutOfResourcesError,"Shared mapping failed - system limit on the total number of open files has been reached (see man mmap).");
		}
		case ( ENODEV ) : {
			return StandardException(LOC,NotSupportedError,"Shared mapping failed - underlying filesystem of the specified file doesn't support memory mapping (see man mmap).");
		}
		case ( ENOMEM ) : {
			return StandardException(LOC,MemoryError,"Shared mapping failed - no mem available, or max mappings exceeded (see man mmap).");
		}
		case ( EPERM ) : {
			return StandardException(LOC,PermissionsError,"Shared mapping failed - EPERM (see man mmap).");
		}
//		case ( ETXBSY ) : {
//			return StandardException(LOC,"Shared mapping failed.","ETXBSY (see man mmap).");
//			break;
//		}
		default         :
		{
			std::ostringstream ostream;
			ostream << "Posix error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}
#endif /* ECL_DISABLE_EXCEPTIONS */

} // namespace ipc
} // namespace ecl

#endif /* ECL_HAS_POSIX_SHARED_MEMORY */
