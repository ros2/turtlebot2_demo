/**
 * @file /src/lib/detail/exception_handler_pos.cpp
 *
 * @brief Implements exception handling for posix devices.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Cross platform
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <errno.h>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../../include/ecl/devices/detail/exception_handler_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

/*****************************************************************************
** Using
*****************************************************************************/

using std::ostringstream;
using std::string;
using ecl::StandardException;


/*****************************************************************************
** Implementation [Exception Handlers]
*****************************************************************************/

StandardException open_exception(const char* loc, const std::string& file_name) {
	int error_result = errno;
	switch (error_result) {
		case ( ENOENT ) : return StandardException(loc, ecl::NotFoundError, file_name + std::string(" could not be found."));
		case ( EINVAL ) : return StandardException(loc, ecl::InvalidArgError, "File mode setting (read/write/append) was incorrectly specified.");
		case ( EACCES ) : return StandardException(loc, ecl::PermissionsError, string("Could not open ")+file_name+string(". Access permission was denied."));
		case ( EFBIG )  : case (EOVERFLOW)  :
						  return StandardException(loc, ecl::OutOfResourcesError, string("Could not open ")+file_name+string(". File was too large (you need to use alternative api/configuration)."));
		case ( EINTR )  : return StandardException(loc, ecl::InterruptedError, string("Could not open ")+file_name+string(". Interrupted by a signal while opening."));
		case ( EISDIR ) : return StandardException(loc, ecl::InvalidObjectError, string("Could not open ")+file_name+string(". This is a directory and not a file."));
		case ( ELOOP )  : return StandardException(loc, ecl::SystemFailureError, string("Could not open ")+file_name+string(". Very nested symbolic link hell."));
		case ( EMFILE ) : return StandardException(loc, ecl::OutOfResourcesError, string("Could not open ")+file_name+string(". This process has already maxxed out its permitted number of open files."));
		case ( ENFILE ) : return StandardException(loc, ecl::OutOfResourcesError, string("Could not open ")+file_name+string(". This system has already maxxed out its permitted number of open files."));
		case ( ENAMETOOLONG ) : return StandardException(loc, ecl::InvalidArgError, string("Could not open ")+file_name+string(". The file name is too long."));
		case ( ENOMEM ) : return StandardException(loc, ecl::MemoryError, string("Could not open ")+file_name+string(". Insufficient memory."));
		case ( ENOSPC ) : return StandardException(loc, ecl::OutOfResourcesError, string("Could not open ")+file_name+string(". The container device (usually hard disk) has insufficient space to create the file."));
		case ( ENOTDIR ): return StandardException(loc, ecl::InvalidObjectError, string("Could not open ")+file_name+string(". Pathname invalid (a directory was not a directory)."));
		case ( EROFS )  : return StandardException(loc, ecl::PermissionsError, string("Could not open ")+file_name+string(". Trying to write to a readonly file system."));
		case ( ETXTBSY ): return StandardException(loc, ecl::UsageError, string("Could not open ")+file_name+string(". Trying to write to a currently executing file."));
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown errno " << error_result << " [" << strerror(error_result) << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}


StandardException write_exception(const char* loc) {
	int error_result = errno;
	switch (error_result) {
		case ( EAGAIN ) : return StandardException(loc, ecl::BlockingError, "The device has been marked non blocking and the write would block.");
		case ( EBADF  ) : case (EINVAL)
						: return StandardException(loc, ecl::InvalidObjectError, "The device is not a valid device for writing.");
		case ( EFAULT ) : return StandardException(loc, ecl::OutOfRangeError, "The device's write buffer is outside your accessible address space.");
		case ( EFBIG  ) : return StandardException(loc, ecl::MemoryError, "Tried to write beyond the device's (or process's) size limit.");
		case ( EINTR )  : return StandardException(loc, ecl::InterruptedError, "A signal interrupted the write.");
		case ( EIO    ) : return StandardException(loc, ecl::SystemFailureError, "A low level input-output error occured (possibly beyond your control).");
		case ( ENOSPC ) : return StandardException(loc, ecl::OutOfResourcesError, "The device has no room left for the data you are trying to write.");
		case ( EPIPE  ) : return StandardException(loc, ecl::PermissionsError, "You tried to write to a pipe whose reading end is closed.");
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}


StandardException read_exception(const char* loc) {
	int error_result = errno;
	switch (error_result) {
		case ( EAGAIN ) : return StandardException(loc, ecl::BlockingError, "The device has been marked non blocking and the read would block.");
		case ( EBADF  ) : case (EINVAL)
						: return StandardException(loc, ecl::PermissionsError, "The device is not a valid device for reading.");
		case ( EFAULT ) : return StandardException(loc, ecl::OutOfRangeError, "The device's read buffer is outside your accessible address space.");
		case ( EINTR )  : return StandardException(loc, ecl::InterruptedError, "A signal interrupted the read.");
		case ( EIO    ) : return StandardException(loc, ecl::SystemFailureError, "A low level input-output error occured (possibly beyond your control).");
		case ( EISDIR ) : return StandardException(loc, ecl::InvalidObjectError, "The file descriptor refers to a directory (not readable).");
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}
StandardException sync_exception(const char* loc, const std::string &file_name) {
	int error_result = errno;
	switch (error_result) {
		case ( EBADF  ) : return StandardException(loc, ecl::InvalidArgError, string("Could not sync ") + file_name + string(", the file descriptor was not valid for writing."));
		case ( EIO  )   : return StandardException(loc, ecl::CloseError, string("Could not sync ") + file_name + string(", could not synchronize while closing."));
		case ( EROFS )  : case ( EINVAL )
						: return StandardException(loc, ecl::NotSupportedError, string("Could not sync ") + file_name + string(", file descriptor does not support synchronization."));
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}
StandardException close_exception(const char* loc, const std::string &file_name) {
	int error_result = errno;
	switch (error_result) {
		case ( EBADF  ) : return StandardException(loc, ecl::InvalidArgError, string("Could not close ") + file_name + string(". The associated file descriptor was not valid."));
		case ( EIO  )   : return StandardException(loc, ecl::SystemFailureError, string("Could not close ") + file_name + string(". Closing io problem."));
		case ( EINTR )  : return StandardException(loc, ecl::InterruptedError, string("Could not close ") + file_name + string(". Interrupted by a signal."));
		default         :
		{
			ostringstream ostream;
			ostream << "Unknown error " << error_result << ": " << strerror(error_result) << ".";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}

} // namespace devices
} // namespace ecl


