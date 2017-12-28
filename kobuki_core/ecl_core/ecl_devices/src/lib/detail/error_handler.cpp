/**
 * @file /src/lib/detail/error_handler_pos.cpp
 *
 * @brief Implements simple error handling for posix devices.
 *
 * @date April 2009
 **/

/*****************************************************************************
** Cross platform
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <errno.h>
#include <ecl/errors/handlers.hpp>
#include "../../../include/ecl/devices/detail/error_handler.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

/*****************************************************************************
** Implementation [Exception Handlers]
*****************************************************************************/

ecl::Error open_error() {
	int error_result = errno;
	switch (error_result) {
		case ( EINVAL ) : return ecl::Error(ecl::InvalidArgError);
		case ( EACCES ) : return ecl::Error(ecl::PermissionsError);
		case ( EFBIG )  : case (EOVERFLOW)  :
						  return ecl::Error(ecl::OutOfResourcesError);
		case ( EINTR )  : return ecl::Error(ecl::InterruptedError);
		case ( EISDIR ) : return ecl::Error(ecl::InvalidObjectError);
		case ( ELOOP )  : return ecl::Error(ecl::SystemFailureError);
		case ( EMFILE ) : return ecl::Error(ecl::OutOfResourcesError);
		case ( ENFILE ) : return ecl::Error(ecl::OutOfResourcesError);
		case ( ENAMETOOLONG ) : return ecl::Error(ecl::InvalidArgError);
		case ( ENOMEM ) : return ecl::Error(ecl::MemoryError);
		case ( ENOSPC ) : return ecl::Error(ecl::OutOfResourcesError);
		case ( ENOTDIR ): return ecl::Error(ecl::InvalidObjectError);
		case ( EROFS )  : return ecl::Error(ecl::PermissionsError);
		case ( ETXTBSY ): return ecl::Error(ecl::UsageError);
		default         : return ecl::Error(ecl::UnknownError);
	}
}


ecl::Error write_error() {
	int error_result = errno;
	switch (error_result) {
		case ( EAGAIN ) : return ecl::Error(ecl::BlockingError);
		case ( EBADF  ) : case (EINVAL)
						: return ecl::Error(ecl::InvalidObjectError);
		case ( EFAULT ) : return ecl::Error(ecl::OutOfRangeError);
		case ( EFBIG  ) : return ecl::Error(ecl::MemoryError);
		case ( EINTR )  : return ecl::Error(ecl::InterruptedError);
		case ( EIO    ) : return ecl::Error(ecl::SystemFailureError);
		case ( ENOSPC ) : return ecl::Error(ecl::OutOfResourcesError);
		case ( EPIPE  ) : return ecl::Error(ecl::PermissionsError);
		default         : return ecl::Error(ecl::UnknownError);
	}
}


ecl::Error read_error() {
	int error_result = errno;
	switch (error_result) {
		case ( EAGAIN ) : return ecl::Error(ecl::BlockingError);
		case ( EBADF  ) : case (EINVAL)
						: return ecl::Error(ecl::PermissionsError);
		case ( EFAULT ) : return ecl::Error(ecl::OutOfRangeError);
		case ( EINTR )  : return ecl::Error(ecl::InterruptedError);
		case ( EIO    ) : return ecl::Error(ecl::SystemFailureError);
		case ( EISDIR ) : return ecl::Error(ecl::InvalidObjectError);
		default         : return ecl::Error(ecl::UnknownError);
	}
}
ecl::Error sync_error() {
	int error_result = errno;
	switch (error_result) {
		case ( EBADF  ) : return ecl::Error(ecl::InvalidArgError);
		case ( EIO  )   : return ecl::Error(ecl::CloseError);
		case ( EROFS )  : case ( EINVAL )
						: return ecl::Error(ecl::NotSupportedError);
		default         : return ecl::Error(ecl::UnknownError);
	}
}
ecl::Error close_error() {
	int error_result = errno;
	switch (error_result) {
		case ( EBADF  ) : return ecl::Error(ecl::InvalidArgError);
		case ( EIO  )   : return ecl::Error(ecl::SystemFailureError);
		case ( EINTR )  : return ecl::Error(ecl::InterruptedError);
		default         : return ecl::Error(ecl::UnknownError);
	}
}

} // namespace devices
} // namespace ecl
