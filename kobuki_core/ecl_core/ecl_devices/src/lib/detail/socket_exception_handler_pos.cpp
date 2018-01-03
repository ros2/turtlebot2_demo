/**
 * @file /src/lib/detail/socket_exception_handler_pos.cpp
 *
 * @brief Implements exception handling for posix sockets.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Cross platform
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_IS_APPLE
#ifdef ECL_IS_POSIX

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sstream>
#include <errno.h>
#include <netdb.h> // gethostbyname
#include <ecl/exceptions/standard_exception.hpp>
#include "../../../include/ecl/devices/detail/socket_exception_handler_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

StandardException socket_exception(const char* loc) {
    switch( errno ) {
        case ( EACCES )     : return StandardException(LOC,OpenError,"Unable to open socket. Permission to create is denied.");
        case ( EAFNOSUPPORT): return StandardException(LOC,NotSupportedError,"Unable to open socket. Your implementation does not support the specified address family (in this case AF_INET or otherwise known as ipv4).");
        case ( EINVAL )     : return StandardException(LOC,InvalidArgError,"Unable to open socket. Unknown or invalid protocol, family.");
        case ( EMFILE )     : return StandardException(LOC,OutOfRangeError,"Unable to open socket. Process file table overflow.");
        case ( ENFILE )     : return StandardException(LOC,OutOfResourcesError,"Unable to open socket. The system limit on the number of open files has been reached.");
        case ( ENOBUFS )    : return StandardException(LOC,MemoryError,"Unable to open socket. Insufficient memory available.");
        case ( ENOMEM )     : return StandardException(LOC,MemoryError,"Unable to open socket. Insufficient memory available.");
        case ( EPROTONOSUPPORT )    : return StandardException(LOC,NotSupportedError,"Unable to open socket. The protocol type (socket streams) is not supported within this address family (ipv4).");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown errno [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException bind_exception(const char* loc) {
    switch( errno ) {
        case ( EACCES )     : return StandardException(LOC,PermissionsError,"Unable to bind the socket. The address is protected (maybe need to be superuser?).");
        case ( EADDRINUSE ) : return StandardException(LOC,BusyError,"Unable to bind the socket. Address already in use (might be timing out, try again in a moment).");
        case ( EBADF )      : return StandardException(LOC,InvalidObjectError,"Unable to bind the socket. Not a valid socket descriptor.");
        case ( EINVAL )     : return StandardException(LOC,BusyError,"Unable to bind the socket. The socket is already bound to an address.");
        case ( ENOTSOCK )   : return StandardException(LOC,InvalidObjectError,"Unable to bind the socket. The descriptor is a file descriptor, not a socket descriptor.");
        case ( EADDRNOTAVAIL ) : return StandardException(LOC,InvalidObjectError,"Unable to bind the socket. Interface does not exist or is not local.");
        case ( EFAULT )     : return StandardException(LOC,OutOfRangeError,"Unable to bind the socket. Socket specification is outside the user address space.");
        case ( ELOOP )      : return StandardException(LOC,SystemFailureError,"Unable to bind the socket. Too many symbolic links involved.");
        case ( ENAMETOOLONG )  : return StandardException(LOC,InvalidArgError,"Unable to bind the socket. Address is too long.");
        case ( ENOENT )     : return StandardException(LOC,InvalidObjectError,"Unable to bind the socket. The file does not exist.");
        case ( ENOMEM )     : return StandardException(LOC,MemoryError,"Unable to bind the socket. Insufficient kernel memory.");
        case ( ENOTDIR )    : return StandardException(LOC,InvalidArgError,"Unable to bind the socket. A component of the path prefix is not a directory.");
        case ( EROFS )      : return StandardException(LOC,PermissionsError,"Unable to bind the socket. Socket inode resides on a read only file system.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException accept_exception(const char* loc) {
    switch( errno ) {
		case ( EWOULDBLOCK ) : return StandardException(LOC,BlockingError,"Unable to accept client connection. The socket is non-blocking and no connections are available.");
		case ( EBADF )       : return StandardException(LOC,InvalidObjectError,"Unable to accept client connection. Not a valid socket descriptor.");
		case ( ECONNABORTED ): return StandardException(LOC,InterruptedError,"Unable to accept client connection. A connection has been aborted.");
		case ( EINTR )       : return StandardException(LOC,InterruptedError,"Unable to accept client connection. A system signal has interrupted.");
		case ( EINVAL )      : return StandardException(LOC,UsageError,"Unable to accept client connection. Socket is not listening for connections or address length is invalid.");
		case ( EMFILE )      : return StandardException(LOC,OutOfResourcesError,"Unable to accept client connection. The system or per-process limit on files has been reached.");
		case ( ENFILE )      : return StandardException(LOC,OutOfResourcesError,"Unable to accept client connection. The system or per-process limit on files has been reached.");
		case ( ENOTSOCK )    : return StandardException(LOC,InvalidObjectError,"Unable to accept client connection. The descriptor is a file descriptor, not a socket descriptor..");
		case ( EOPNOTSUPP )  : return StandardException(LOC,InvalidObjectError,"Unable to accept client connection. The client socket is not of type SOCK_STREAM.");
		case ( EFAULT )      : return StandardException(LOC,PermissionsError,"Unable to accept client connection. The address argument is not writable by the user.");
		case ( ENOBUFS )     : return StandardException(LOC,MemoryError,"Unable to accept client connection. Not enough free memory (buffer or system).");
		case ( ENOMEM )      : return StandardException(LOC,MemoryError,"Unable to accept client connection. Not enough free memory (buffer or system).");
		case ( EPROTO )      : return StandardException(LOC,InvalidArgError,"Unable to accept client connection. Protocol error.");
		case ( EPERM )       : return StandardException(LOC,PermissionsError,"Unable to accept client connection. Permissions do not allow this connection.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException receive_exception(const char* loc) {

    switch( errno ) {
		case ( EAGAIN || EWOULDBLOCK ) : return StandardException(LOC,InterruptedError,"Unable to read the socket. Probably a timeout occured.");
		case ( EBADF       ) : return StandardException(LOC,InvalidObjectError,"Unable to read the socket. Bad file descriptor.");
		case ( ECONNREFUSED) : return StandardException(LOC,ConnectionError,"Unable to read the socket. Remote host refused the connection (probably not running).");
		case ( EFAULT      ) : return StandardException(LOC,SystemFailureError,"Unable to read the socket. Receive buffer has an address problem.");
		case ( EINTR       ) : return StandardException(LOC,InterruptedError,"Unable to read the socket. Signal interruption.");
		case ( EINVAL      ) : return StandardException(LOC,InvalidArgError,"Unable to read the socket. Invalid argument was used.");
		case ( ENOMEM      ) : return StandardException(LOC,MemoryError,"Unable to read the socket. Could not allocate memory for the operation.");
		case ( ENOTCONN    ) : return StandardException(LOC,ConnectionError,"Unable to read the socket. Has not been connected.");
		case ( ENOTSOCK    ) : return StandardException(LOC,InvalidObjectError,"Unable to read the socket. The file descriptor does not refer to a socket.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException send_exception(const char* loc) {

	switch( errno ) {
		case ( EAGAIN || EWOULDBLOCK ) : return StandardException(LOC,BlockingError,"Unable to write to the socket. Socket is configured as non-blocking and this would block.");
		case ( EWOULDBLOCK ) : return StandardException(LOC,BlockingError,"Unable to write to the socket. Socket is configured as non-blocking and this would block.");
		case ( EACCES      ) : return StandardException(LOC,PermissionsError,"Unable to write to the socket. Permission to write is denied.");
		case ( EBADF       ) : return StandardException(LOC,InvalidObjectError,"Unable to write to the socket. Bad file descriptor.");
		case ( ECONNRESET  ) : return StandardException(LOC,InterruptedError,"Unable to write to the socket. Connection reset by peer.");
		case ( EFAULT      ) : return StandardException(LOC,SystemFailureError,"Unable to write to the socket. Buffer has an address problem.");
		case ( EINTR       ) : return StandardException(LOC,InterruptedError,"Unable to write to the socket. Signal interruption.");
		case ( EINVAL      ) : return StandardException(LOC,InvalidArgError,"Unable to write to the socket. Invalid argument was used.");
		case ( EISCONN     ) : return StandardException(LOC,ConnectionError,"Unable to write to the socket. Connection mismatch???");
		case ( EMSGSIZE    ) : return StandardException(LOC,WriteError,"Unable to write to the socket. Socket type required to send atomically, but the size of this message is too large to handle in this way.");
		case ( ENOBUFS     ) : return StandardException(LOC,OutOfResourcesError,"Unable to write to the socket. Output queue is full (could be caused by transient congestion, but this doesn't usually happen in linux which typically just drops packets).");
		case ( ENOMEM      ) : return StandardException(LOC,MemoryError,"Unable to write to the socket. Could not allocate memory for the operation.");
		case ( ENOTCONN    ) : return StandardException(LOC,ConnectionError,"Unable to write to the socket. Has not been connected.");
		case ( ENOTSOCK    ) : return StandardException(LOC,InvalidObjectError,"Unable to write to the socket. The file descriptor does not refer to a socket.");
		case ( EOPNOTSUPP  ) : return StandardException(LOC,NotSupportedError,"Unable to write to the socket. Some api here not supported.");
		case ( EPIPE       ) : return StandardException(LOC,InterruptedError,"Unable to write to the socket. Local end has been shutdown. Probably bad and will receive a SIGPIPE signal too.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException ioctl_exception(const char* loc) {

	switch( errno ) {
		case ( EBADF       ) : return StandardException(LOC,InvalidObjectError, "Socket control error. The file descriptor was not valid.");
		case ( EFAULT      ) : return StandardException(LOC,OutOfRangeError, "Socket control error. Tried to reference inaccessible memory.");
		case ( EINVAL      ) : return StandardException(LOC,InvalidArgError, "Socket control error. Ioctl input arguments were not valid.");
		case ( ENOTTY      ) : return StandardException(LOC,InvalidObjectError, "Socket control error. The file descriptor is not valid or this operation may not be performed on it.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }
}

StandardException gethostbyname_exception(const char* loc, const std::string& hostname) {
	switch( h_errno ) {
		case ( HOST_NOT_FOUND ) : {
			std::string header;
			header += "Unable to correctly determine the server hostname: ";
			header += hostname;
			return StandardException(LOC,OpenError,header);
		}
		case ( TRY_AGAIN ) : return StandardException(LOC, OpenError,"A temporary error occurred on an authoritative name server. Try again later.");
		case ( NO_ADDRESS )  : return StandardException(LOC,InvalidArgError,"Requested server hostname is valid, but does not have an IP address.");
		case ( NO_RECOVERY ) : return StandardException(LOC,UnknownError);
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << h_errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
	}
}

StandardException connection_exception(const char* loc) {
    switch( errno ) {
        case ( ( EACCES ) || ( EPERM ) ): return StandardException(LOC,PermissionsError,"Write permission on the socket denied or firewalled.");
        case ( EADDRINUSE )  : return StandardException(LOC,BusyError,"Address already in use.");
        case ( EAFNOSUPPORT ): return StandardException(LOC,NotSupportedError,"Incorrect address family used (no support for AF maybe?");
        case ( EAGAIN )      : return StandardException(LOC,OutOfResourcesError,"No free local ports remaining.");
        case ( EALREADY )    : return StandardException(LOC,BlockingError,"Socket is non-blocking and a previous connection attempt has not yet completed (wtf?).");
        case ( EBADF )       : return StandardException(LOC,InvalidObjectError,"Not a valid socket descriptor.");
        case ( ECONNREFUSED ): return StandardException(LOC,ConnectionRefusedError,"Connection refused (no-one listening).");
        case ( EFAULT )      : return StandardException(LOC,OutOfRangeError,"Socket specification is outside the user address space.");
        case ( EINPROGRESS ) : return StandardException(LOC,BlockingError,"Socket is non-blocking and the connection cannot be completed immediately (try select or poll for writing).");
        case ( EINTR )       : return StandardException(LOC,InterruptedError,"Connection interrupted by a system signal.");
        case ( EISCONN )     : return StandardException(LOC,ConnectionError,"This socket is already connected.");
        case ( ENETUNREACH ) :
        case ( EHOSTUNREACH ): return StandardException(LOC,NotFoundError,"The host is unreachable.");
        case ( ENOTSOCK )    : return StandardException(LOC,InvalidObjectError,"This is not a socket file descriptor.");
        case ( ETIMEDOUT )   : return StandardException(LOC,TimeOutError,"Timed out.");
		default             : {
			std::ostringstream ostream;
			ostream << "Unknown error [" << errno << "]";
			return StandardException(loc, UnknownError, ostream.str());
		}
    }

}


} // namespace devices
} // namespace ecl

#endif  /* ECL_IS_POSIX */
#endif  /* !ECL_IS_APPLE */

