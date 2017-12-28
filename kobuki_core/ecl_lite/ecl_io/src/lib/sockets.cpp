/**
 * @file /ecl_io/src/lib/sockets.cpp
 *
 * @brief Initialising a socket subsystem.
 *
 * @date February 2011.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstring>
#include <ecl/config/ecl.hpp>
#ifndef ECL_IS_WIN32
  #include <unistd.h>
#endif
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/io/sockets.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

SocketError init_sockets(void) {
#ifdef ECL_IS_WIN32
	static bool already_initialised = false;
	if ( !already_initialised ) {
		struct WSAData wsaData;
		int err;
		/* Initialises use of the Winsock DLL subsystem. */
		if ( (err = WSAStartup(MAKEWORD(2, 0), &wsaData)) != 0) {
			switch( WSAGetLastError() ) {
				case(WSASYSNOTREADY) : { return SocketError(NotInitialisedError); }
				case(WSAVERNOTSUPPORTED) : { return SocketError(NotSupportedError); }
				case(WSAEINPROGRESS) : { return SocketError(BusyError); }
				case(WSAEPROCLIM) : { return SocketError(OutOfResourcesError); }
				case(WSAEFAULT) : { return SocketError(InvalidArgError); }
				default : { return SocketError(UnknownError); }
			}
		}
	}
#endif
	return SocketError(NoError);
}

SocketError close_socket(const socket_descriptor& socket) {
#if defined(ECL_IS_WIN32)
	if ( ::closesocket(socket) == SOCKET_ERROR ) {
		switch(WSAGetLastError()) {
			case(WSANOTINITIALISED) : { return SocketError(NotInitialisedError); }
			case(WSAENETDOWN) : { return SocketError(SystemFailureError); }
			case(WSAENOTSOCK) : { return SocketError(InvalidArgError); }
			case(WSAEINTR) : { return SocketError(InterruptedError); }
			case(WSAEWOULDBLOCK) : { return SocketError(BlockingError); }
			default : { return SocketError(UnknownError); }
		}
	}
#elif defined(ECL_IS_POSIX)
	if ( ::close(socket) != 0 ) {
		switch(errno) {
			case(EBADF) : { return SocketError(InvalidArgError); }
			case(EINTR) : { return SocketError(InterruptedError); }
			case(EIO) :   { return SocketError(InterruptedError); }
			default : { return SocketError(UnknownError); }
		}
	}
#endif
	return SocketError(NoError);
}

SocketError shutdown_sockets() {
#ifdef ECL_IS_WIN32
	if ( WSACleanup() == SOCKET_ERROR ) {
		switch(WSAGetLastError()) {
			case(WSANOTINITIALISED) : { return SocketError(NotInitialisedError); }
			case(WSAENETDOWN) : { return SocketError(SystemFailureError); }
			case(WSAEINPROGRESS) : { return SocketError(BusyError); }
			default : { return SocketError(UnknownError); }
		}
	}
#endif
	return SocketError(NoError);
}


} // namespace ecl
