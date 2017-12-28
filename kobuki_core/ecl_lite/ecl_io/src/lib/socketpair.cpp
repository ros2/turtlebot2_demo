/**
 * @file /ecl_io/src/lib/socketpair.cpp
 *
 * @brief Crossplatform implementation for generating a local socket pair.
 *
 * @date February, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cerrno>
#include "../../include/ecl/io/socketpair.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

SocketError socketpair(socket_descriptor socket_fd_pair[2], const bool non_blocking) {
#ifdef ECL_IS_WIN32

    union {
       struct sockaddr_in inaddr;
       struct sockaddr addr;
    } a;
    socklen_t addrlen = sizeof(a.inaddr);

    /*********************
	** Listener
	**********************/
    socket_descriptor listen_socket = INVALID_SOCKET;
    listen_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listen_socket == INVALID_SOCKET) {
		return SocketError(ConfigurationError);
	}

    // allow it to be bound to an address already in use - do we actually need this?
    int reuse = 1;
    if (setsockopt(listen_socket, SOL_SOCKET, SO_REUSEADDR, (char*) &reuse, (socklen_t) sizeof(reuse)) == SOCKET_ERROR ) {
    	::closesocket(listen_socket);
		return SocketError(ConfigurationError);
    }

    memset(&a, 0, sizeof(a));
    a.inaddr.sin_family = AF_INET;
    a.inaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    // For TCP/IP, if the port is specified as zero, the service provider assigns
    // a unique port to the application from the dynamic client port range.
    a.inaddr.sin_port = 0;

    if  (bind(listen_socket, &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return SocketError(ConfigurationError);
    }
    // we need this below because the system auto filled in some entries, e.g. port #
    if  (getsockname(listen_socket, &a.addr, &addrlen) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return SocketError(ConfigurationError);
    }
    // max 1 connection permitted
    if (listen(listen_socket, 1) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
		return SocketError(ConfigurationError);
    }
    /*********************
	** Connection
	**********************/
    // initialise
    socket_fd_pair[0] = socket_fd_pair[1] = INVALID_SOCKET;
    // create first socket and connect to the listener
    // DWORD flags = (make_overlapped ? WSA_FLAG_OVERLAPPED : 0);
    DWORD overlapped_flag = 0;
    socket_fd_pair[0] = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, overlapped_flag);
    if (socket_fd_pair[0] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(socket_fd_pair[0]);
		return SocketError(ConfigurationError);
    }
    // reusing the information from above to connect to the listener
    if (connect(socket_fd_pair[0], &a.addr, sizeof(a.inaddr)) == SOCKET_ERROR) {
    	::closesocket(listen_socket);
    	::closesocket(socket_fd_pair[0]);
		return SocketError(ConfigurationError);
    }
    /*********************
	** Accept
	**********************/
    socket_fd_pair[1] = accept(listen_socket, NULL, NULL);
    if (socket_fd_pair[1] == INVALID_SOCKET) {
    	::closesocket(listen_socket);
    	::closesocket(socket_fd_pair[0]);
    	::closesocket(socket_fd_pair[1]);
		return SocketError(ConfigurationError);
    }
	/*********************
	** Nonblocking
	**********************/
    // should we do this or should we set io overlapping?
	unsigned long non_blocking_flag = 0; // by default is blocking.
	if ( non_blocking ) {
		non_blocking_flag = 1;
		if(ioctlsocket( socket_fd_pair[0], FIONBIO, &non_blocking_flag ) != 0 ) {
			::closesocket(listen_socket);
			::closesocket(socket_fd_pair[0]);
			::closesocket(socket_fd_pair[1]);
			return SocketError(ConfigurationError);
		}
		if(ioctlsocket( socket_fd_pair[1], FIONBIO, &non_blocking_flag ) != 0 ) {
			::closesocket(listen_socket);
			::closesocket(socket_fd_pair[0]);
			::closesocket(socket_fd_pair[1]);
			return SocketError(ConfigurationError);
		}
	}
	/*********************
	** Cleanup
	**********************/
    // the listener has done its job.
    ::closesocket(listen_socket);

#else
	// returns 0 on success, -1 and errno otherwise
	int result = 0;
	if ( non_blocking ) {
#if defined(SOCK_NONBLOCK)
		result = ::socketpair(AF_LOCAL, SOCK_STREAM|SOCK_NONBLOCK, 0, socket_fd_pair);
#else
		result = ::socketpair(AF_LOCAL, SOCK_STREAM, 0, socket_fd_pair);
                // TODO: SOCK_NONBLOCK is n/a at least on macosx -> figure out how to make it non-blocking
#endif
	} else {
		::socketpair(AF_LOCAL, SOCK_STREAM, 0, socket_fd_pair);
	}
	if (result < 0) {
		switch(errno) {
			case (EAFNOSUPPORT) : { return SocketError(ArgNotSupportedError); } // AF_LOCAL is not supported
			case (EFAULT) : { return SocketError(MemoryError); }
			case (EMFILE) : { return SocketError(OutOfResourcesError); }
			case (ENFILE) : { return SocketError(OutOfResourcesError); }
			case (EOPNOTSUPP) : { return SocketError(ArgNotSupportedError); }
			case (EPROTONOSUPPORT) : { return SocketError(ArgNotSupportedError); }
			default : { return SocketError(UnknownError); }
		}
	}
#endif
	return SocketError(NoError);
}

} // namespace ecl




/*****************************************************************************
** Graveyard
*****************************************************************************/

//	/*********************
//	** Notes
//	**********************/
//	// Using port 21 - is this a problem always using this port?
//
//	/******************************************
//	** Server
//	*******************************************/
//	struct addrinfo *result = NULL, *ptr = NULL, hints;
//
//	// windows sockets can't do AF_LOCAL
//	ZeroMemory(&hints, sizeof (hints));
//	hints.ai_family = AF_INET;
//	hints.ai_socktype = SOCK_STREAM;
//	hints.ai_protocol = IPPROTO_TCP;
//	// AI_PASSIVE flag indicates the caller intends to use the
//	// returned socket address structure in a call to the bind function.
//	// When the AI_PASSIVE flag is set and nodename parameter to the
//	// getaddrinfo function is a NULL pointer, the IP address portion
//	// of the socket address structure is set to INADDR_ANY for IPv4
//	// addresses or IN6ADDR_ANY_INIT for IPv6 addresses.
//	hints.ai_flags = AI_PASSIVE;
//
//	// Resolve the local address and port to be used by the server
//	// first argument is the nodename parameter as described above.
//	int wsock_error = getaddrinfo(NULL, "21", &hints, &result);
//	if (wsock_error != 0) {
//		return SocketError(ConfigurationError);
//	}
//
//	/*********************
//	** Server Socket
//	**********************/
//	socket_descriptor listen_socket = INVALID_SOCKET;
//	// Create a SOCKET for the server to listen for client connections
//	listen_socket = ::socket(result->ai_family, result->ai_socktype, result->ai_protocol);
//	if (listen_socket == INVALID_SOCKET) {
//	    freeaddrinfo(result);
//		return SocketError(ConfigurationError);
//	}
//
//	/*********************
//	** Bind Server
//	**********************/
//	// Bind server to an address.
//	wsock_error = bind( listen_socket, result->ai_addr, (int)result->ai_addrlen);
//	if (wsock_error == SOCKET_ERROR) {
//		freeaddrinfo(result);
//		::closesocket(listen_socket);
//		return SocketError(ConfigurationError);
//	}
//	freeaddrinfo(result);
//
//	/*********************
//	** Listen for client
//	**********************/
//	if ( listen( listen_socket, SOMAXCONN ) == SOCKET_ERROR ) {
//	    closesocket(listen_socket);
//		return SocketError(ConfigurationError);
//	}
//	socket_fd_pair[0] = INVALID_SOCKET;
//
//	/******************************************
//	** Client
//	*******************************************/
//	ZeroMemory( &hints, sizeof(hints) );
//	hints.ai_family = AF_UNSPEC;
//	hints.ai_socktype = SOCK_STREAM;
//	hints.ai_protocol = IPPROTO_TCP;
//	// Resolve the server address and port
//	wsock_error = getaddrinfo("localhost", "21", &hints, &result);
//	if (wsock_error != 0) {
//	    ::closesocket(listen_socket);
//		return SocketError(ConfigurationError);
//	}
//	socket_fd_pair[1] = INVALID_SOCKET;
//
//	/*********************
//	** Create client
//	**********************/
//	ptr=result;
//	socket_fd_pair[1] = ::socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
//	if (socket_fd_pair[1] == INVALID_SOCKET) {
//	    ::closesocket(listen_socket);
//	    freeaddrinfo(result);
//		return SocketError(ConfigurationError);
//	}
//
//	/*********************
//	** Connect to server
//	**********************/
//	wsock_error = connect( socket_fd_pair[1], ptr->ai_addr, (int)ptr->ai_addrlen);
//	if (wsock_error == SOCKET_ERROR) {
//	    ::closesocket(listen_socket);
//	    closesocket(socket_fd_pair[1]);
//	    socket_fd_pair[1] = INVALID_SOCKET;
//		return SocketError(ConfigurationError);
//	}
//
//	// Should really try the next address returned by getaddrinfo
//	// if the connect call failed
//	// But for this simple example we just free the resources
//	// returned by getaddrinfo and print an error message
//
//	freeaddrinfo(result);
//
//	if (socket_fd_pair[1] == INVALID_SOCKET) {
//		return SocketError(ConfigurationError);
//	}
//
//	/*********************
//	** Accept connection
//	**********************/
//	// Server accepts a client socket
//	socket_fd_pair[0] = accept(listen_socket, NULL, NULL);
//	if (socket_fd_pair[0] == INVALID_SOCKET) {
//	    ::closesocket(listen_socket);
//	    ::closesocket(socket_fd_pair[1]);
//		return SocketError(ConfigurationError);
//	}
//
//
//	/*********************
//	** Nonblocking
//	**********************/
//	unsigned long non_blocking_flag = 0; // by default is blocking.
//	if ( non_blocking ) {
//		non_blocking_flag = 1;
//		if(ioctlsocket( socket_fd_pair[0], FIONBIO, &non_blocking_flag ) != 0 ) {
//		    ::closesocket(listen_socket);
//		    ::closesocket(socket_fd_pair[0]);
//		    ::closesocket(socket_fd_pair[1]);
//			return SocketError(ConfigurationError);
//		}
//		if(ioctlsocket( socket_fd_pair[1], FIONBIO, &non_blocking_flag ) != 0 ) {
//		    ::closesocket(listen_socket);
//		    ::closesocket(socket_fd_pair[0]);
//		    ::closesocket(socket_fd_pair[1]);
//			return SocketError(ConfigurationError);
//		}
//	}
