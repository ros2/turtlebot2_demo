/**
 * @file /src/lib/socket_server_pos.cpp
 *
 * @brief Posix implementation for tcp/ip servers.
 *
 * @date January 2009
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

#include <unistd.h>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/devices/socket_connection_status.hpp"
#include "../../include/ecl/devices/socket_server_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [SocketServer]
*****************************************************************************/

SocketServer::SocketServer(const unsigned int &port_number) ecl_throw_decl(StandardException) :
		port(port_number),
		is_open(false),
		error_handler(NoError)
{
    ecl_try {
        open(port_number);
    } ecl_catch ( const StandardException &e ) {
    	ecl_throw(StandardException(LOC,e));
    }
}

bool SocketServer::open( const unsigned int& port_number ) ecl_throw_decl(StandardException) {

    if ( this->open() ) { this->close(); }
    port = port_number;

    /*************************************************************************
     * Open socket
     *************************************************************************
     * PF_INET (IP4), PF_LOCAL (LOCALHOST)
     * SOCK_STREAM (TCPIP), SOCK_DGRAM (UDP), SOCK_RAW
     * Last argument is generally always 0 (sub-type)
     */
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
//    socket_fd = socket(PF_INET, SOCK_STREAM, 0);
    if ( socket_fd == -1 ) {
    	ecl_throw(devices::socket_exception(LOC));
    	error_handler = devices::socket_error();
    	return false;
    }

    /*************************************************************************
     * Configure Socket Details
     ************************************************************************/
    // allow *immediate* socket reuse, http://www.ibm.com/developerworks/library/l-sockpit/
    int ret, on;
    on = 1;
    ret = setsockopt( socket_fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) );
    // other options
    struct sockaddr_in server;
    server.sin_family = AF_INET;    // host byte order
    server.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
    server.sin_port = htons(port);  // short host byte order to network byte order
    memset(server.sin_zero, '\0', sizeof server.sin_zero); // zero the server structure

    /*************************************************************************
     * Bind address to the socket
     ************************************************************************/
    int bind_result = bind(socket_fd, (struct sockaddr *) &server, sizeof(server));
    if ( bind_result == - 1 ) {
    	is_open = true;
    	ecl_throw(devices::bind_exception(LOC));
    	error_handler = devices::bind_error();
    	return false;
    }
    is_open = true;
    error_handler = NoError;
    return true;
}

int SocketServer::listen() ecl_throw_decl(StandardException) {

	/*********************
	** Params
	**********************/
	::listen(socket_fd,1); // Number of clients to allow

	/*********************
	** Listen
	**********************/
    struct sockaddr_in client;
    int client_length = sizeof(client);

    client_socket_fd = accept(socket_fd, (struct sockaddr *) &client, (socklen_t *) &client_length);
    if (client_socket_fd < 0) {
    	ecl_throw(devices::accept_exception(LOC));
    	error_handler =devices::accept_error();
    	return -1;
    }
    error_handler = NoError;
    return client_socket_fd;

}

/*****************************************************************************
** Implementation [SocketServer][Source]
*****************************************************************************/

long SocketServer::read(char *s, const unsigned long &n) ecl_debug_throw_decl(StandardException) {

    if ( !open() ) { return ConnectionDisconnected; }

    int bytes_read = ::recv(client_socket_fd, s, n, 0);
    if ( bytes_read < 0 ) {
    	ecl_debug_throw(devices::receive_exception(LOC));
    	error_handler = devices::receive_error();
    	return ConnectionProblem;
    }

    if ( bytes_read == 0 ) {
        // Server has dropped
        close();
        return ConnectionHungUp;
    }
    error_handler = NoError;
    return bytes_read;
}

long SocketServer::peek(char *s, const unsigned long &n) ecl_debug_throw_decl(StandardException) {

	int bytes_read = ::recv(client_socket_fd, s, n, MSG_PEEK);
    if ( bytes_read < 0 ) {
    	ecl_debug_throw(devices::receive_exception(LOC));
    	error_handler = devices::receive_error();
    	return ConnectionProblem;
    }
    error_handler = NoError;
    return bytes_read;
};

long SocketServer::remaining() {
    unsigned long bytes;
    int result = ioctl(client_socket_fd, FIONREAD, &bytes);
    if ( result == -1 ) {
    	ecl_debug_throw(devices::ioctl_exception(LOC));
    	error_handler = devices::ioctl_error();
    	return ConnectionProblem;
    }
    error_handler = NoError;
    return bytes;
};


/*****************************************************************************
** Implementation [SocketServer][Sink]
*****************************************************************************/

long SocketServer::write(const char *s, unsigned long n) ecl_debug_throw_decl(StandardException) {
    #ifdef MSG_NOSIGNAL
        int bytes_written = ::send(client_socket_fd,s,n,0|MSG_NOSIGNAL);
    #else
        int bytes_written = ::send(client_socket_fd,s,n,0);
    #endif
    if ( bytes_written < 0 ) {
        switch(errno) {
            case ( EPIPE ) : {
                close();
                return ConnectionHungUp;
            }
            default : {
        	    ecl_debug_throw( devices::send_exception(LOC) );
        	    error_handler = devices::send_error();
        	    return ConnectionProblem;
            }
        }
    }
    error_handler = NoError;
    return bytes_written;
}

} // namespace ecl

#endif /* ECL_IS_POSIX */
#endif  /* !ECL_IS_APPLE */
