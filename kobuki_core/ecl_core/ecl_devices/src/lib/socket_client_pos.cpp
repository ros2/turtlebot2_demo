/**
 * @file /src/lib/socket_client_pos.cpp
 *
 * @brief Posix implementation for tcp/ip clients.
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
#include <iostream>
#include <netdb.h> // gethostbyname
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/devices/detail/socket_error_handler_pos.hpp"
#include "../../include/ecl/devices/detail/socket_exception_handler_pos.hpp"
#include "../../include/ecl/devices/socket_connection_status.hpp"
#include "../../include/ecl/devices/socket_client_pos.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;

/*****************************************************************************
** Implementation
*****************************************************************************/

SocketClient::SocketClient(const std::string &host_name, const unsigned int &port_number) ecl_throw_decl(StandardException) :
		hostname(host_name),
		port(port_number),
		is_open(false),
		error_handler(NoError)
{
    ecl_try {
        open(host_name, port_number);
    } ecl_catch ( StandardException &e ) {
        ecl_throw(StandardException(LOC,e));
    }
}

void SocketClient::close() {
	 is_open = false;
	 ::close(socket_fd); // should have error handling here
}

bool SocketClient::open( const std::string &host_name, const unsigned int& port_number ) ecl_throw_decl(StandardException) {

	if ( this->open() ) { this->close(); }
	hostname = host_name;
    port = port_number;

	/*********************
	 * Open
	**********************
     * PF_INET (IP4), PF_LOCAL (LOCALHOST)
     * SOCK_STREAM (TCPIP), SOCK_DGRAM (UDP), SOCK_RAW
     * Last argument is generally always 0 (sub-type)
     */
    // maybe hostname == localhost or 127.0.0.1 -> use PF_LOCAL?
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
//    socket_fd = socket(PF_INET, SOCK_STREAM, 0);
    if ( socket_fd == -1 ) {
    	ecl_throw(devices::socket_exception(LOC));
    	error_handler = devices::socket_error();
    	return false;
    }
    /*********************
	** Hostname
	**********************/
    struct hostent *host_entry;
    /******************************************
     * struct hostent {
     *   char  *h_name;            //official name of host
     *   char **h_aliases;         // array of aliases (terminated by NULL)
     *   int    h_addrtype;        // host address type
     *   int    h_length;          // length of address
     *   char **h_addr_list;       // array of addresses (terminated by NULL)
     * }
     ******************************************/
    host_entry = gethostbyname(hostname.c_str());
    if ( host_entry == NULL ) { // gethostbyname doesn't seem to be checking hostname validity???
    	::close(socket_fd);
    	ecl_throw(devices::gethostbyname_exception(LOC,hostname));
    	error_handler = devices::gethostbyname_error();
    	return false;
    }

    /*********************
	** Socket Details
	**********************/
    struct sockaddr_in destination;
    destination.sin_family = AF_INET;    // host byte order
    destination.sin_addr = *((struct in_addr *)host_entry->h_addr);
    destination.sin_port = htons(port);  // short host byte order to network byte order
    memset(destination.sin_zero, '\0', sizeof destination.sin_zero);

    /*********************
	** Connect
	**********************/
    int connect_result = connect( socket_fd, (struct sockaddr *) &destination, sizeof(destination) );
    if ( connect_result == -1 ) {
    	is_open = false;
    	::close(socket_fd);
    	ecl_throw(devices::connection_exception(LOC));
    	error_handler = devices::connection_error();
    	return false;
    }
    is_open = true;
    error_handler = NoError;
    return true;
}

/*****************************************************************************
** Implementation [SocketClient][Source]
*****************************************************************************/

long SocketClient::read(char &c) ecl_assert_throw_decl(StandardException) {
    return read(&c,1);
}

long SocketClient::read(char *s, const unsigned long &n) ecl_assert_throw_decl(StandardException) {

    if ( !open() ) {
    	return ConnectionDisconnected;
    }

    /*********************
     * Check Status
     *********************/
//    SocketStatus status = pollStatus(POLLIN); // Times out according to timeout_ms if it can't read in the specified time.
//    std::cout << "Polling Status " << status << std::endl;
//    if ( status != SocketReadReady ) { return status; } // else it is ok to write.

    int bytes_read = ::recv(socket_fd, s, n, 0); // Consider using MSG_WAITALL

    /*********************
	** Error Handling
	**********************/
    if ( bytes_read == 0 ) {
        // Server has dropped
        close();
        return ConnectionHungUp;
    }
    if ( bytes_read < 0 ) {
    	if ( errno == ECONNRESET ) {
    		close();
    		return ConnectionHungUp;
    	} else {
			ecl_debug_throw( devices::receive_exception(LOC) );
			error_handler = devices::receive_error();
			return ConnectionProblem;
    	}
    }
    error_handler = NoError;
    return bytes_read;
}

long SocketClient::peek(char *s, const unsigned long &n) ecl_assert_throw_decl(StandardException) {

    if ( !open() ) { return ConnectionDisconnected; }

	int bytes_read = ::recv(socket_fd, s, n, MSG_PEEK);
    if ( bytes_read < 0 ) {
    	ecl_debug_throw( devices::receive_exception(LOC) );
    	error_handler = devices::receive_error();
    	return ConnectionProblem;
    }
    error_handler = NoError;
    return bytes_read;
};

long SocketClient::remaining() {

    if ( !open() ) { return ConnectionDisconnected; }

    unsigned long bytes;
    int result = ioctl(socket_fd, FIONREAD, &bytes);
    if ( result == -1 ) {
    	ecl_debug_throw( devices::ioctl_exception(LOC) );
    	error_handler =  devices::ioctl_error();
    }
    error_handler = NoError;
    return bytes;
};

/*****************************************************************************
** Implementation [SocketClient][Sink]
*****************************************************************************/

long SocketClient::write(const char &c) ecl_assert_throw_decl(StandardException) {

	return write(&c, 1);
}

long SocketClient::write(const char *s, unsigned long n) ecl_assert_throw_decl(StandardException) {

	if ( !open() ) { return ConnectionDisconnected; }

    /*********************
     * Write
     *********************/
    int bytes_written = ::send(socket_fd,s,n,MSG_NOSIGNAL);

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
    return bytes_written;
}

} // namespace ecl

#endif /* ECL_IS_POSIX */
#endif  /* !ECL_IS_APPLE */
