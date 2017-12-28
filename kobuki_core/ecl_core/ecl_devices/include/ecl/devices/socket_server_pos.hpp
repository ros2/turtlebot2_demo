/**
 * @file /include/ecl/devices/socket_server_pos.hpp
 *
 * @brief Posix interface for a simple tcp/ip server.
 *
 * Posix interface for a simple tcp/ip server.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_SOCKET_SERVER_POS_HPP_
#define ECL_DEVICES_SOCKET_SERVER_POS_HPP_

/*****************************************************************************
** Cross platform
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_IS_APPLE
#ifdef ECL_IS_POSIX

/*****************************************************************************
** Includes
*****************************************************************************/

#include <arpa/inet.h> // inet_addr()
#include <sys/ioctl.h> // used in remaining()
#include <errno.h>
#include <sys/socket.h>

#include "detail/socket_error_handler_pos.hpp"
#include "detail/socket_exception_handler_pos.hpp"
#include "traits.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [SocketServer]
*****************************************************************************/
/**
 * @brief Simple implementation of an ipv4 socket server.
 *
 * This is certainly not industrial strength - use a proper library for that.
 * Rather, this is designed to merely fill a simple need, such as that of
 * building a cross-platform remote debugging program.
 *
 * Usage is simple, simply specify the port number when constructing, the
 * rest of the configuration happens under the hood. From there, the usual
 * read/write pair work, or alternatively a stream can be hooked up to it.
 *
 * @sa @ref SocketClient "SocketClient".
 */
class SocketServer {
public:
	/*********************
	** C&D
	**********************/
	SocketServer() : is_open(false) {}; /**< @brief Default constructor, use with open(). **/
	/**
	 * @brief Automatically configures, opens and begins listening on the specified port.
	 *
	 * This is the RAII constructor which does all the configuration and setup for the
	 * port. Once done, it has already started its job and is listening on the specified
	 * port number.
	 *
	 * @param port_number : port on which to listen for connections.
	 * @exception StandardException : throws if the connection failed to open.
	 **/
	SocketServer(const unsigned int &port_number) ecl_throw_decl(StandardException);
	virtual ~SocketServer() { close(); }; /**< @brief If connected, close down the connection to the client. **/

	/*********************
	** Configuration
	**********************/
	/**
	 * @brief Opens the connection.
	 *
	 * Configures and opens the socket with the specified
	 * parameters. You don't need to do this if you constructed this
	 * class with the RAII style constructor.
	 *
	 * @param port_number : port on which to listen for connections.
	 * @exception StandardException : throws if the connection failed to open.
	 **/
	bool open( const unsigned int& port_number ) ecl_throw_decl(StandardException);
	/**
	 * @brief If connected to a client, terminate the connection.
	 *
	 * There's a peculiarity which
	 * often pops up with these - even after closing, you will still
	 * get an ECONNREFUSED if your client tries immediately to connect
	 * again. This is because even though the interface gets closed, it
	 * may be there are packets out there floating around on the net
	 * still waiting to be delivered, so the kernel generally
	 * keeps it alive for a few minutes after you close it. From the
	 * c++ newsgroups:
	 *
	 * It is normal, for example for the socket to go into a TIME_WAIT state,
	 * on the server side, for a few minutes.  People have reported ranges
	 * from 20 seconds to 4 minutes to me.  The official standard says
	 * that it should be 4 minutes.
	 */
	void close() { is_open = false; ::close(socket_fd); }

	bool open() const { return is_open; }

	/*********************
	** Writing
	**********************/
	/**
	 * @brief Write a character to the serial port.
	 *
	 * Write a character to the serial port.
	 *
	 * @param c : the character to write.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char &c) ecl_debug_throw_decl(StandardException) { return write(&c,1); }

	/**
	 * @brief Write a character string to the serial port.
	 *
	 * Write a character string to the serial port.
	 * @param s : points to the beginning of the character string.
	 * @param n : the number of characters to write.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char *s, unsigned long n) ecl_debug_throw_decl(StandardException);

	/**
	 * @brief A dummy flush function, not used, but needed by streams.
	 *
	 * This is unused - but is included so as to implement streaming functionality
	 * on top of this device.
	 */
	void flush() {}

	/*********************
	** Reading
	**********************/
	/**
	 * @brief Get the number of bytes remaining in the buffer, but do not read.
	 *
	 * Check the serial port's buffer to determine how many bytes are left in the buffer for reading.
	 *
	 * @return int : the number of bytes in the buffer
	 **/
	long remaining();
	/**
	 * @brief Read a character from the port.
	 *
	 * Note that by
	 * default, posix socket connections will wait if there is nothing to read
	 * and as soon as something arrives, it will return, regardless of whether
	 * n requested bytes have arrived or not. It is possible to implement non-blocking
	 * behaviour, but that will use valuable cpu cycles. Better to just let
	 * posix sleep the system inbetween arrivals and deal with them as they come
	 * in (possibly implementing your own buffer which is fed from posix calls
	 * made by this class is a good idea).
	 *
	 * This will return @ref ConnectionHungUp "ConnectionHungup" if
	 * it returns with zero bytes (it's blocked and waiting by default, see above).
	 *
	 * @param c : character to read into from the serial port's buffer.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char &c) ecl_debug_throw_decl(StandardException) { return read(&c,1); }
	/**
	 * @brief Read a character string from the port.
	 *
	 * Simply passes along the read work to its big brother.
	 *
	 * @param s : character string to read into from the serial port's buffer.
	 * @param n : the number of bytes to read.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char *s, const unsigned long &n) ecl_debug_throw_decl(StandardException);
    /**
     * @brief Reads at the incoming socket buffer, but doesn't remove from the buffer.
     *
     * Reads, but doesn't actually read the contents of the incoming buffer.
     *
     * @param s : variable to store the characters in.
     * @param n : the desired number of characters to read.
     * @return long : number of characters read (-1 implies an error).
	 * @exception StandardException : throws if reading returned an error [debug mode only].
     **/
    long peek(char*s, const unsigned long &n) ecl_debug_throw_decl(StandardException);

    /*********************
	** Socket Specific
	**********************/
    /**
     * @brief Listens for incoming connections.
     *
     * Listen for a client connection and return the handle to the file descriptor.
     *
     * @return int : client socket descriptor.
	 * @exception StandardException : throws if the server could not begin listening.
     **/
    int listen() ecl_throw_decl(StandardException);

	/**
	 * @brief Reports on the error state of the last operation.
	 */
	const Error& error() const { return error_handler; }

private:
    int port;
    int socket_fd;
    int client_socket_fd;
    bool is_open;
    Error error_handler;
};

/*****************************************************************************
** Traits [Serial]
*****************************************************************************/
/**
 * @brief Serial sink (output device) trait.
 *
 * Specialisation for the serial sink (output device) trait.
 */
template <>
class is_sink<SocketServer> : public True {};

/**
 * @brief Serial sink (input device) trait.
 *
 * Specialisation for the serial sink (input device) trait.
 */
template <>
class is_source<SocketServer> : public True {};

/**
 * @brief Serial sourcesink (input-output device) trait.
 *
 * Specialisation for the serial sourcesink (input-output device) trait.
 */
template <>
class is_sourcesink<SocketServer> : public True {};

} // namespace ecl

#endif  /* ECL_IS_POSIX */
#endif  /* !ECL_IS_APPLE */

#endif /* ECL_DEVICES_SOCKET_SERVER_POS_HPP_ */
