/**
 * @file /include/ecl/streams/socket_streams.hpp
 *
 * @brief Convenience handles for socket textstreams.
 *
 * Convenience handles for socket textstreams.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_STREAMS_SOCKET_STREAMS_HPP_
#define ECL_STREAMS_SOCKET_STREAMS_HPP_

#ifdef ECL_IS_POSIX

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/devices/socket.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "text_stream.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces [SocketClientStream]
*****************************************************************************/
/**
 * @brief Convenience class for SocketClient TextStream definitions.
 *
 * This class provides a convenient handle for both writing and
 * opening TextStream<SocketClient> objects. It has no further functionality.
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
class ecl_streams_PUBLIC SocketClientStream : public TextStream<SocketClient> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the device manually via device().open() as you would do
	 * if using a TextStream.
	 */
	SocketClientStream() {};

	/**
	 * @brief Convenience constructor for output file text streams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * device (this makes it distinct from a generic TextStream<OFile> object).
	 *
	 * @param host_name : host to connect to.
	 * @param port_number : port number of the connection.
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	SocketClientStream(const std::string &host_name, const unsigned int &port_number) ecl_throw_decl(StandardException) {
		ecl_try {
			if ( !this->device().open(host_name, port_number) ) {
				error = this->device().error();
			}
		} ecl_catch(StandardException &e) {
			ecl_throw(StandardException(LOC,e));
		}
	}

	virtual ~SocketClientStream() {};
};

/*****************************************************************************
** Interfaces [SocketClientStream]
*****************************************************************************/
/**
 * @brief Convenience class for SocketServer TextStream definitions.
 *
 * This class provides a convenient handle for both writing and
 * opening TextStream<SocketServer> objects. It has no further functionality.
 *
 * @sa @ref ecl::TextStream "TextStream".
 */
class ecl_streams_PUBLIC SocketServerStream : public TextStream<SocketServer> {
public:
	/**
	 * @brief Default constructor, underlying device must be manually opened.
	 *
	 * This must open the device manually via device().open() as you would do
	 * if using a TextStream.
	 */
	SocketServerStream() {};

	/**
	 * @brief Convenience constructor for output file text streams.
	 *
	 * This constructor enables RAII style construction of the underlying
	 * device (this makes it distinct from a generic TextStream<OFile> object).
	 *
	 * @param port_number : port number of the connection.
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	SocketServerStream(const unsigned int &port_number) throw(StandardException) {
		try {
			this->device().open(port_number);
		} catch(StandardException &e) {
			throw StandardException(LOC,e);
		}
	}

	virtual ~SocketServerStream() {};
};

} // namespace ecl

#endif /* ECL_IS_POSIX */

#endif /* ECL_STREAMS_SOCKET_STREAMS_HPP_ */
