/**
 * @file /ecl_io/include/ecl/io/sockets.hpp
 *
 * @brief Cross platform socket api.
 *
 * Winsock references:
 *
 * Getting Started
 * - http://msdn.microsoft.com/en-us/library/ms738545%28v=vs.85%29.aspx
 * Error Codes
 * - http://msdn.microsoft.com/en-us/library/ms740668%28VS.85%29.aspx
 *
 * @date February 2011.
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_IO_SOCKETS_HPP_
#define ECL_IO_SOCKETS_HPP_

/*****************************************************************************
 ** Include
 *****************************************************************************/

#include <string>
#include <ecl/config/ecl.hpp>
#include <ecl/errors/handlers.hpp>

#ifdef ECL_IS_WIN32
  #include <ecl/config/windows.hpp>
  #include <winsock2.h>
  #include <ws2tcpip.h>
  //  #include <iphlpapi.h>
  //  #include <sys/types.h>
#elif defined(ECL_IS_POSIX)
  #include <netinet/in.h> // provides AF_LOCAL etc...
  #include <errno.h>
#else
  #error("There is not a supporting sockets implementation on this platform (possibly needs extended ecl support).")
#endif
#include "macros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

/*****************************************************************************
 ** Types
 *****************************************************************************/

#ifdef ECL_IS_WIN32
  typedef SOCKET socket_descriptor; /**< @brief Cross-platform typedef for a socket file descriptor. **/
#else
  typedef int socket_descriptor; /**< @brief Cross-platform typedef for a socket file descriptor. **/
#endif

/*****************************************************************************
 ** Errors
 *****************************************************************************/

/**
 * @brief Extends the generic error handler with socket specific error strings.
 */
class ecl_io_PUBLIC SocketError : public Error
{
public:
  /**
   * @brief Configures the error class with the specified error flag.
   *
   * @param flag : the error type.
   */
  SocketError(const ErrorFlag& flag = UnknownError) : Error(flag)
  {}
protected:
  virtual const char* invalidArgErrorString() const
  { return "One of the arguments is invalid (usually a socket descriptor).";}
#ifdef ECL_IS_WIN32
  virtual const char* notSupportedError() const
  { return "This version of winsock is not supported on this platform.";}
  virtual const char* interruptedErrorString() const
  { return "Interrupted by WSACancelBlockingCall.";}
  virtual const char* notInitialisedErrorString() const
  { return "The underlying winsock subsystem is not initialised (needs WSAStartup).";}
  virtual const char* blockingErrorString() const
  { return "Marked as non-blocking, but current configuration would block.";}
  virtual const char* busyErrorString() const
  { return "A blocking winsock operation is in progress.";}
  virtual const char* systemFailureErrorString() const
  { return "The network subsystem has failed.";}
#elif defined(ECL_IS_POSIX)
  virtual const char* interruptedErrorString() const
  { return "Interrupted by a signal or an io error.";}
  virtual const char* argNotSupportedString() const
  { return "The specified address family or protocol is not supported on this machine.";}
#endif
};

/*****************************************************************************
 ** Functions
 *****************************************************************************/

/**
 * @brief Initialise the socket subsystem.
 *
 * Usually only needed on windows.
 *
 * Valid return values:
 *
 * - NoError,
 * - NotInitialisedError (win),
 * - NotSupportedError (win),
 * - BusyError (win),
 * - OutOfResourcesError (win),
 * - InvalidArgError (win).
 *
 * @return SocketError : error return value.
 */
ecl_io_PUBLIC SocketError init_sockets();
/**
 * @brief Close a socket.
 *
 * Cross platform api for closing a socket.
 *
 * Valid return values:
 *
 * - NoError,
 * - InvalidArgError,
 * - InterruptedError,
 * - NotInitialisedError (win),
 * - SystemFailureError (win),
 * - BlockingError (win).
 *
 * @return SocketError : error return value.
 */
ecl_io_PUBLIC SocketError close_socket(const socket_descriptor& sock);
/**
 * @brief Shutdown the socket subsystem.
 *
 * Usually only needed on windows.
 *
 * Valid return values:
 *
 * - NoError,
 * - NotInitialisedError (win),
 * - SystemFailureError (win),
 * - BusyError (win).
 *
 * @return SocketError : error return value.
 */
ecl_io_PUBLIC SocketError shutdown_sockets();

} // namespace ecl

#endif /* ECL_IO_SOCKETS_HPP_ */
