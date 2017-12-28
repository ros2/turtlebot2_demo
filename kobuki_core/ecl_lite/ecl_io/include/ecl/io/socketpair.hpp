/**
 * @file /ecl_io/include/ecl/io/socketpair.hpp
 *
 * @brief Extends socketpair functionality to windows.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IO_SOCKETPAIR_HPP_
#define ECL_IO_SOCKETPAIR_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/errors/handlers.hpp>
#include "sockets.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
/**
 * @brief Creates a socket pair internal to the current process.
 *
 * Valid return values are quite simplified. On posix they'll return a variety of
 * arguments similar to the posix return values, on windows it will only
 * return ConfigurationError if something went wrong. This can be expanded
 * in further detail if necessary.
 *
 * @param socket_fd_pair : pair of file descriptors for the sockets to be opened.
 * @param non_blocking : set the pair to be non-blocking(true) or blocking (false).
 *
 * @return SocketError : error return value.
 */
ecl_io_PUBLIC SocketError socketpair(socket_descriptor socket_fd_pair[2], const bool non_blocking = false );


} // namespace ecl

#endif /* ECL_IO_SOCKETPAIR_HPP_ */
