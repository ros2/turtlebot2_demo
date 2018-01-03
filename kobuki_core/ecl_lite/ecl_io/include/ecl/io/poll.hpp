/**
 * @file /ecl_io/include/ecl/io/poll.hpp
 *
 * @brief Cross platform api for a polling method.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IO_POLL_HPP_
#define ECL_IO_POLL_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "sockets.hpp"

#ifdef ECL_IS_WIN32
  // nothing special here.
#else
  #include <poll.h>
#endif

/*****************************************************************************
** Macros
*****************************************************************************/

#ifdef ECL_IS_WIN32
  #ifndef POLLRDNORM
    #define POLLRDNORM  0x0100 /* mapped to read fds_set */
  #endif
  #ifndef POLLRDBAND
    #define POLLRDBAND  0x0200 /* mapped to exception fds_set */
  #endif
  #ifndef POLLIN
    #define POLLIN      (POLLRDNORM | POLLRDBAND) /* There is data to read.  */
  #endif
  #ifndef POLLPRI
    #define POLLPRI     0x0400 /* There is urgent data to read.  */
  #endif

  #ifndef POLLWRNORM
    #define POLLWRNORM  0x0010 /* mapped to write fds_set */
  #endif
  #ifndef POLLOUT
    #define POLLOUT     (POLLWRNORM) /* Writing now will not block.  */
  #endif
  #ifndef POLLWRBAND
    #define POLLWRBAND  0x0020 /* mapped to write fds_set */
  #endif
  #ifndef POLLERR
    #define POLLERR     0x0001 /* Error condition.  */
  #endif
  #ifndef POLLHUP
    #define POLLHUP     0x0002 /* Hung up.  */
  #endif
  #ifndef POLLNVAL
    #define POLLNVAL    0x0004 /* Invalid polling request.  */
  #endif
#else
    // Nothing to see here!
#endif

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Types
*****************************************************************************/

#ifdef ECL_IS_WIN32
  /* poll emulation support */
  typedef struct socket_pollfd {
    socket_descriptor fd;      /* file descriptor */
    short events;     /* requested events */
    short revents;    /* returned events */
  } socket_pollfd;

  typedef unsigned long int nfds_t;
#else
  typedef struct pollfd socket_pollfd;
  typedef ::nfds_t ndfs_t;
#endif


  ecl_io_PUBLIC int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout);

} // namespace ecl

#endif /* ECL_IO_POLL_HPP_ */
