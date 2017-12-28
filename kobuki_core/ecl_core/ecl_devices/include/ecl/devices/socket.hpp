/**
 * @file /include/ecl/devices/socket.hpp
 *
 * @brief Cross platform server tcp/ip socket header.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_SOCKET_HPP_
#define ECL_DEVICES_SOCKET_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include "socket_connection_status.hpp"

#if defined(ECL_IS_POSIX)
  #ifndef ECL_IS_APPLE
    #include "socket_client_pos.hpp"
    #include "socket_server_pos.hpp"
  #endif
#endif

#endif /* ECL_DEVICES_SOCKET_SERVER_HPP_ */
