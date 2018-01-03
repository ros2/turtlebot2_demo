/**
 * @file /include/ecl/devices/socket_connection_status.hpp
 *
 * @brief Various connection status flags.
 *
 * @date January 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_CONNECTION_STATUS_HPP_
#define ECL_DEVICES_CONNECTION_STATUS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Enums
*****************************************************************************/

/**
 * @brief Used to denote the status of a connection.
 */
enum ConnectionStatus {
	ConnectionProblem = -1, /**< @brief Was some error in the last operation.. **/
	ConnectionDisconnected = -2, /**< @brief Used to state when a server/client detects that there is no other connection at the other end. **/
	ConnectionHungUp = -3 /**< @brief Used to signify when a server/client detects that the other end has 'hung up'. **/
};

} // namespace ecl

#endif /* ECL_DEVICES_CONNECTION_STATUS_HPP_ */
