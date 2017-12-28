/**
 * @file /include/ecl/devices/serial.hpp
 *
 * @brief Serial (RS232) interfaces.
 *
 * @date September 2009
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_SERIAL_HPP_
#define ECL_DEVICES_SERIAL_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include "serial_parameters.hpp"
#if defined(ECL_IS_POSIX)
  #include "serial_pos.hpp"
#elif defined(ECL_IS_WIN32)
  #include "serial_w32.hpp"
#endif

#endif /* ECL_DEVICES_SERIAL_HPP_ */
