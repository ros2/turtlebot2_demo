/**
 * @file /include/ecl/devices/ofile.hpp
 *
 * @brief Synchronous and asynchronous file io.
 *
 * @date September 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_OFILE_HPP_
#define ECL_DEVICES_OFILE_HPP_

/*************************************************************************
 * Includes
 ************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#if defined(ECL_IS_POSIX)
  #include "ofile_pos.hpp"
#elif defined(ECL_IS_WIN32)
  #include "ofile_w32.hpp"
#endif


#endif /* ECL_DEVICES_OFILE_HPP_ */
