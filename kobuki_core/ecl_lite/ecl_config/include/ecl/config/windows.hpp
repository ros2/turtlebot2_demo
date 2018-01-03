/**
 * @file /ecl_config/include/ecl/config/windows.hpp
 *
 * @brief Custom setup for windoze.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_WINDOWS_HPP_
#define ECL_CONFIG_WINDOWS_HPP_

/*****************************************************************************
** Win32
*****************************************************************************/

#include <ecl/config/ecl.hpp>

#if defined(ECL_IS_WIN32)
  #define _WINSOCKAPI_    // stops windows.h including winsock.h (we use winsock2)
  #include <windows.h>
  #ifdef min
    #undef min
  #endif
  #ifdef max
    #undef max
  #endif

  #ifdef _MSC_VER
    #pragma warning(disable: 4251)  // Disable warnings about import/exports when deriving from std classes
    #pragma warning(disable: 4275)  // ""
    #pragma warning (disable:4996)  // Disable warnings about deprecated ctime
    #pragma warning (disable:4290)  // Disable warnings about unsupported c++ exception specifications
  #endif
#endif

#endif /* ECL_CONFIG_WINDOWS_HPP_ */
