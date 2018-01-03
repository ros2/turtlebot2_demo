/**
 * @file /include/ecl/config.hpp
 *
 * @brief Compiler and platform specific configurations.
 *
 * Collates all compiler and type configurations. Most of the
 * implementation involves usage of macros and typedefs to manage
 * the code in a standard way across platforms.
 *
 * @date April 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_HPP_
#define ECL_CONFIG_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#include "config/macros.hpp"
#include "config/portable_types.hpp"
#include "config/endianness.hpp"
#include "config/char_sign.hpp"

/*****************************************************************************
 * Includes
 ****************************************************************************/

#if defined(ECL_IS_WIN32)
	#include "config/windows.hpp"
#elif defined(ECL_IS_POSIX)
    #include <unistd.h>
#endif


#endif /*ECL_CONFIG_HPP_*/
