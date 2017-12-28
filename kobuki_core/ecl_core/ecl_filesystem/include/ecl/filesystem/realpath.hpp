/**
 * @file /include/ecl/filesystem/functions_pos.hpp
 *
 * @brief Simple posix implementation of ecl time functions.
 *
 * This implementation has its problems - most notably that you cannot get
 * monotonic functionality.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_FILESYSTEM_REALPATH_HPP_
#define ECL_FILESYSTEM_REALPATH_HPP_

/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/filesystem/config.hpp>
#include "macros.hpp"

#if defined(ECL_PRIVATE_HAS_POSIX_REALPATH)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ecl/errors/handlers.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementations
*****************************************************************************/
/**
 * @brief Convert a path with relative markers to absolute.
 *
 * Expands all symbolic links and resolves references to /./, /../ and extra '/'
 * characters to produce a canonicalized absolute pathname.
 *
 * Valid return values:
 *
 * - InvalidArgError,
 * - ReadError,
 * - InvalidObjectError,
 * - InvalidArgError,
 * - NotFoundError,
 * - InvalidObjectError,
 *
 * @param path : relative or absolute path
 * @return string : path with all ./ and ../ references replaced with the full path.
 */
ecl_filesystem_PUBLIC  ecl::Error realpath(const std::string& path, std::string& absolute_path);


} // namespace ecl

#endif /* ECL_PRIVATE_HAS_POSIX_REALPATH */
#endif /* ECL_FILESYSTEM_REALPATH_HPP_ */
