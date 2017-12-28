/**
 * @file /src/lib/realpath.cpp
 *
 * @brief Realpath implementations.
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/filesystem/config.hpp>

#if defined(ECL_PRIVATE_HAS_POSIX_REALPATH)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <errno.h>
#include <climits>  // limits.h
#include <cstdlib>  // stdlib.h
#include <string>
#include <ecl/errors/handlers.hpp>
#include "../../include/ecl/filesystem/realpath.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [realpath]
*****************************************************************************/

ecl_filesystem_PUBLIC ecl::Error realpath(const std::string& path, std::string& absolute_path) {
  int path_max;
  // see the man page for realpath for details
  #ifdef PATH_MAX
    path_max = PATH_MAX; /* PATH_MAX from limits.h, but not always defined */
  #else
    path_max = pathconf(path, _PC_PATH_MAX);
    if (path_max <= 0) { path_max = 4096; } /* Not guaranteed to give you results */
  #endif

  char buffer[path_max];
  char *result = ::realpath(path.c_str(), buffer);
  absolute_path = buffer;
  if ( result != NULL ) {
    absolute_path = buffer;
  } else {
    switch(errno) {
      case(EACCES)       : { return Error(PermissionsError); }   // Read or search permission was denied for a component of the path prefix.
      case(EINVAL)       : { return Error(InvalidArgError); }    // path is NULL
      case(EIO)          : { return Error(ReadError); }          // An I/O error occurred while reading from the filesystem.
      case(ELOOP)        : { return Error(InvalidObjectError); } // too many symbolic links were encountered in translating the pathname
      case(ENAMETOOLONG) : { return Error(InvalidArgError); }    // A component of a pathname exceeded NAME_MAX characters, or an entire pathname exceeded PATH_MAX characters.
      case(ENOENT)       : { return Error(NotFoundError); }      // The named file does not exist
      case(ENOTDIR)      : { return Error(InvalidObjectError); } // A component of the path prefix is not a directory.
      default            : { return Error(UnknownError); }
    }
  }
  return Error(NoError);
}

}; // namespace ecl

#endif /* ECL_PRIVATE_HAS_POSIX_REALPATH */

