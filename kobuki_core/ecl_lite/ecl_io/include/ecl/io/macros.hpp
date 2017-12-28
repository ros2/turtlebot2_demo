/**
 * @file /include/ecl/io/macros.hpp
 *
 * @brief Macros for windows dlls.
 *
 * @date March 2013.
 **//*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IO_MACROS_HPP_
#define ECL_IO_MACROS_HPP_

/*****************************************************************************
** PreProcessing
*****************************************************************************/
/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef ecl_io_EXPORTS // we are building a shared lib/dll
    #define ecl_io_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_io_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_io_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_io_PUBLIC
  #define ecl_io_LOCAL
#endif


#endif /* ECL_IO_MACROS_HPP_*/
