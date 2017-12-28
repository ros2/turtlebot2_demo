/**
 * @file /include/ecl/ipc/shared_memory.hpp
 *
 * @brief Provides a class for sharing data between processes.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IPC_SHARED_MEMORY_HPP_
#define ECL_IPC_SHARED_MEMORY_HPP_

/*****************************************************************************
** Platform Detection
*****************************************************************************/

#include <ecl/config/ecl.hpp> // ECL_ macros

/*****************************************************************************
** Cross Platform Implementation
*****************************************************************************/

#if defined(ECL_IS_POSIX)
  #ifdef _POSIX_SHARED_MEMORY_OBJECTS
    #if _POSIX_SHARED_MEMORY_OBJECTS > 0
      #include "shared_memory_pos.hpp"
    #endif
  #endif
#endif


#endif /*ECL_IPC_SHARED_MEMORY_HPP_*/
