/**
 * @file /include/ecl/ipc/semaphore.hpp
 *
 * @brief Provides a class for securing shared data between processes.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IPC_SEMAPHORE_HPP_
#define ECL_IPC_SEMAPHORE_HPP_

/*****************************************************************************
** Platform Detection
*****************************************************************************/

#include <ecl/config/ecl.hpp> // ECL_ macros

/*****************************************************************************
** Cross Platform Implementation
*****************************************************************************/

#if defined(ECL_IS_POSIX)
  #ifdef _POSIX_SEMAPHORES
    #if _POSIX_SEMAPHORES > 0
      #include "semaphore_pos.hpp"
    #endif
  #endif
#endif


#endif /*ECL_IPC_SEMAPHORE_HPP_*/
