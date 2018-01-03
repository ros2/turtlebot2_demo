/**
 * @file /include/ecl/ipc.hpp
 *
 * @brief Cross-platform inter-process communication mechanisms.
 *
 * @date May, 2009
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_IPC_HPP_
#define ECL_IPC_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifdef emit
    #undef emit
    #define replace_qt_emit
#endif

#include "ipc/semaphore.hpp"
#include "ipc/shared_memory.hpp"

#ifdef replace_qt_emit
    #define emit
#endif

#endif /*ECL_IPC_HPP_*/
