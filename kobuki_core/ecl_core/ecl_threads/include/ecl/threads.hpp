/**
 * @file /include/ecl/threads.hpp
 *
 * @brief Cross-platform threading interfaces.
 *
 * Threads are modelled here as classes. The threading functionality can be
 * utilised in two ways, either via inheritance or composition. Choice of which
 * is simply a matter of personal style. There are more complex thread managers
 * out there, but this suits the requirements of a small project and merely
 * aims to provide a robust and lightweight cross-platform interface.
 *
 * @date June, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_THREADS_HPP_
#define ECL_THREADS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Includes
*****************************************************************************/

/*
 * Guard against the qt macros.
 */
#ifdef emit
    #undef emit
    #define replace_qt_emit
#endif

//#include "threads/barrier.hpp"
#include "threads/mutex.hpp"
#include "threads/priority.hpp"
#include "threads/thread.hpp"
#include "threads/threadable.hpp"

#ifdef replace_qt_emit
    #define emit
#endif



#endif /* ECL_THREADS_HPP_ */
