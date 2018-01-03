/**
 * @file /include/ecl/sigslots.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Apr 17, 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_HPP_
#define ECL_SIGSLOTS_HPP_

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

#include "sigslots/signal.hpp"
#include "sigslots/slot.hpp"

#ifdef replace_qt_emit
    #define emit
#endif

#endif /* ECL_SIGSLOTS_HPP_ */


