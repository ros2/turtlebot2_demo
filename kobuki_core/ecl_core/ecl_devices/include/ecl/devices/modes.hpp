/**
 * @file /include/ecl/devices/modes.hpp
 *
 * @brief Device modes.
 *
 * @date October, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_MODES_HPP_
#define ECL_DEVICES_MODES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

/**
 * @brief Write mode for devices.
 *
 * Defines the write modes for writing to an output device. Primarily used for
 * files.
 **/
enum WriteMode {
    New,             /**< @brief Opens a new object (deletes existing objects). **/
    Append 			/**< @brief Appends to an existing object (opens if not existing). **/
};


} // namespace ecl

#endif /* ECL_DEVICES_MODES_HPP_ */
