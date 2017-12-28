/**
 * @file /include/ecl/devices/traits.hpp
 *
 * @brief Traits for devices.
 *
 * @date August 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_TRAITS_HPP_
#define ECL_DEVICES_TRAITS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/mpl.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/
/**
 * @brief Default action for detection of source devices (input) (false).
 *
 * Sets the default value (false) for detection of the source device (input)
 * trait. Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_source : public False {};

/**
 * @brief Default action for detection of sink devices (output) (false).
 *
 * Sets the default value (false) for detection of the sink device (output)
 * trait. Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_sink : public False {};

/**
 * @brief Default action for detection of source-sink devices (input-output) (false).
 *
 * Sets the default value (false) for detection of the source-sink device
 * (input-output) trait. Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_sourcesink : public False {};

/**
 * @brief Default action for detection of seekable devices (false).
 *
 * Sets the default value (false) for detection of the seekable device
 * trait. Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_seekable : public False {};

} // namespace ecl

#endif /* ECL_DEVICES_TRAITS_HPP_ */
