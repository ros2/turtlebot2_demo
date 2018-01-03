/**
 * @file /ecl_mobile_robot/include/ecl/mobile_robot/differential_drive.hpp
 *
 * @brief Kinematics equations for differential drive type bases.
 *
 * @date 20/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_HPP_
#define ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "kinematics/differential_drive.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface
*****************************************************************************/

/**
 * @brief Differential drive properties and algorithms.
 *
 * This class typedefs various algorithm groups as well as providing
 * some generic structures/methods for differential drive robot types.
 */
class ecl_mobile_robot_PUBLIC DifferentialDrive {
public:
	typedef mobile_robot::DifferentialDriveKinematics Kinematics;
};

} // namespace ecl

#endif /* ECL_MOBILE_ROBOT_DIFFERENTIAL_DRIVE_HPP_ */
