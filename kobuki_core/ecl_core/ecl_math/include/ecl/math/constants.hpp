/**
 * @file /include/ecl/math/constants.hpp
 *
 * @brief Mathematical constants.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MATH_CONSTANTS_HPP_
#define ECL_MATH_CONSTANTS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

#ifdef M_PI // defined by all POSIX systems and some non-POSIX ones
	double const pi = M_PI; /**< @brief Mathematical constant for pi. */
	const double two_pi  = 2.0*pi;  /**< @brief Mathematical constant for 2*pi. */
	const double pi_2    = pi/2.0;  /**< @brief Mathematical constant for pi/2. */
	const double pi_4    = pi/4.0;  /**< @brief Mathematical constant for pi/4. */
	const float  two_pi_f= two_pi;  /**< @brief Mathematical constant (float format) for 2*pi. */
	const float  pi_f    = pi;  /**< @brief Mathematical constant (float format) for pi. */
	const float  pi_2_f  = pi_2;  /**< @brief Mathematical constant (float format) for pi/2. */
	const float  pi_4_f  = pi_4;  /**< @brief Mathematical constant (float format) for pi/4. */
#else
	double const pi = 4.0*std::atan(1.0); /**< @brief Mathematical constant for pi. */
	const double two_pi  = 2.0*pi;  /**< @brief Mathematical constant for 2*pi. */
	const double pi_2    = pi/2.0;  /**< @brief Mathematical constant for pi/2. */
	const double pi_4    = pi/4.0;  /**< @brief Mathematical constant for pi/4. */
	const float  two_pi_f= two_pi;  /**< @brief Mathematical constant (float format) for 2*pi. */
	const float  pi_f    = pi;  /**< @brief Mathematical constant (float format) for pi. */
	const float  pi_2_f  = pi_2;  /**< @brief Mathematical constant (float format) for pi/2. */
	const float  pi_4_f  = pi_4;  /**< @brief Mathematical constant (float format) for pi/4. */
#endif

// Above method of setting is more intelligent.
//const double two_pi  = 6.283185307179586476925286766559005768394338798750211641949888;  /**< @brief Mathematical constant for 2*pi. */
//const double pi      = 3.141592653589793238462643383279502884197169399375105820974944;  /**< @brief Mathematical constant for pi. */
//const double pi_2    = 1.57079632679489661923;  /**< @brief Mathematical constant for pi/2. */
//const double pi_4    = 0.78539816339744830962;  /**< @brief Mathematical constant for pi/4. */
//const float  two_pi_f= 6.283185307179586476925286766559005768394338798750211641949888;  /**< @brief Mathematical constant (float format) for 2*pi. */
//const float  pi_f    = 3.141592653589793238462643383279502884197169399375105820974944;  /**< @brief Mathematical constant (float format) for pi. */
//const float  pi_2_f  = 1.57079632679489661923;  /**< @brief Mathematical constant (float format) for pi/2. */
//const float  pi_4_f  = 0.78539816339744830962;  /**< @brief Mathematical constant (float format) for pi/4. */


}; // namespace ecl

#endif /*ECL_MATH_CONSTANTS_HPP_*/
