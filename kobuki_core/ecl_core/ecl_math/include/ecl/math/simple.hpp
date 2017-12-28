/**
 * @file /ecl_math/include/ecl/math/simple.hpp
 *
 * @brief Simple math functions.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CORE_MATH_SIMPLE_HPP_
#define ECL_CORE_MATH_SIMPLE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Functions
*****************************************************************************/
/**
 * @brief A sign function for numerical values.
 *
 * Note that zero will return zero, positive numbers +1, and negative numbers -1.
 *
 * @param x : scalar to test.
 * @tparam Scalar : some numerical quantity satisfying a comparison with >= 0
 * @return bool : true if x is >= 0.
 */
template <typename Scalar>
inline int sign(const Scalar &x) {
	// ToDo: should probably check some numeric traits here
	// Is this faster? (v > 0) - (v < 0);
        // http://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
	if ( x > 0 ) {
		return 1;
	} else if ( x < 0 ) {
		return -1;
	} else {
		return 0;
	}
}

/**
 * @brief A sign function for numerical values (positive return if arg is zero).
 *
 * Note that zero will return +1, positive numbers +1, and negative numbers -1.
 *
 * @param x : scalar to test.
 * @tparam Scalar : some numerical quantity satisfying a comparison with >= 0
 * @return bool : true if x is >= 0.
 */
template <typename Scalar>
inline int psign(const Scalar &x) {
	// ToDo: should probably check some numeric traits here
	if ( x >= 0 ) {
		return 1;
	} else {
		return -1;
	}
}

/**
 * @brief A sign function for numerical values (negative return if arg is zero).
 *
 * Note that zero will return -1, positive numbers +1, and negative numbers -1.
 *
 * @param x : scalar to test.
 * @tparam Scalar : some numerical quantity satisfying a comparison with >= 0
 * @return bool : true if x is >= 0.
 */
template <typename Scalar>
inline int nsign(const Scalar &x) {
        // ToDo: should probably check some numeric traits here
        if ( x > 0 ) {
                return 1;
        } else {
                return -1;
        }
}

/**
 * @brief The real solution to a cube root.
 *
 * @param x : scalar to test.
 * @tparam Scalar : a real numerical value
 * @return Scalar : real value solution to the cubic root
 */
template <typename Scalar>
inline Scalar cube_root(const Scalar &x) {
	// ToDo: should probably check some numeric traits here
	return psign(x)*pow(fabs(x),1.0/3.0);
}


} // namespace ecl

#endif /* ECL_CORE_MATH_SIMPLE_HPP_ */
