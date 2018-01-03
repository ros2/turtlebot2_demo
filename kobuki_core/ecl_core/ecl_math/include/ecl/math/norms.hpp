/**
 * @file /include/ecl/math/norms.hpp
 *
 * @brief Some simple functors to calculate various norms.
 *
 * These aren't meant to be all encompassing, rather just a few oft used
 * functions that always seem to crop up. Use eigen for more complicated
 * functionalities.
 *
 * @date February 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MATH_NORMS_HPP_
#define ECL_MATH_NORMS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Functors
*****************************************************************************/
/**
 * @brief Functor for euclidean norms.
 *
 * This class can calculate euclidean norms for any type (int, float, double).
 */
class EuclideanNorm {
public:
	/**
	 * @brief Functor operator for euclidean norms of dimension 2.
	 *
	 * @param x1 : first argument
	 * @param x2 : second argument
	 * @return T : result of the input.
	 */
	template <typename T>
	T operator()(const T& x1, const T& x2) {
		return std::sqrt(x1*x1 + x2*x2);
	}

	/**
	 * @brief Functor operator for euclidean norms of dimension 3.
	 *
	 * @param x1 : first argument
	 * @param x2 : second argument
	 * @param x3 : third argument
	 * @return T : result of the input.
	 */
	template <typename T>
	T operator()(const T& x1, const T& x2, const T& x3) {
		return std::sqrt(x1*x1 + x2*x2 + x3*x3);
	}

	// n dimensional cases using vectors, valarrays and eigen vectors here.

	// maybe also a norm(vectorx, vectory) which is sqrt((x0-y0)^2+(x1-y1)^2.....)
};

/*****************************************************************************
** Functions
*****************************************************************************/
/**
 * @brief Function for euclidean norms of dimension 2.
 *
 * @param x1 : first argument
 * @param x2 : second argument
 * @return T : result of the input.
 */
template <typename T>
T euclidean_norm(const T& x1, const T& x2) {
	return std::sqrt(x1*x1 + x2*x2);
}

/**
 * @brief Function for euclidean norms of dimension 3.
 *
 * @param x1 : first argument
 * @param x2 : second argument
 * @param x3 : second argument
 * @return T : result of the input.
 */
template <typename T>
T euclidean_norm(const T& x1, const T& x2, const T& x3) {
	return std::sqrt(x1*x1 + x2*x2 + x3*x3);
}

} // namespace ecl

#endif /* ECL_MATH_NORMS_HPP_ */
