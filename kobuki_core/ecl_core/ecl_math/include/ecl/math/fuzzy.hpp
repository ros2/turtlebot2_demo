/**
 * @file /ecl_math/include/ecl/math/fuzzy.hpp
 *
 * @brief Fuzzy math operators.
 *
 * Not really fuzzy math, but simple operators for when values are close to
 * the real deal. This could probably use an epsilon-approximate operator as well.
 *
 * @date Dec 29, 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MATH_FUZZY_HPP_
#define ECL_MATH_FUZZY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

//#include "num_traits.hpp"
#include <cmath> // floating point std::abs
#include <cstdlib> // integral std::abs
#include <algorithm> // std::min, std::max
#include <ecl/type_traits/numeric_limits.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
/**
 * @cond DO_NOT_DOXYGEN
 */
namespace implementations {

/****************************************************************************
* Implementation of fuzzy comparisons                                       *
****************************************************************************/

/**
 * @brief Parent template for scalar fuzzy implementations.
 *
 * This class is fully realised in its specialisations.
 */
template<typename Scalar,
         bool IsInteger>
struct scalar_fuzzy_default_impl {};

/**
 * @brief Fuzzy math implementation for floating points.
 *
 * @tparam Scalar : autoselected by numeric traits as always being integer.
 */
template<typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, false>
{
	typedef typename ecl::numeric_limits<Scalar>::Precision Precision;

	template<typename OtherScalar>
	static inline bool isApprox(const Scalar& x, const OtherScalar& y, const Precision& prec) {
		// casting just in case y is an integer type.
		return std::abs(x - static_cast<Scalar>(y)) <= std::min( std::abs(x),  std::abs(static_cast<Scalar>(y))) * prec;
	}

	template<typename OtherScalar>
	static inline bool isApproxOrLessThan(const Scalar& x, const OtherScalar& y, const Precision& prec) {
		return x <= y || isApprox(x, y, prec);
	}
};

/**
 * @brief Fuzzy math implementation for integers.
 *
 * @tparam Scalar : autoselected by numeric traits as always being integer.
 */
template<typename Scalar>
struct scalar_fuzzy_default_impl<Scalar, true> {

	typedef typename ecl::numeric_limits<Scalar>::Precision Precision;

	template<typename OtherScalar>
	static inline bool isApprox(const Scalar& x, const OtherScalar& y, const Precision& prec) {
		// this will trigger if default argument is used (dummy precision for integers == 0)
		if ( ecl::numeric_limits<OtherScalar>::is_integer ) {
			return x == y;
		} else {
			// only get here if its not complex, so OtherScalar has to be a floating type
			typename ecl::numeric_limits<OtherScalar>::Precision precision;
			if ( prec == 0.0 ) {
				precision = ecl::numeric_limits<OtherScalar>::dummy_precision;
			} else {
				precision = static_cast<OtherScalar>(prec);
			}
			return std::abs(static_cast<OtherScalar>(x) - y) <= std::min( std::abs(static_cast<OtherScalar>(x)),  std::abs(y)) * precision;
		}
	}
	template<typename OtherScalar>
	static inline bool isApproxOrLessThan(const Scalar& x, const OtherScalar& y, const Precision&) {
		return x <= y;
	}
};

///**
// * @brief Fuzzy math implementation for complex numbers.
// *
// * Specialisation of fuzzy math functions for complex numbers.
// *
// * @tparam Scalar : autoselected by numeric traits as always being complex.
// */
//template<typename Scalar>
//struct scalar_fuzzy_default_impl<Scalar, true, false> {
//	typedef typename ecl::NumTraits<Scalar>::Precision Precision;
//
//	static inline bool isApprox(const Scalar& x, const Scalar& y, const Precision& prec) {
//		return ei_abs2(x - y) <= std::min(ei_abs2(x), ei_abs2(y)) * prec * prec;
//	}
//};

/**
 * @brief Fuzzy implementation selector (using numeric traits).
 *
 * Chooses the correct implementation via use of the numeric traits class.
 *
 * @tparam Scalar : the type of number being used (floating, integer, complex).
 */
template<typename Scalar>
struct scalar_fuzzy_impl : implementations::scalar_fuzzy_default_impl<Scalar, numeric_limits<Scalar>::is_integer> {};

} // implementations

/**
 * @endcond
 */
/*****************************************************************************
** Direct Interfaces
*****************************************************************************/

/**
 * @brief Checks if two values are approximate.
 *
 * Note that this does not use the usual direct epsilon bound check,
 * instead, it uses the magnitudes of the values and supplied
 * precision to determine the epsilon bounds before carrying out the
 * operation.
 *
 * @param x - first input argument
 * @param y - second input argument
 * @param precision - precision to use (default is defined by the type).
 * @return bool : true or false of the operation.
 */
template<typename Scalar, typename OtherScalar>
inline bool isApprox(const Scalar& x, const OtherScalar& y, typename numeric_limits<Scalar>::Precision precision = numeric_limits<Scalar>::dummy_precision) {
	return implementations::scalar_fuzzy_impl<Scalar>::isApprox(x, y, precision);
}

/**
 * @brief Checks if the first value is less than or approximate to the second.
 *
 * Note that this does not use the usual direct epsilon bound check,
 * instead, it uses the magnitudes of the values and supplied
 * precision to determine the epsilon bounds before carrying out the
 * operation.
 *
 * @param x - first input argument
 * @param y - second input argument
 * @param precision - precision to use (default is defined by the type).
 * @return bool : true or false of the operation.
 */
template<typename Scalar, typename OtherScalar>
inline bool isApproxOrLessThan(const Scalar& x, const OtherScalar& y, typename numeric_limits<Scalar>::Precision precision = numeric_limits<Scalar>::dummy_precision) {
	return implementations::scalar_fuzzy_impl<Scalar>::isApproxOrLessThan(x, y, precision);
}



} // namespace ecl

#endif /* ECL_MATH_FUZZY_HPP_ */
