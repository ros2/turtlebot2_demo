/**
 * @file /include/ecl/geometry/angle.hpp
 *
 * @brief C++ interface for angles (degrees/radians).
 *
 * @date February 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_ANGLE_HPP_
#define ECL_GEOMETRY_ANGLE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include <ecl/mpl/enable_if.hpp>
#include <ecl/math/constants.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Converts radians to degrees and returns the result.
 *
 * @param radians : input value in radians (float or double type).
 * @param dummy : ignore this, is used for sfinae concept.
 * @return T : the angle in degrees.
 */
template <typename T>
ecl_geometry_PUBLIC T radians_to_degrees(const T &radians, typename enable_if<ecl::is_float<T> >::type* dummy = 0) {
	static const T rads_to_degs = 180.0/pi;
	return radians*rads_to_degs;
}

/**
 * @brief Converts degrees to radians and returns the result.
 *
 * @param degrees : input value in degrees (float or double type).
 * @param dummy : ignore this, is used for sfinae concept.
 * @return T : the angle in radians.
 */
template <typename T>
ecl_geometry_PUBLIC T degrees_to_radians(const T &degrees, typename enable_if<ecl::is_float<T> >::type* dummy = 0) {
	static const T degs_to_rads = pi/180.0;
	return degrees*degs_to_rads;
}

/**
 * @brief Wrap the angle on -pi,pi (float types).
 *
 * This uses the float versions of the math functions to wrap
 * the angle on -pi, pi. This is the fast version which
 * acts on the input variable.
 *
 * @param angle : the angle to be wrapped.
 */
ecl_geometry_PUBLIC const float& wrap_angle(float &angle);

/**
 * @brief Return the wrapped the angle on -pi,pi (float types).
 *
 * This uses the float versions of the math functions to wrap
 * the angle on -pi, pi. This is the slow version which creates
 * a copy and returns it.
 *
 * @param angle : the angle to be wrapped.
 * @return float : the wrapped angle.
 */
ecl_geometry_PUBLIC float wrap_angle(const float &angle);
/**
 * @brief Wrap the angle on -pi,pi (double types).
 *
 * This uses the double versions of the math functions to wrap
 * the angle on -pi, pi. This is the fast version which
 * acts on the input variable.
 *
 * @param angle : the angle to be wrapped.
 */
ecl_geometry_PUBLIC const double& wrap_angle(double &angle);

/**
 * @brief Return the wrapped the angle on -pi,pi (double types).
 *
 * This uses the double versions of the math functions to wrap
 * the angle on -pi, pi. This is the slow version which creates
 * a copy and returns it.
 *
 * @param angle : the angle to be wrapped.
 * @return double : the wrapped angle.
 */
ecl_geometry_PUBLIC double wrap_angle(const double &angle);

/*****************************************************************************
** Interface [Angle]
*****************************************************************************/
/**
 * @brief Parent template definition for angles.
 *
 * Do not use this directly. Use the specialisations instead.
 */
template <class T, typename Enable = void>
class ecl_geometry_PUBLIC Angle {
private:
	/**
	 * @brief Prevents usage of this template class directly.
	 *
	 * This class isn't yet intended for generic use - use the specialisations
	 * instead.
	 */
	Angle() {};
};

/**
 * @brief Interface for angular measurements.
 *
 * This is a simple interface designed for scientific calculations (thus
 * the default format is in radians). It allows free interaction with
 * float types, does automatic wrapping on [-pi, pi] under the hood and
 * provides some generic methods for conversions and wrapping on
 * plain double types.
 *
 * @tparam T : must be a float type (e.g. float, double, float32, float64)
 */
template <typename T>
class ecl_geometry_PUBLIC Angle<T, typename enable_if<is_float<T> >::type> {
public:
	/*********************
	** Typedefs
	**********************/
	typedef ecl::linear_algebra::Matrix<T,2,2> RotationMatrix;
	/**
	 * @brief Constructs an angle in radians.
	 *
	 * Configures the angle from the input double value. Automatically wraps
	 * this value if it doesn't fall within [-pi,pi].
	 *
	 * @param angle : input angle (radians).
	 */
	Angle(const T &angle = 0.0) : value(angle) { wrap_angle(value); }
	/**
	 * @brief Construct from a 2x2 rotation matrix.
	 * @param rotation : input rotation matrix.
	 */
	Angle(const RotationMatrix &rotation) {
		value = std::atan2(rotation(1,0),rotation(0,0));
	}

	/**
	 * @brief Allow assignment from float types.
	 *
	 * This allows convenient assignment from float types. It does
	 * automatic wrapping on the input value.
	 *
	 * @param angle : input angle (radians).
	 * @return Angle : the wrapped angle instance.
	 */
	const Angle<T>& operator=(const T &angle);
	/*********************
	** Math Operators
	**********************/
	/**
	 * @brief Sum operator.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to add (radians).
	 * @return Angle : the wrapped angle instance.
	 */
	Angle<T> operator+(const T &angle) const;
	/**
	 * @brief Sum operator.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to add (radians).
	 * @return Angle : the wrapped angle instance.
	 */
	Angle<T> operator+(const Angle<T> &angle) const;
	/**
	 * @brief Sum operator that works on this instance.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to add (radians).
	 */
	void operator+=(const T &angle);
	/**
	 * @brief Sum operator that works on this instance.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to add (radians).
	 */
	void operator+=(const Angle<T> &angle);
	/**
	 * @brief Difference operator.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to subtract (radians).
	 * @return Angle : the wrapped angle instance.
	 */
	Angle<T> operator-(const T &angle) const;
	/**
	 * @brief Difference operator.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to subtract (radians).
	 * @return Angle : the wrapped angle instance.
	 */
	Angle<T> operator-(const Angle<T> &angle) const;
	/**
	 * @brief Difference operator that works on this instance.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to subtract (radians).
	 */
	void operator-=(const T &angle);
	/**
	 * @brief Difference operator that works on this instance.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param angle : angle to subtract (radians).
	 */
	void operator-=(const Angle<T> &angle);
	/**
	 * @brief Multiplier.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param scalar : scalar multipler to apply.
	 * @return Angle : the wrapped angle instance.
	 */
	Angle<T> operator*(const T &scalar) const;
	/**
	 * @brief Multiplier that works on this instance.
	 *
	 * Note this also ensures the angle is wrapped.
	 *
	 * @param scalar : scalar multipler to apply.
	 */
	void operator*=(const T &scalar);

	/*********************
	** Casting
	**********************/
	/**
	 * @brief Convenient equivalence to the c/c++ float type.
	 *
	 * Allows for easy manipulation, especially within calculations. This
	 * breaks the rule which dictates that we should never use them
	 * as they create uncertainty about an object. Here it's just
	 * ridiculously convenient and more to the point, intuitive,
	 * so we do so.
	 */
	operator const T&() const { return value; }
	/**
	 * @brief Returns the value of the angle in unit degrees.
	 *
	 * This does not change the internal formatting for the angle
	 * (which is in radians). It simply returns its equivalent
	 * in unit degrees. The returned value will be wrapped
	 * on [-180, 180].
	 *
	 * @return T : the angle in degrees.
	 */
	T degrees();

	/**
	 * @brief Convert to a 2x2 rotation matrix.
	 * @return RotationMatrix : representation as a 2x2 rotation matrix.
	 */
	RotationMatrix rotationMatrix() const {
		T c = std::cos(value);
		T s = std::sin(value);
		return (RotationMatrix() << c, -s, s, c).finished();
	}

	/**
	 * @brief Generates an angle instance from an angle in degrees.
	 *
	 * Generates an angle specified in degrees as a double to
	 * a fully constructed Angle instance.
	 *
	 * @code
	 * Angle angle = Angle::Degrees(45);
	 * @endcode
	 *
	 * @param angle : the angle (degrees).
	 * @return Angle : the constructed instance.
	 */
	static Angle<T> Degrees(const T &angle);
	/**
	 * @brief Generates an angle instance from an angle in radians.
	 *
	 * Generates an angle specified in radians as a double to a
	 * fully constructed Angle instance.
	 *
	 * @code
	 * Angle angle = Angle::Radians(ecl::math::pi);
	 * @endcode
	 *
	 * @param angle : the angle (radians).
	 * @return Angle : the constructed instance.
	 */
	static Angle<T> Radians(const T &angle);

private:
	T value;
};

/**
 * This template magic mucks up the doxy a bit.
 * @cond DO_NOT_DOXYGEN
 */
/*****************************************************************************
** Implementation
*****************************************************************************/

template <typename T>
T Angle<T, typename enable_if<is_float<T> >::type>::degrees() {
	return radians_to_degrees(value);
}

template <typename T>
const Angle<T>& Angle<T, typename enable_if<is_float<T> >::type>::operator=(const T &angle) {
	value = angle;
	wrap_angle(value);
	return *this;
}

/*****************************************************************************
** Operators
*****************************************************************************/

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::operator+(const T &angle) const {
	return Angle<T>(wrap_angle(value+angle));
}

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::operator+(const Angle<T> &angle) const {
	return Angle<T>(wrap_angle(value+angle.value));
}

template <typename T>
void Angle<T, typename enable_if<is_float<T> >::type>::operator+=(const T &angle) {
	value += angle;
	wrap_angle(value);
}

template <typename T>
void Angle<T, typename enable_if<is_float<T> >::type>::operator+=(const Angle<T> &angle) {
	value += angle.value;
	wrap_angle(value);
}

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::operator-(const T &angle) const {
	return Angle<T>(wrap_angle(value-angle));
}

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::operator-(const Angle<T> &angle) const {
	return Angle<T>(wrap_angle(value-angle.value));
}

template <typename T>
void Angle<T, typename enable_if<is_float<T> >::type>::operator-=(const T &angle) {
	value -= angle;
	wrap_angle(value);
}

template <typename T>
void Angle<T, typename enable_if<is_float<T> >::type>::operator-=(const Angle<T> &angle) {
	value -= angle.value;
	wrap_angle(value);
}

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::operator*(const T &scalar) const {
	return Angle<T>(wrap_angle(scalar*value));
}

template <typename T>
void Angle<T, typename enable_if<is_float<T> >::type>::operator*=(const T &scalar) {
	value *= scalar;
	wrap_angle(value);
}

/*****************************************************************************
** Statics
*****************************************************************************/

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::Degrees(const T &angle) {
	return Angle<T>(degrees_to_radians(angle));
}

template <typename T>
Angle<T> Angle<T, typename enable_if<is_float<T> >::type>::Radians(const T &angle) {
	return Angle<T>(angle);
}
/**
 * @endcond
 */

} // namespace ecl

#endif /* ECL_GEOMETRY_ANGLE_HPP_ */
