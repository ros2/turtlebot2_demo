/**
 * @file /src/lib/angle.cpp
 *
 * @brief Implementation for regular radian/degree angle types.
 *
 * Implementation for regular radian/degree angle types.
 *
 * @date February 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include "../../include/ecl/geometry/angle.hpp"
#include <ecl/math/constants.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [Functions]
*****************************************************************************/

const float& wrap_angle(float &angle) {
	if ( (  angle <= pi ) && ( angle >= -pi ) ) {
		return angle; // nothing to do.
	}
	if ( angle < 0.0 ) {
		angle = fmodf(angle-pi,2.0*pi)+pi;
	} else {
		angle = fmodf(angle+pi,2.0*pi)-pi;
	}
	return angle;
}
float wrap_angle(const float &angle) {
	float wrapped;
	if ( (  angle <= pi ) && ( angle >= -pi ) ) {
		wrapped = angle;
	} else if ( angle < 0.0 ) {
		wrapped = fmodf(angle-pi,2.0*pi)+pi;
	} else {
		wrapped = fmodf(angle+pi,2.0*pi)-pi;
	}
	return wrapped;
}

const double& wrap_angle(double &angle) {
	if ( (  angle <= pi ) && ( angle >= -pi ) ) {
		return angle; // nothing to do.
	}
	if ( angle < 0.0 ) {
		angle = fmod(angle-pi,2.0*pi)+pi;
	} else {
		angle = fmod(angle+pi,2.0*pi)-pi;
	}
	return angle;
}

double wrap_angle(const double &angle) {
	double wrapped;
	if ( (  angle <= pi ) && ( angle >= -pi ) ) {
		wrapped = angle;
	} else if ( angle < 0.0 ) {
		wrapped = fmod(angle-pi,2.0*pi)+pi;
	} else {
		wrapped = fmod(angle+pi,2.0*pi)-pi;
	}
	return wrapped;
}

} // namespace ecl
