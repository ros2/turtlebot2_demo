/**
 * @file /src/lib/cubic_spline.cpp
 *
 * @brief Implementation for cubic splines.
 *
 * Implementation for cubic splines.
 *
 * @sa @ref splinesGeometry "Math::Splines.
 *
 * @date June 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/geometry/cubic_spline.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

double CubicSpline::operator()(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return cubic_polynomials[index](x);
}

double CubicSpline::derivative(double x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return cubic_polynomials[index].derivative()(x);
}

double CubicSpline::dderivative(double x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return cubic_polynomials[index].derivative().derivative()(x);
}

} // namespace ecl
