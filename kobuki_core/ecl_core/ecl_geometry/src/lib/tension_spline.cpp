/**
 * @file /src/lib/tension_spline.cpp
 *
 * @brief Implementation for tension splines.
 *
 * Implementation for tension splines.
 *
 * @sa @ref splinesGeometry "Math::Splines.
 *
 * @author Daniel J. Stonier
 * @date 20/05/2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/geometry/tension_spline.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

double TensionSpline::operator()(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return functions[index](tension,x);
}

double TensionSpline::derivative(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return functions[index].derivative(tension,x);
}

double TensionSpline::dderivative(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    return functions[index].dderivative(tension,x);
}

} // namespace ecl
