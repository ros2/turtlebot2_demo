/**
 * @file /src/lib/tension_spline_blueprints.cpp
 *
 * @brief Implementation for tension spline blueprints.
 *
 * @sa @ref splinesGeometry "Math::Splines.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/geometry/tension_spline.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace blueprints {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::Array;
using ecl::OutOfRangeError;
using ecl::StandardException;

/*****************************************************************************
** Implementation [C2CubicSpline]
*****************************************************************************/
/*
 * This algorithm can be found on page 323 of "Numerical Analysis" [Kincaid &
 * Cheney].
 */
C2TensionSpline::C2TensionSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set,
		const double &tau) ecl_assert_throw_decl(ecl::StandardException) :
    x_data(x_set),
    y_data(y_set)
{
    ecl_assert_throw( x_data.size() == y_data.size(), StandardException(LOC,OutOfRangeError) );

    /*********************
    ** Basic Parameters
    **********************/
    unsigned int n = x_data.size()-1; // No. of data points (x_0...x_n)
    yddot_data.resize(n+1);
    yddot_data[0] = 0;
    tension = tau;

    /******************************************
    ** Tridiagonal parameters
    *******************************************/
    // These equations are derived from the c2 constraints - it results in a tridiagonal system
    // of equations. See p.319 of kincaid and cheney for the tridiagonal solution (note this
    // is only valid for diagonally dominant tridiagonal matrices! Is this a problem for us?
    Array<double> h(n), a(n), beta(n), gamma(n), u(n), v(n);
    h[0] = x_set[1] - x_set[0];
    for ( unsigned int i = 0; i < n; ++i ) {
        h[i] = x_set[i+1]-x_set[i];
        a[i] = 1/h[i] - tension/sinh(tension*h[i]);
        beta[i] = tension*(cosh(tension*h[i])/sinh(tension*h[i])) - 1/h[i];
        gamma[i] = tension*tension*(y_data[i+1]-y_data[i])/h[i];
    }
    /*
     * [ u1 a1 0   0   0 ] [z1]    [ v1 ]
     * [ a1 u2 a2  0   0 ] [z2]    [ v2 ]
     * [  0 a2 u3 a3   0 ] [z3]    [ v3 ]
     *
     * but with back substitution here,
     *
     * [ u1      a1       0   0   0 ] [z1]    [     v1      ]
     * [  0 (u2-a1^2/u1) a2   0   0 ] [z2]    [ v2-a1*v1/u1 ]
     *
     * etc
     */
    u[1] = beta[1] + beta[0];
    v[1] = gamma[1]-gamma[0];
    for ( unsigned int i = 2; i < n; ++i ) {
        u[i] = beta[i] + beta[i-1] - a[i-1]*a[i-1]/u[i-1];
        v[i] = gamma[i]-gamma[i-1] - a[i-1]*v[i-1]/u[i-1];
    }
    /*
     * Now we can roll the upper triangular matrix backwards
     */
    yddot_data[n] = 0;
    for ( unsigned int i = n-1; i > 0; --i ) {
        yddot_data[i] = (v[i] - a[i]*yddot_data[i+1])/u[i];
    }
    yddot_data[0] = 0;
}

TensionSpline C2TensionSpline::instantiate() {
    TensionSpline cubic;
    apply(cubic);
    return cubic;
};

void C2TensionSpline::apply(TensionSpline& spline) const {

    spline.discretised_domain = x_data;
    spline.tension = tension;
    spline.functions.resize(x_data.size()-1); // One less polynomials than there is points
    for (unsigned int i = 0; i < spline.functions.size(); ++i ) {
        spline.functions[i] = TensionFunction::Interpolation(
                        x_data[i],   y_data[i],   yddot_data[i],
                        x_data[i+1], y_data[i+1], yddot_data[i+1]  );
    }
};

} // namespace blueprints

/*****************************************************************************
** Implementation [BluePrintFactory][TensionSpline]
*****************************************************************************/

using blueprints::C2TensionSpline;


C2TensionSpline BluePrintFactory< TensionSpline >::Natural(const Array<double>& x_set,
		const Array<double>& y_set, const double &tau) ecl_assert_throw_decl(StandardException) {
    return C2TensionSpline(x_set, y_set, tau);
};

} // namespace ecl
