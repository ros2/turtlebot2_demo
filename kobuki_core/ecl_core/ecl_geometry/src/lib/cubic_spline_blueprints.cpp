/**
 * @file /src/lib/cubic_spline_blueprints.cpp
 *
 * @brief Implementation for cubic spline blueprints.
 *
 * Implementation for cubic spline blueprints.
 *
 * @sa @ref splinesGeometry "Math::Splines.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/geometry/cubic_spline.hpp"

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
 * This algorithm can be found in page 115 of "Numerical Recipes in C"
 * [The art of scientific computing] by Press, Vetterling, Tuekolsky, Flannery.
 */
C2CubicSpline::C2CubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set,
                                const double ydot_0, const double ydot_f) ecl_assert_throw_decl(StandardException) :
                                    x_data(x_set),
                                    y_data(y_set)
{
    if (x_data.size() < 2 || y_data.size() < 2)
    {
      ecl_throw(StandardException(LOC,OutOfRangeError) );
    }
    ecl_assert_throw( x_data.size() == y_data.size(), StandardException(LOC,OutOfRangeError) );
    unsigned int n = x_data.size();
    yddot_data.resize(n);
    Array<double> u(n);  // u is a temporary used in the algorithm

    // Initial boundary conditions
    yddot_data[0] = -0.5;
    u[0] = (3.0/(x_data[1]-x_data[0])) * ((y_data[1]-y_data[0])/(x_data[1]-x_data[0])-ydot_0);
    // Set up the tridiagonal matrix to solve for the accelerations
    for (unsigned int i = 1; i <= n-2; ++i){
        double sig = (x_data[i]-x_data[i-1]) / (x_data[i+1]-x_data[i-1]);
        double p = sig*yddot_data[i-1]+2.0;
        yddot_data[i] = (sig-1.0)/p;
        u[i] = (y_data[i+1]-y_data[i])/(x_data[i+1]-x_data[i]) -
               (y_data[i]-y_data[i-1])/(x_data[i]-x_data[i-1]);
        u[i] = (6.0*u[i]/(x_data[i+1]-x_data[i-1]) - sig*u[i-1])/p;
    }
    // Final boundary conditions
    double qn = 0.5;
    u[n-1] = (3.0/(x_data[n-1]-x_data[n-2])) * (ydot_f - (y_data[n-1]-y_data[n-2])/(x_data[n-1]-x_data[n-2]));

    // Back substitution loop of the tridiagonal algorithm
    yddot_data[n-1] = ( u[n-1] - qn*u[n-2]) / ( qn*yddot_data[n-2] + 1.0 );
    for ( int k = n-2; k >= 0; --k ) {
        yddot_data[k] = yddot_data[k]*yddot_data[k+1] + u[k];
    }
}

/*
 * This is a special case of the above. You'll note that only initial and
 * boundary conditions change here.
 */
C2CubicSpline::C2CubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set) ecl_assert_throw_decl(StandardException) :
                                    x_data(x_set),
                                    y_data(y_set)
{
    if (x_data.size() < 2 || y_data.size() < 2)
    {
      ecl_throw(StandardException(LOC,OutOfRangeError) );
    }
    ecl_assert_throw( x_data.size() == y_data.size(), StandardException(LOC,OutOfRangeError) );
    unsigned int n = x_data.size();
    yddot_data.resize(n);
    Array<double> u(n);  // u is a temporary used in the algorithm

    // Initial boundary conditions
    yddot_data[0] = 0.0;
    u[0] = 0.0;
    // Set up the tridiagonal matrix to solve for the accelerations
    for (unsigned int i = 1; i <= n-2; ++i){
        double sig = (x_data[i]-x_data[i-1]) / (x_data[i+1]-x_data[i-1]);
        double p = sig*yddot_data[i-1]+2.0;
        yddot_data[i] = (sig-1.0)/p;
        u[i] = (y_data[i+1]-y_data[i])/(x_data[i+1]-x_data[i]) -
               (y_data[i]-y_data[i-1])/(x_data[i]-x_data[i-1]);
        u[i] = (6.0*u[i]/(x_data[i+1]-x_data[i-1]) - sig*u[i-1])/p;
    }
    // Final boundary conditions
    double qn = 0.0;
    u[n-1] = 0.0;

    // Back substitution loop of the tridiagonal algorithm
    yddot_data[n-1] = ( u[n-1] - qn*u[n-2]) / ( qn*yddot_data[n-2] + 1.0 );
    for ( int k = n-2; k >= 0; --k ) {
        yddot_data[k] = yddot_data[k]*yddot_data[k+1] + u[k];
    }
}

ecl::CubicSpline C2CubicSpline::instantiate() {
    CubicSpline cubic;
    apply(cubic);
    return cubic;
};

void C2CubicSpline::apply(ecl::CubicSpline& spline) const {

    spline.discretised_domain = x_data;
    spline.cubic_polynomials.resize(x_data.size()-1); // One less polynomials than there is points
    for (unsigned int i = 0; i < spline.cubic_polynomials.size(); ++i ) {
        spline.cubic_polynomials[i] = CubicPolynomial::SecondDerivativeInterpolation(
                        x_data[i],   y_data[i],   yddot_data[i],
                        x_data[i+1], y_data[i+1], yddot_data[i+1]  );
    }
};


/*****************************************************************************
** Implementation [DerivativeHeuristicCubicSpline]
*****************************************************************************/

DerivativeHeuristicCubicSpline::DerivativeHeuristicCubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set,
        const double ydot_0, const double ydot_f) ecl_assert_throw_decl(StandardException) :
        x_data(x_set),
        y_data(y_set)
{
    ecl_assert_throw( x_data.size() == y_data.size(), StandardException(LOC,OutOfRangeError) );
    /*********************
    ** Via Velocities
    **********************/
    ydot_data.resize(x_data.size());
    ydot_data[0] = ydot_0;
    for (unsigned int i = 1; i < (x_data.size()-1); ++i ) { // Skip first and last.
        double ydot_before, ydot_after;
        ydot_before = (y_data[i]-y_data[i-1])/(x_data[i]-x_data[i-1]);
        ydot_after  = (y_data[i+1]-y_data[i])/(x_data[i+1]-x_data[i]);
        ydot_data[i] = (ydot_before + ydot_after)/2;
    }
    ydot_data[x_data.size()-1] = ydot_f;
}

ecl::CubicSpline DerivativeHeuristicCubicSpline::instantiate() {
    CubicSpline cubic;
    apply(cubic);
    return cubic;
};

void DerivativeHeuristicCubicSpline::apply(ecl::CubicSpline& spline) const {

    spline.discretised_domain = x_data;
    spline.cubic_polynomials.resize(x_data.size()-1); // One less polynomials than there is points
    for (unsigned int i = 0; i < spline.cubic_polynomials.size(); ++i ) {
        spline.cubic_polynomials[i] = CubicPolynomial::DerivativeInterpolation(
                        x_data[i],   y_data[i],   ydot_data[i],
                        x_data[i+1], y_data[i+1], ydot_data[i+1]  );
    }
};

} // namespace blueprints

/*****************************************************************************
** Implementation [BluePrintFactory][CubicSpline]
*****************************************************************************/

using blueprints::C2CubicSpline;
using blueprints::DerivativeHeuristicCubicSpline;

C2CubicSpline BluePrintFactory< CubicSpline >::Natural(const Array<double>& x_set, const Array<double>& y_set) {
    return C2CubicSpline(x_set, y_set);
};

            C2CubicSpline BluePrintFactory< CubicSpline >::ContinuousDerivatives(
            const Array<double>& x_set, const Array<double>& y_set, const double ydot_0, const double ydot_f) {
    return C2CubicSpline(x_set, y_set, ydot_0, ydot_f);
};

DerivativeHeuristicCubicSpline BluePrintFactory< CubicSpline >::DerivativeHeuristic(
            const Array<double>& x_set, const Array<double>& y_set, const double ydot_0, const double ydot_f) {
    return DerivativeHeuristicCubicSpline(x_set, y_set, ydot_0, ydot_f);
}

} // namespace ecl
