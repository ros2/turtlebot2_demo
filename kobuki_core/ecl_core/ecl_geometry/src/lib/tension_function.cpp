/**
 * @file /src/lib/tension_function.cpp
 *
 * @brief Implementation of the tension function.
 *
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include "../../include/ecl/geometry/tension_function.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [TensionFunction]
*****************************************************************************/

double TensionFunction::derivative(const double &tau, const double &x) const {
    double value;
    double h = x_f-x_0;
    value = (-1.0*tau*z_0*cosh(tau*(x_f-x)) + tau*z_f*cosh(tau*(x-x_0)))/(tau*tau*sinh(tau*h));
    value += -1.0*(y_0-z_0/(tau*tau))/h;
    value += (y_f-z_f/(tau*tau))/h;

    return value;
}


double TensionFunction::dderivative(const double &tau, const double &x) const {
    double value;
    double h = x_f-x_0;
    value = (tau*tau*z_0*sinh(tau*(x_f-x)) + tau*tau*z_f*sinh(tau*(x-x_0)))/(tau*tau*sinh(tau*h));

    return value;
}

double TensionFunction::operator ()(const double &tau, const double &x) const {
    double value;
    double h = x_f-x_0;
    value = (z_0*sinh(tau*(x_f-x)) + z_f*sinh(tau*(x-x_0)))/(tau*tau*sinh(tau*h));
    value += (y_0-z_0/(tau*tau))*(x_f-x)/h;
    value += (y_f-z_f/(tau*tau))*(x-x_0)/h;
    return value;
}

namespace blueprints {

using ecl::TensionFunction;

/*****************************************************************************
** Implementation [TensionSecondDerivativeInterpolation]
*****************************************************************************/

ecl::TensionFunction TensionSecondDerivativeInterpolation::instantiate() {
    TensionFunction function;
    apply(function);
    return function;
}

void TensionSecondDerivativeInterpolation::apply(base_type &function) const {

    // Dont have to do much here, just copy across the parameters
    function.z_0 = yddot_initial;
    function.z_f = yddot_final;
    function.x_0 = x_initial;
    function.x_f = x_final;
    function.y_0 = y_initial;
    function.y_f = y_final;
}

}; // namespace blueprints

using blueprints::TensionSecondDerivativeInterpolation;

/*****************************************************************************
** BluePrintFactory[TensionFunction]
*****************************************************************************/

TensionSecondDerivativeInterpolation BluePrintFactory< TensionFunction >::Interpolation(const double x_i, const double y_i, const double yddot_i, const double x_f, const double y_f, const double yddot_f) {
    return TensionSecondDerivativeInterpolation(x_i, y_i, yddot_i, x_f, y_f, yddot_f);
}

} // namespace ecl
