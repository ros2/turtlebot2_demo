/**
 * @file /src/lib/polynomial_blueprints.cpp
 *
 * @brief Implementation of the polynomial blueprints.
 *
 * Implementation of the polynomial blueprints.
 *
 * @sa @ref polynomialsGeometry "Math::Polynomials".
 *
 * @author Daniel J. Stonier
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/geometry/polynomial.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace blueprints {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::LinearFunction;
using ecl::CubicPolynomial;
using ecl::QuinticPolynomial;

/*****************************************************************************
** Implementation [LinearInterpolation][LinearPointSlopeForm]
*****************************************************************************/

ecl::LinearFunction LinearInterpolation::instantiate() {
    LinearFunction function;
    apply(function);
    return function;
}

void LinearInterpolation::apply(ecl::LinearFunction& function) const {
    LinearFunction::Coefficients &coefficients = function.coefficients();
    // First compute the polynomial on [0,xf-xi] (this is easier), then shift it.

    // y = a_1 x + a_0
    double a_1 = (y_final-y_initial)/(x_final-x_initial);
    double a_0 = y_initial - a_1*x_initial;

    coefficients <<  a_0, a_1;
}

ecl::LinearFunction LinearPointSlopeForm::instantiate() {
    LinearFunction function;
    apply(function);
    return function;
}

void LinearPointSlopeForm::apply(ecl::LinearFunction& function) const {
    LinearFunction::Coefficients &coefficients = function.coefficients();
    // First compute the polynomial on [0,xf-xi] (this is easier), then shift it.

    // y = a_1 x + a_0
    double a_1 = slope;
    double a_0 = y_final - a_1*x_final;

    coefficients <<  a_0, a_1;
}


/*****************************************************************************
** Implementation [CubicDerivativeInterpolation]
*****************************************************************************/


ecl::CubicPolynomial CubicDerivativeInterpolation::instantiate() {
    CubicPolynomial cubic;
    apply(cubic);
    return cubic;
}

void CubicDerivativeInterpolation::apply(ecl::CubicPolynomial& polynomial) const {
    Polynomial<3>::Coefficients &coefficients = polynomial.coefficients();
    // First compute the polynomial on [0,xf-xi] (this is easier), then shift it.
    double dx = x_final - x_initial;
    double dy = y_final - y_initial;
    coefficients << y_initial,
                    ydot_initial,
                    (3/(dx*dx))*(dy) - (2/dx)*ydot_initial - (1/dx)*ydot_final,
                    (-2/(dx*dx*dx))*(dy) + (ydot_final + ydot_initial)/(dx*dx);

//    Matrix<double,4,4> A;
//	Matrix<double,4,1> a,b;
//	b << y_initial, ydot_initial, y_final, ydot_final;
//
//	A <<     1,     0,     0,     0,
//			 0,     1,     0,     0,
//			 1,	   dx, dx*dx, dx*dx*dx,
//			 0,     1,  2*dx,  3*dx*dx;
//
//	a = A.inverse()*b;
//	std::cout << coefficients << std::endl;
//	std::cout << a << std::endl;

    if ( x_initial != 0.0 ) {
        polynomial.shift_horizontal(x_initial);
    }
}

/*****************************************************************************
** Implementation [CubicDerivativeInterpolation]
*****************************************************************************/

ecl::CubicPolynomial CubicSecondDerivativeInterpolation::instantiate() {
    CubicPolynomial cubic;
    apply(cubic);
    return cubic;
}

void CubicSecondDerivativeInterpolation::apply(ecl::CubicPolynomial& polynomial) const {
    Polynomial<3>::Coefficients &coefficients = polynomial.coefficients();
    // First compute the polynomial on [0,xf-xi] (this is easier), then shift it.
    double dx = x_final - x_initial;
    double a_0 = y_initial;
    double a_2 = yddot_initial/2;
    double a_3 = (yddot_final - yddot_initial)/(6*dx);
    double a_1 = (y_final-a_0-a_2*dx*dx - a_3*dx*dx*dx)/dx;
    coefficients <<  a_0, a_1, a_2, a_3;
    if ( x_initial != 0.0 ) {
        polynomial.shift_horizontal(x_initial);
    }
}

/*****************************************************************************
** Implementation [QuinticInterpolation]
*****************************************************************************/

ecl::QuinticPolynomial QuinticInterpolation::instantiate() {
    QuinticPolynomial quintic;
    apply(quintic);
    return quintic;
}

void QuinticInterpolation::apply(ecl::QuinticPolynomial& polynomial) const {
    QuinticPolynomial::Coefficients &coefficients = polynomial.coefficients();
    // First compute the polynomial on [0,xf-xi] (this is easier), then shift it.

    double dx = x_final - x_initial;
    double d2x = dx*dx;
    double d3x = d2x*dx;
    double d4x = d3x*dx;
    double d5x = d4x*dx;
    double a_0 = y_initial;
    double a_1 = ydot_initial;
    double a_2 = yddot_initial/2;
    double a_3 = ( 20*(y_final - y_initial) - (8*ydot_final + 12*ydot_initial)*dx - (3*yddot_initial - yddot_final)*dx*dx )
                            / (2*d3x);
    double a_4 = ( 30*(y_initial - y_final) + (14*ydot_final + 16*ydot_initial)*dx + (- 2*yddot_final + 3*yddot_initial)*dx*dx )
                            / (2*d4x);
    double a_5 = ( 12*(y_final - y_initial) - (6*ydot_final + 6*ydot_initial)*dx - (yddot_initial - yddot_final)*dx*dx )
                            / (2*d5x);

    coefficients <<  a_0, a_1, a_2, a_3, a_4, a_5;

//    Matrix<double,6,6> A;
//    Matrix<double,6,1> a,b;
//    b << y_initial, ydot_initial, yddot_initial, y_final, ydot_final, yddot_final;
//
//    A <<     1,     0,     0,     0,      0,      0,
//             0,     1,     0,     0,      0,      0,
//             0,     0,     2,     0,      0,      0,
//             1,    dx,   d2x,   d3x,    d4x,    d5x,
//             0,     1,  2*dx, 3*d2x,  4*d3x,  5*d4x,
//             0,     0,    2,   6*dx, 12*d2x, 20*d3x;
//
//    a = A.inverse()*b;
//    std::cout << coefficients << std::endl;
//    std::cout << a << std::endl;

    if ( x_initial != 0.0 ) {
        polynomial.shift_horizontal(x_initial);
    }
}


}; // namespace blueprints

using blueprints::LinearInterpolation;
using blueprints::LinearPointSlopeForm;
using blueprints::CubicDerivativeInterpolation;
using blueprints::CubicSecondDerivativeInterpolation;
using blueprints::QuinticInterpolation;

/*****************************************************************************
** BluePrintFactory[LinearFunction]
*****************************************************************************/

LinearInterpolation BluePrintFactory< LinearFunction >::Interpolation(const double x_i, const double y_i, const double x_f, const double y_f) {
    return LinearInterpolation(x_i, y_i, x_f, y_f);
}

LinearPointSlopeForm BluePrintFactory< LinearFunction >::PointSlopeForm(const double x_f, const double y_f, const double slope) {
    return LinearPointSlopeForm(x_f, y_f, slope);
}

/*****************************************************************************
** BluePrintFactory[CubicPolynomial]
*****************************************************************************/

CubicDerivativeInterpolation BluePrintFactory< CubicPolynomial >::DerivativeInterpolation(const double x_i, const double y_i, const double ydot_i, const double x_f, const double y_f, const double ydot_f) {
    return CubicDerivativeInterpolation(x_i, y_i, ydot_i, x_f, y_f, ydot_f);
}

CubicSecondDerivativeInterpolation BluePrintFactory< CubicPolynomial >::SecondDerivativeInterpolation(const double x_i, const double y_i, const double yddot_i, const double x_f, const double y_f, const double yddot_f) {
    return CubicSecondDerivativeInterpolation(x_i, y_i, yddot_i, x_f, y_f, yddot_f);
}

/*****************************************************************************
** BluePrintFactory[QuinticPolynomial]
*****************************************************************************/

QuinticInterpolation BluePrintFactory< QuinticPolynomial >::Interpolation(const double x_i, const double y_i, const double ydot_i, const double yddot_i,
                            const double x_f, const double y_f, const double ydot_f, const double yddot_f) {
    return QuinticInterpolation(x_i, y_i, ydot_i, yddot_i, x_f, y_f, ydot_f, yddot_f);
}


} // namespace ecl
