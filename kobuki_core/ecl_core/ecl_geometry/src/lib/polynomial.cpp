/**
 * @file /src/lib/polynomial.cpp
 *
 * @brief Implementation of the polynomials.
 *
 * Implementation of the polynomials.
 *
 * @author Daniel J. Stonier
 * @date May 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
//#include <ecl/linear_algebra.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/math/constants.hpp>
#include <ecl/math/fuzzy.hpp>
#include <ecl/math/simple.hpp> // cube_root
#include "../../include/ecl/geometry/polynomial.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [Synthetic Division]
*****************************************************************************/

LinearFunction Division< QuadraticPolynomial >::operator()(const QuadraticPolynomial &p, const double &factor, double &remainder) {
	double a, b; // linear coefficients
	a = p.coefficients()[2];
	b = p.coefficients()[1] + factor*a;
	LinearFunction f;
	f.coefficients() << b, a;
	remainder = p.coefficients()[0]+factor*b;
	return f;
}

QuadraticPolynomial Division< CubicPolynomial >::operator()(const CubicPolynomial &p, const double &factor, double &remainder) {
	double a, b, c; // quadratic coefficients
	a = p.coefficients()[3];
	b = p.coefficients()[2] + factor*a;
	c = p.coefficients()[1] + factor*b;
	QuadraticPolynomial q;
	q.coefficients() << c, b, a;
	remainder = p.coefficients()[0]+factor*c;
	return q;
}

/*****************************************************************************
** Implementation [Minimum|Maximum][Polynomial]
*****************************************************************************/

Array<double> Roots< LinearFunction >::operator()(const LinearFunction& p) {
	Array<double> intercepts;
	double a = p.coefficients()[1];
	double b = p.coefficients()[0];
	if ( a != 0 ) {
		intercepts.resize(1);
		intercepts << -1.0*b/a;
	}
	return intercepts;
}

Array<double> Roots< QuadraticPolynomial >::operator()(const QuadraticPolynomial& p) {
	Array<double> intercepts;
	double a = p.coefficients()[2];
	double b = p.coefficients()[1];
	double c = p.coefficients()[0];
	if ( a == 0 ) {
		LinearFunction f;
		f.coefficients() << c, b;
		intercepts = Roots<LinearFunction>()(f);
	} else {
		double discriminant = b*b - 4*a*c;
		if ( discriminant > 0.0 ) {
			intercepts.resize(2);
			intercepts << (-b + sqrt(discriminant))/(2*a), (-b - sqrt(discriminant))/(2*a);
		} else if ( discriminant == 0.0 ) {
			intercepts.resize(1);
			intercepts << -b/(2*a);
		}
	}
	return intercepts;
}

	// http://en.wikipedia.org/wiki/Cubic_function#Roots_of_a_cubic_function
Array<double> Roots< CubicPolynomial >::operator()(const CubicPolynomial& polynomial) {
	Array<double> intercepts;
	double a = polynomial.coefficients()[3];
	double b = polynomial.coefficients()[2];
	double c = polynomial.coefficients()[1];
	double d = polynomial.coefficients()[0];

//	form the companion matrix (http://stackoverflow.com/questions/2003465/fastest-numerical-solution-of-a-real-cubic-polynomial)
//	ecl::linear_algebra::Matrix3d A;
//	A << 0, 0, -d/a, 1, 0, -c/a, 0, 1, -b/a;
//	ecl::linear_algebra::EigenSolver<ecl::linear_algebra::Matrix3d> eigensolver(A);
//	if (eigensolver.info() != ecl::linear_algebra::Success) abort();
//	cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;

	// Monic Trinomial coefficients
	double p = (3*a*c - b*b)/(3*a*a);
	double q = (2*b*b*b - 9*a*b*c + 27*a*a*d)/(27*a*a*a);
	double discriminant = p*p*p/27 + q*q/4;
//	std::cout << "p: " << p << std::endl;
//	std::cout << "q: " << q << std::endl;
	// using transform x = t - b/3a, we can solve t^3+pt+q=0
	double shift = -b/(3*a);
	if ( ( p == 0 ) && ( q == 0 ) ) {
		// single, triple root at t = 0
		intercepts.resize(1);
		intercepts << shift;
//		std::cout << "Single triple root" << std::endl;
	} else if ( p == 0 ) { // && ( q != 0 )
		// single root from the cube root function
//		std::cout << "Single root from cube root function" << std::endl;
		intercepts.resize(1);
		intercepts << ecl::cube_root(-q)+ shift;
	} else if ( q == 0 ) {
//		std::cout << "Three real roots" << std::endl;
		// three real roots
		intercepts.resize(3);
		intercepts << shift, sqrt(-1*p) + shift, -sqrt(-1*p) + shift;
	} else if ( discriminant == 0 ) { // && ( p != 0 ) )  {
//		std::cout << "Double root" << std::endl;
//		// double root and simple root
		intercepts.resize(2);
		intercepts << 3*q/p + shift, (-3*q)/(2*p) + shift;
	} else if ( discriminant >= 0 ) {
//		std::cout << "Discriminant: " << discriminant << std::endl;
		double u = ecl::cube_root(-q/2 + sqrt(discriminant));
		double v = ecl::cube_root(-q/2 - sqrt(discriminant));
		intercepts.resize(1);
		intercepts << u + v + shift;
	} else { // discriminant < 0 this is cardano's casus irreducibilis and there is three real roots
		// switch to transcendental solutions (only works for three real roots)
		// http://en.wikipedia.org/wiki/Cubic_function#Trigonometric_.28and_hyperbolic.29_method
//		std::cout << "Discriminant: " << discriminant << std::endl;
		double t_1 = 2.0*sqrt(-p/3.0)*cos((1.0/3.0)*acos( ((3.0*q)/(2.0*p)) * sqrt(-3.0/p)));
		double t_2 = 2.0*sqrt(-p/3.0)*cos((1.0/3.0)*acos(( (3.0*q)/(2.0*p)) * sqrt(-3.0/p))-(2.0*ecl::pi)/3.0);
		double t_3 = 2.0*sqrt(-p/3.0)*cos((1.0/3.0)*acos(( (3.0*q)/(2.0*p)) * sqrt(-3.0/p))-(4.0*ecl::pi)/3.0);
		intercepts.resize(3);
		intercepts << t_1+shift, t_2+shift, t_3+shift;
	}
	return intercepts;
}

CartesianPoint2d Intersection< LinearFunction >::operator()(const LinearFunction &f, const LinearFunction &g) ecl_throw_decl(StandardException) {
	CartesianPoint2d point;
	double a_0 = f.coefficients()[0];
	double b_0 = f.coefficients()[1];
	double a_1 = g.coefficients()[0];
	double b_1 = g.coefficients()[1];
	if ( isApprox(b_0, b_1) ) { // should use some epsilon distance here.
		last_operation_failed = true;
		ecl_throw(StandardException(LOC,OutOfRangeError,"Functions are collinear, no intersection possible."));
	} else {
		point.x((a_0 - a_1)/(b_1 - b_0));
		point.y(f(point.x()));
	}
    return point;
}

double Maximum< LinearFunction >::operator()(
        const double& x_begin, const double& x_end, const LinearFunction &function) {
    double max = function(x_begin);
    double test_max = function(x_end);
    if ( test_max > max ) {
        max = test_max;
    }
    return max;
}

double Maximum<CubicPolynomial>::operator()(
        const double& x_begin, const double& x_end, const CubicPolynomial& cubic) {
    // 3a_3x^2 + 2a_2x + a_1 = 0
    double max = cubic(x_begin);
    double test_max = cubic(x_end);
    if ( test_max > max ) {
        max = test_max;
    }
    CubicPolynomial::Coefficients coefficients = cubic.coefficients();
    double a = 3*coefficients[3];
    double b = 2*coefficients[2];
    double c = coefficients[1];
    if ( a == 0 ) {
    	double root = -c/b;
		if ( ( root > x_begin ) && ( root < x_end ) ) {
			test_max = cubic(root);
			if ( test_max > max ) {
				max = test_max;
			}
		}
    } else {
		double sqrt_term = b*b-4*a*c;
		if ( sqrt_term > 0 ) {
			double root = ( -b + sqrt(b*b-4*a*c))/(2*a);
			if ( ( root > x_begin ) && ( root < x_end ) ) {
				test_max = cubic(root);
				if ( test_max > max ) {
					max = test_max;
				}
			}
			root = ( -b - sqrt(b*b-4*a*c))/(2*a);
			if ( ( root > x_begin ) && ( root < x_end ) ) {
				test_max = cubic(root);
				if ( test_max > max ) {
					max = test_max;
				}
			}
		}
    }
    return max;
}

double Minimum< LinearFunction >::operator()(
        const double& x_begin, const double& x_end, const LinearFunction &function) {
    double min = function(x_begin);
    double test_min = function(x_end);
    if ( test_min < min ) {
        min = test_min;
    }
    return min;
}

double Minimum<CubicPolynomial>::operator()(
        const double& x_begin, const double& x_end, const CubicPolynomial& cubic) {
    // 3a_3x^2 + 2a_2x + a_1 = 0
    double min = cubic(x_begin);
    double test_min = cubic(x_end);
    if ( test_min < min ) {
        min = test_min;
    }
    CubicPolynomial::Coefficients coefficients = cubic.coefficients();
    double a = 3*coefficients[3];
    double b = 2*coefficients[2];
    double c = coefficients[1];
    if ( a == 0 ) {
    	double root = -c/b;
		if ( ( root > x_begin ) && ( root < x_end ) ) {
			test_min = cubic(root);
			if ( test_min < min ) {
				min = test_min;
			}
		}
    } else {
		double sqrt_term = b*b-4*a*c;
		if ( sqrt_term > 0 ) {
			double root = ( -b + sqrt(b*b-4*a*c))/(2*a);
			if ( ( root > x_begin ) && ( root < x_end ) ) {
				test_min = cubic(root);
				if ( test_min < min ) {
					min = test_min;
				}
			}
			root = ( -b - sqrt(b*b-4*a*c))/(2*a);
			if ( ( root > x_begin ) && ( root < x_end ) ) {
				test_min = cubic(root);
				if ( test_min < min ) {
					min = test_min;
				}
			}
		}
    }
    return min;
}

} // namespace ecl
