/**
 * @file /src/lib/covariance_ellipsoid.cpp
 *
 * @brief Implementation for covariance ellipsoids.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/linear_algebra.hpp>
#include "../../include/ecl/statistics/covariance_ellipsoid.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Using
*****************************************************************************/

using ecl::linear_algebra::Matrix2f;
using ecl::linear_algebra::Vector2f;
using ecl::linear_algebra::Matrix3f;
using ecl::linear_algebra::Vector3f;

using ecl::linear_algebra::Matrix2d;
using ecl::linear_algebra::Vector2d;
using ecl::linear_algebra::Matrix3d;
using ecl::linear_algebra::Vector3d;

/*****************************************************************************
** Implementation [CovarianceEllipsoid2f]
*****************************************************************************/
CovarianceEllipsoid<float,2>::CovarianceEllipsoid() :
	ellipse_lengths(Vector2f::Ones()),
	ellipse_axes(Matrix2f::Identity())
{}

CovarianceEllipsoid<float,2>::CovarianceEllipsoid(const ecl::linear_algebra::Matrix2f &M) {
	compute(M);
}
void CovarianceEllipsoid<float,2>::compute(const ecl::linear_algebra::Matrix2f &M) {

	// CHECK: symmetricity of M

	/*********************
	** Eigenvalues
	**********************/
	float a = M(0,0);
	float b = M(0,1);
	float c = M(1,0);
	float d = M(1,1);

	float tmp = sqrtf((a+d)*(a+d)/4 - a*d + b*c);
    ellipse_lengths << sqrtf((a+d)/2 + tmp), sqrtf((a+d)/2 - tmp);

    // CHECK: all ellipse_lengths are >= 0 (positive semi-definite check)
    // CHECK: ordered in decreasing order of magnitude

    /*********************
	** Eigenvectors
	**********************/
    if( c != 0 ) {
        ellipse_axes(0,0) = ellipse_lengths(0)*ellipse_lengths(0)-d;
        ellipse_axes(1,0) = c;
        ellipse_axes(0,1) = ellipse_lengths(1)*ellipse_lengths(1)-d;
        ellipse_axes(1,1) = c;
    } else if( b != 0 ) {
    	ellipse_axes(0,0) = b;
    	ellipse_axes(1,0) = ellipse_lengths(0)*ellipse_lengths(0)-a;
    	ellipse_axes(0,1) = b;
    	ellipse_axes(1,1) = ellipse_lengths(1)*ellipse_lengths(1)-a;
    }
    else {
    	if ( a > d ) {
			ellipse_axes << 1, 0,
						    0, 1;
    	} else {
    		ellipse_axes << 0, 1,
						    1, 0;
    	}
    }
    /*********************
	** Normalise Evectors
	**********************/
    ellipse_axes.block<2,1>(0,0).normalize();
    ellipse_axes.block<2,1>(0,1).normalize();
}

double CovarianceEllipsoid<float,2>::rotation() {
	return atan2f(ellipse_axes(1,0), ellipse_axes(0,0));
}

Vector2f CovarianceEllipsoid<float,2>::intercepts() {

	Vector2f intercept_magnitudes;
	float t;
	float angle = rotation();

	/*********************
	** Parametric Form
	**********************/
	//  here t is a parameter and theta is the rotation angle
	//
	//	x = lambda1 cos(t)cos(theta) - lambda2 sin(t)sin(theta)
	//  y = lambda1 cos(t)sin(theta) + lambda2 sin(t)cos(theta)
	//
	// To find the intersection with the x axis, set y = 0, then
	//      t = arctan[ - (lambda1/lambda2) tan(theta) ]
	// To find the intersection with the y axis, set x = 0, then
	//      t = arctan[ (lambda1/lambda2) cot(theta) ]

	/*********************
	** X Axis
	**********************/
	t = atan2f( -(ellipse_lengths(0)/ellipse_lengths(1))*tanf(angle), 1.0 );
	intercept_magnitudes(0) = fabsf(ellipse_lengths(0)*cosf(t)*cosf(angle) - ellipse_lengths(1)*sinf(t)*sinf(angle));
	/*********************
	** Y Axis
	**********************/
	t = atan2f( (ellipse_lengths(0)/ellipse_lengths(1)), tanf(angle) ); // Could this get dodgy with tan going to infinity?
	intercept_magnitudes(1) = fabsf(ellipse_lengths(0)*cosf(t)*sinf(angle) + ellipse_lengths(1)*sinf(t)*sinf(angle));

	return intercept_magnitudes;
}

/*****************************************************************************
** Implementation [CovarianceEllipsoid2d]
*****************************************************************************/
CovarianceEllipsoid<double,2>::CovarianceEllipsoid() :
	ellipse_lengths(Vector2d::Ones()),
	ellipse_axes(Matrix2d::Identity())
{}

CovarianceEllipsoid<double,2>::CovarianceEllipsoid(const ecl::linear_algebra::Matrix2d &M) {
	compute(M);
}
void CovarianceEllipsoid<double,2>::compute(const ecl::linear_algebra::Matrix2d &M) {

	// CHECK: symmetricity of M

	/*********************
	** Eigenvalues
	**********************/
	double a = M(0,0);
	double b = M(0,1);
	double c = M(1,0);
	double d = M(1,1);

    double tmp = sqrt((a+d)*(a+d)/4 - a*d + b*c);
    ellipse_lengths << sqrt((a+d)/2 + tmp), sqrt((a+d)/2 - tmp);

    // CHECK: all ellipse_lengths are >= 0 (positive semi-definite check)
    // CHECK: ordered in decreasing order of magnitude

    /*********************
	** Eigenvectors
	**********************/
    if( c != 0 ) {
        ellipse_axes(0,0) = ellipse_lengths(0)*ellipse_lengths(0)-d;
        ellipse_axes(1,0) = c;
        ellipse_axes(0,1) = ellipse_lengths(1)*ellipse_lengths(1)-d;
        ellipse_axes(1,1) = c;
    } else if( b != 0 ) {
    	ellipse_axes(0,0) = b;
    	ellipse_axes(1,0) = ellipse_lengths(0)*ellipse_lengths(0)-a;
    	ellipse_axes(0,1) = b;
    	ellipse_axes(1,1) = ellipse_lengths(1)*ellipse_lengths(1)-a;
    }
    else {
    	if ( a > d ) {
			ellipse_axes << 1, 0,
						    0, 1;
    	} else {
    		ellipse_axes << 0, 1,
						    1, 0;
    	}
    }
    /*********************
	** Normalise Evectors
	**********************/
    ellipse_axes.block(0,0,2,1).normalize();
    ellipse_axes.block(0,1,2,1).normalize();
}

double CovarianceEllipsoid<double,2>::rotation() {
	return atan2(ellipse_axes(1,0), ellipse_axes(0,0));
}

Vector2d CovarianceEllipsoid<double,2>::intercepts() {

	Vector2d intercept_magnitudes;
	double t;
	double angle = rotation();

	/*********************
	** Parametric Form
	**********************/
	//  here t is a parameter and theta is the rotation angle
	//
	//	x = lambda1 cos(t)cos(theta) - lambda2 sin(t)sin(theta)
	//  y = lambda1 cos(t)sin(theta) + lambda2 sin(t)cos(theta)
	//
	// To find the intersection with the x axis, set y = 0, then
	//      t = arctan[ - (lambda1/lambda2) tan(theta) ]
	// To find the intersection with the y axis, set x = 0, then
	//      t = arctan[ (lambda1/lambda2) cot(theta) ]

	/*********************
	** X Axis
	**********************/
	t = atan2( -(ellipse_lengths(0)/ellipse_lengths(1))*tan(angle), 1.0 );
	intercept_magnitudes(0) = fabs(ellipse_lengths(0)*cos(t)*cos(angle) - ellipse_lengths(1)*sin(t)*sin(angle));
	/*********************
	** Y Axis
	**********************/
	t = atan2( (ellipse_lengths(0)/ellipse_lengths(1)), tan(angle) ); // Could this get dodgy with tan going to infinity?
	intercept_magnitudes(1) = fabs(ellipse_lengths(0)*cos(t)*sin(angle) + ellipse_lengths(1)*sin(t)*sin(angle));

	return intercept_magnitudes;
}



/*****************************************************************************
** Implementation [CovarianceEllipsoid3f]
*****************************************************************************/
CovarianceEllipsoid<float,3>::CovarianceEllipsoid() :
	ellipse_lengths(Vector3f::Ones()),
	ellipse_axes(Matrix3f::Identity())
{}

CovarianceEllipsoid<float,3>::CovarianceEllipsoid(const ecl::linear_algebra::Matrix3f &M, const bool sort) {
	compute(M, sort);
}
void CovarianceEllipsoid<float,3>::compute(const ecl::linear_algebra::Matrix3f &M, const bool sort) {

	Eigen::EigenSolver<Matrix3f> esolver(M);

	ellipse_lengths[0] = sqrtf(esolver.pseudoEigenvalueMatrix()(0,0));
	ellipse_lengths[1] = sqrtf(esolver.pseudoEigenvalueMatrix()(1,1));
	ellipse_lengths[2] = sqrtf(esolver.pseudoEigenvalueMatrix()(2,2));
	ellipse_axes = esolver.pseudoEigenvectors ();

	if ( sort ) {
		// Note that sorting of eigenvalues may end up with left-hand coordinate system.
		// So here we correctly sort it so that it does end up being righ-handed and normalised.
		ecl::linear_algebra::Vector3f c0 = ellipse_axes.block<3,1>(0,0);  c0.normalize();
		ecl::linear_algebra::Vector3f c1 = ellipse_axes.block<3,1>(0,1);  c1.normalize();
		ecl::linear_algebra::Vector3f c2 = ellipse_axes.block<3,1>(0,2);  c2.normalize();
		ecl::linear_algebra::Vector3f cc = c0.cross(c1);
		if (cc.dot(c2) < 0) {
			ellipse_axes << c1, c0, c2;
		  double e = ellipse_lengths[0];  ellipse_lengths[0] = ellipse_lengths[1];  ellipse_lengths[1] = e;
		} else {
			ellipse_axes << c0, c1, c2;
		}
	}
}

CovarianceEllipsoid<double,3>::CovarianceEllipsoid() :
	ellipse_lengths(Vector3d::Ones()),
	ellipse_axes(Matrix3d::Identity())
{}

CovarianceEllipsoid<double,3>::CovarianceEllipsoid(const ecl::linear_algebra::Matrix3d &M, const bool sort) {
	compute(M,sort);
}
void CovarianceEllipsoid<double,3>::compute(const ecl::linear_algebra::Matrix3d &M, const bool sort) {
	Eigen::EigenSolver<Matrix3d> esolver(M);

	ellipse_lengths[0] = sqrt(esolver.pseudoEigenvalueMatrix()(0,0));
	ellipse_lengths[1] = sqrt(esolver.pseudoEigenvalueMatrix()(1,1));
	ellipse_lengths[2] = sqrt(esolver.pseudoEigenvalueMatrix()(2,2));
	ellipse_axes = esolver.pseudoEigenvectors ();

	if ( sort ) {
		// Note that sorting of eigenvalues may end up with left-hand coordinate system.
		// So here we correctly sort it so that it does end up being righ-handed and normalised.
		ecl::linear_algebra::Vector3d c0 = ellipse_axes.block<3,1>(0,0);  c0.normalize();
		ecl::linear_algebra::Vector3d c1 = ellipse_axes.block<3,1>(0,1);  c1.normalize();
		ecl::linear_algebra::Vector3d c2 = ellipse_axes.block<3,1>(0,2);  c2.normalize();
		ecl::linear_algebra::Vector3d cc = c0.cross(c1);
		if (cc.dot(c2) < 0) {
			ellipse_axes << c1, c0, c2;
		  double e = ellipse_lengths[0];  ellipse_lengths[0] = ellipse_lengths[1];  ellipse_lengths[1] = e;
		} else {
			ellipse_axes << c0, c1, c2;
		}
	}
}


} // namespace ecl

