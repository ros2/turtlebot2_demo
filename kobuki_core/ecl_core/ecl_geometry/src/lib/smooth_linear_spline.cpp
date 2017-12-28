/**
 * @file /src/lib/smooth_linear_spline.cpp
 *
 * @brief Implementation for ramped splines.
 *
 * Implementation for ramped splines.
 *
 * @date July 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cmath>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/geometry/smooth_linear_spline.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

SmoothLinearSpline::SmoothLinearSpline(const Array<double>& x_data, const Array<double>& y_data, double a_max) throw (DataException<int>) {

	if ( x_data.size() != y_data.size() ) {
		throw DataException<int>(LOC,OutOfRangeError,"Input domain and range sets were not the same size.", 0);
	}

    /*********************
    ** Basic Parameters
    **********************/
	// n = number of linear segments
	// 2*n = number of points (2 end points + 2*(n-1) corners) remember we're including the polynomial corners in this calculation
    unsigned int n = x_data.size()-1;

    /******************************************
	** Storage
	*******************************************/
    segments.resize(n);
    corners.resize(n-1);
    discretised_domain.resize(2*n);

    /******************************************
	** Build Segments
	*******************************************/
    for ( unsigned int i = 0; i < n; ++i ) {
    	segments[i] = LinearFunction::Interpolation(x_data[i],y_data[i],x_data[i+1],y_data[i+1]);
    }
	discretised_domain[0] = x_data[0];    // Initial point
	discretised_domain[2*n-1] = x_data[n];  // Last point

	/******************************************
	** Build Corners (this might throw)
	*******************************************/
    for ( unsigned int i = 1; i < n; ++i ) {
    	if ( segments[i-1].coefficients()[1] == segments[i].coefficients()[1] ) { // slope identical
    		// Just set the quintic to be a linear function.
    		corners[i-1].coefficients() << segments[i-1].coefficients()[0], segments[i-1].coefficients()[1], 0.0, 0.0, 0.0, 0.0;
    		discretised_domain[1+2*(i-1)] = x_data[i];
    		discretised_domain[1+2*(i-1)+1] = x_data[i];
    	} else {
			for ( unsigned int j = 1; j <= 5; ++j ) {
				double x_l, x_r;
				if ( i == 1 ) { // First one, we can use the whole segment
					x_l = x_data[i] - j*(x_data[i]-x_data[i-1])/5;
				} else {
					x_l = x_data[i] - j*(x_data[i]-x_data[i-1])/10;
				}
				if ( i == (n-1) ) { // Last one, we can use the whole segment
					x_r = x_data[i] + j*(x_data[i+1]-x_data[i])/5;
				} else {
					x_r = x_data[i] + j*(x_data[i+1]-x_data[i])/10;
				}
				double y_l = segments[i-1](x_l);
				double y_r = segments[i](x_r);
				double ydot_l = segments[i-1].derivative(x_l);
				double ydot_r = segments[i].derivative(x_r);
				corners[i-1] = QuinticPolynomial::Interpolation(x_l,y_l,ydot_l,0.0,
																x_r,y_r,ydot_r,0.0);
				if ( ( fabs( Maximum<CubicPolynomial>()(x_l,x_r,corners[i-1].derivative().derivative())) < fabs(a_max) ) &&
						( fabs( Minimum<CubicPolynomial>()(x_l,x_r,corners[i-1].derivative().derivative())) < fabs(a_max) ) ) {
		    		discretised_domain[1+2*(i-1)] = x_l;
		    		discretised_domain[1+2*(i-1)+1] = x_r;
					break;
				}
				/*********************
				** Debugging
				**********************/
//				if ( i == 1 ) {
//					double max = fabs( Maximum<CubicPolynomial>()(x_l,x_r,corners[i-1].derivative().derivative()));
//					if ( fabs( Minimum<CubicPolynomial>()(x_l,x_r,corners[i-1].derivative().derivative())) > max ) {
//						max = fabs( Minimum<CubicPolynomial>()(x_l,x_r,corners[i-1].derivative().derivative()));
//					}
//					std::cout << "Max: " << max << std::endl;
//				}

				if ( j == 5 ) { // Cannot exceed more than half the length of the segment.
					throw DataException<int>(LOC,ConstructorError,"Max acceleration bound could not be satisfied at corner specified by the data element.",i);
				}
			}
    	}
    }
}

double SmoothLinearSpline::operator()(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    if ( index % 2 == 0 ) { // linear
    	return segments[index/2](x);
    } else { // quintic
    	return corners[(index-1)/2](x);
    }
}

double SmoothLinearSpline::derivative(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    if ( index % 2 == 0 ) { // linear
    	return segments[index/2].derivative(x);
    } else { // quintic
    	return corners[(index-1)/2].derivative(x);
    }
}

double SmoothLinearSpline::dderivative(const double &x) const ecl_assert_throw_decl(StandardException) {
    ecl_assert_throw( ( ( x >= discretised_domain.front() ) && ( x <= discretised_domain.back() ) ), StandardException(LOC,OutOfRangeError) );
    int index = 0;
    while ( x > discretised_domain[index+1] ) {
        ++index;
    }
    if ( index % 2 == 0 ) { // linear
    	return segments[index/2].dderivative(x);
    } else { // quintic
    	return corners[(index-1)/2].dderivative(x);
    }
}

} // namespace ecl




