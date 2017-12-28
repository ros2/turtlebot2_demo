/**
 * @file /include/ecl/geometry/smooth_linear_spline.hpp
 *
 * @brief Spline sequence generated from linear segments with ramped corners.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SMOOTH_LINEAR_SPLINE_HPP_
#define ECL_SMOOTH_LINEAR_SPLINE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/data_exception.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "polynomial.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [SmoothLinearSpline]
*****************************************************************************/
/**
 * @brief  Storage container for a smoothed linear spline interpolation.
 *
 * These interpolations connect waypoints linearly and add a smoothed corner (using
 * a quintic polynomial) to connect each linear segment. The shape of the corners
 * is parameterised with a maximum curvature parameter (acceleration).
 *
 * @ref splinesGeometry "Math::Splines.
 **/
class ecl_geometry_PUBLIC SmoothLinearSpline {
public:
	/**
	 * @brief Default constructor.
	 *
	 * Don't really need this, but things like vectors and array containers need
	 * it so they can reserve the appropriate storage.
	 */
	SmoothLinearSpline() {}
    /**
     * @brief Constructor that properly configures the ramped spline.
     *
     * The constructor uses the input data sets to initialise the segmentation.
     * Corners are constrained by the maximum curvature (acceleration) - if
     * construction should fail (not enough space between the waypoints) then
     * this method will throw an exception indicating failure.
     * It is accompanied with an int type data element that indicates the
     * number of the segment that failed to construct. Failure typically occurs
     * because the maximum curvature constraint at the corner was broken. Try
     * increasing the maximum curvature constraint or increasing the time
     * duration of the specified segment to ensure success a
     * second time around.
     *
     * @param a_max : the maximum bound (absolute) on the acceleration.
     * @param x_data : set of data on the domain axis.
     * @param y_data : set of values on the range axis.
     * @exception : DataException : throws if the spline could not be constructed because of broken a_max constraint.
     *
     * The int data for the exception represents the element in the set which could not make
     * a corner without breaking the acceleration constraint. Note that if you have a data set of size 5 (i.e.
     * elements 0-4), then it will only throw between 1 and 3, as this is where smoothed corners are built.
     */
    SmoothLinearSpline(const Array<double>& x_data, const Array<double>& y_data, double a_max) throw (DataException<int>);

    virtual	~SmoothLinearSpline() {}

	/**
	 * @brief Spline function.
	 *
	 * Extract the spline function's value at the indicated location.
	 *
	 * @param x : the domain value.
	 * @return double : the spline function's value.
	 * @exception : StandardException : throws if x is outside the spline range [debug mode only].
	 */
	double operator()(const double &x) const ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Spline derivative.
	 *
	 * Extract the derivative of the spline at the indicated value.
	 *
	 * @param x : the domain value.
	 *
	 * @return double : the derivative of the spline.
	 * @exception : StandardException : throws if x is outside the spline range [debug mode only].
	 */
	double derivative(const double &x) const ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Spline second derivative.
	 *
	 * Extract the second derivative of the spline.
	 * @param x : the domain value.
	 * @return double : the second derivative of the spline.
	 * @exception : StandardException : throws if x is outside the spline range [debug mode only].
	 */
	double dderivative(const double &x) const ecl_assert_throw_decl(StandardException);

	/**
	 * @brief The discretised domain for this spline.
	 *
	 * This returns the array of discretised time values that define
	 * the domains of each polynomial within the spline.
	 *
	 * @return const Array<double>& : the discretised domain.
	 */
	const Array<double>& domain() const { return discretised_domain; }

	/*********************
	 * Static Constructor
	**********************/
	/**
	 * @brief Static construction that interpolates on a set of waypoints.
	 *
	 * Static construction that interpolates on a set of waypoints.
	 *
     * @param x_data : set of data on the domain axis.
     * @param y_data : set of values on the range axis.
     * @param a_max : the maximum bound (absolute) on the acceleration.
     *
     * @exception : DataException : throws if the spline could not be constructed, data is the segment # that failed.
	 */
	static SmoothLinearSpline Interpolation(const Array<double>& x_data, const Array<double>& y_data, double a_max) throw (DataException<int>)
	{
		try {
			return SmoothLinearSpline(x_data, y_data, a_max);
		} catch ( DataException<int> &e ) {
			throw DataException<int>(LOC,e);
		}
		return SmoothLinearSpline();  // avoid a warning because of the rethrown exception.
	}

	/******************************************
	 * Streaming
	*******************************************/
	/**
	 * @brief Streaming output insertion operator for smoothed linear splines.
	 *
	 * Streaming output insertion operator for smoothed linear splines. This
	 * simply lists the spline segments and corners (linear functions and
	 * quintic polynomials) in algebraic form.
	 *
	 * @tparam OutputStream : the type of stream being used.
	 *
	 * @param ostream : the output stream being used.
	 * @param smooth_linear_spline : the tension spline.
	 * @return OutputStream : the output stream.
	 */
	template <typename OutputStream>
	friend OutputStream& operator << (OutputStream &ostream, const SmoothLinearSpline &smooth_linear_spline);

private:
	Array<double> discretised_domain;       // 2*N points (2 end points + 2*(N-1) corner points)
	Array<LinearFunction> segments;	        // N linear segments
	Array<QuinticPolynomial> corners;       // N-1 corners
};

/*****************************************************************************
** Implementation [SmoothLinearSpline][Streaming]
*****************************************************************************/

template <typename OutputStream>
OutputStream& operator << (OutputStream &ostream, const SmoothLinearSpline &smooth_linear_spline) {

	ecl_compile_time_concept_check(StreamConcept<OutputStream>);

    ostream << smooth_linear_spline.segments[0] << "\n";
    for ( unsigned int i = 1; i < smooth_linear_spline.segments.size(); ++i ) {
        ostream << smooth_linear_spline.corners[i-1] << "\n";
        ostream << smooth_linear_spline.segments[i] << "\n";
    }
    ostream.flush();
    return ostream;
}

} // namespace ecl

#endif /* ECL_SMOOTH_LINEAR_SPLINE_HPP_ */
