/**
 * @file /include/ecl/geometry/cubic_spline.hpp
 *
 * @brief Storage container for a cubic spline interpolation.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_CUBIC_SPLINE_HPP_
#define ECL_GEOMETRY_CUBIC_SPLINE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "polynomial.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/macros.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/concepts/containers.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/utilities/blueprints.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

class CubicSpline;

namespace blueprints {

class C2CubicSpline;
class DerivativeHeuristicCubicSpline;

} // namespace blueprints

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/
/**
 * @brief Blueprint factory for cubic splines.
 *
 * Generates various blueprints that instantiate or configure cubic splines from commonly
 * used algorithms.
 *
 * @sa @ref ecl::CubicSpline "CubicSpline".
 */
template<>
class ECL_PUBLIC BluePrintFactory< CubicSpline > {
    public:
        /**
         * @brief Cubic spline blueprint satisfying y, y' and y'' continuity with y''_0 = y''_f = 0.
         *
         * This is a special case of the ContinuousDerivatives blueprint that uses
         * y''_0 = y''_f = 0 boundary conditions instead. It is more commonly referred to
         * as the <i>natural cubic spline</i>.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @return C2CubicSpline : the resulting blueprint.
         */
        static blueprints::C2CubicSpline Natural(const Array<double>& x_set, const Array<double>& y_set);
        /**
         * @brief Cubic spline blueprint satisfying y, y' and y'' continuity requirements.
         *
         * Generates a blueprint for creating cubic splines on a set of data
         * satisfying the constraints of continuity for y, y' and y''.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @param ydot_0 : initial derivative value (boundary constraint).
         * @param ydot_f : final derivative value (boundary constraint).
         * @return C2CubicSpline : the resulting blueprint.
         */
        static blueprints::C2CubicSpline ContinuousDerivatives(
                const Array<double>& x_set, const Array<double>& y_set, const double ydot_0, const double ydot_f);
        /**
         * @brief Cubic spline blueprint derived from a y' heuristic.
         *
         * Generates a blueprint for creating cubic splines on a set of data
         * satisfying a heuristic that automatically generates derivatives at the
         * via points.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @param ydot_0 : initial derivative value (boundary constraint).
         * @param ydot_f : final derivative value (boundary constraint).
         * @return DerivativeHeuristicCubicSpline : the resulting blueprint.
         */
        static blueprints::DerivativeHeuristicCubicSpline DerivativeHeuristic(
                const Array<double>& x_set, const Array<double>& y_set, const double ydot_0, const double ydot_f);

        virtual ~BluePrintFactory() {}
};

/*****************************************************************************
** Interface [CubicSpline]
*****************************************************************************/
/**
 * @brief  Storage container for a cubic spline interpolation.
 *
 * Storage container for a cubic spline interpolation. This is a very standard
 * cubic spline container. Use the corresponding blueprints for
 * construction of the splines.
 *
 * There are various ways of constructing the cubic spline, though almost always
 * a C1 or C2 continuous function is desired. To achieve this, either a heuristic
 * or a C2 constraint can be fixed, resulting in a derivation for the spline's
 * functions.
 *
 * @sa @ref ecl::Polynomial "Polynomial", @ref splinesGeometry "Math::Splines.
 **/
class ECL_PUBLIC CubicSpline : public BluePrintFactory< CubicSpline > {
    public:
        /******************************************
        ** Constructors
        *******************************************/
        /**
         *  @brief Default constructor.
         *
         * Default constructor.
         **/
        CubicSpline () {};
        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the BluePrintFactory.
         * @code
         *  Array<double> x_set(5);
         *  Array<double> y_set(5);
         *  x_set = 0.0, 1.0, 2.0, 3.0, 4.0;
         *  y_set = 1.0, 2.0, 1.0, 3.0, 4.0;
         * CubicSpline cubic_spline = CubicSpline::Natural(x_set,y_set);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * CubicSpline cubic_spline;
         * cubic_spline = CubicSpline::Natural(x_set,y_set);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprints concept (refer to ecl_concepts' documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         *
         * @sa ecl::utilities::BluePrintFactory<CubicSpline>.
         */
        template<typename Derived>
        CubicSpline(const BluePrint< Derived > &blueprint) {
            blueprint.implementApply(*this);
        }
        virtual ~CubicSpline () {};

        /******************************************
        ** Access
        *******************************************/
        friend class blueprints::DerivativeHeuristicCubicSpline;
        friend class blueprints::C2CubicSpline;
        /**
         * @brief Spline function.
         *
         * Extract the spline function's value at the indicated location.
         * @param x : the domain value.
         * @return double : the spline function's value.
         * @exception : StandardException : throws if input x value is outside the spline range [debug mode only].
         */
        double operator()(const double &x) const ecl_assert_throw_decl(StandardException);
        /**
         * @brief Spline derivative.
         *
         * Extract the derivative of the spline at the indicated value.
         * @param x : the domain value.
         * @return double : the derivative of the spline.
         * @exception : StandardException : throws if input x value is outside the spline range [debug mode only].
         */
        double derivative(double x) const ecl_assert_throw_decl(StandardException);
        /**
         * @brief Spline second derivative.
         *
         * Extract the second derivative of the spline.
         * @param x : the domain value.
         * @return double : the second derivative of the spline.
         * @exception : StandardException : throws if input x value is outside the spline range [debug mode only].
         */
        double dderivative(double x) const ecl_assert_throw_decl(StandardException);

        /**
         * @brief The discretised domain for this spline.
         *
         * This returns the array of discretised time values that define
         * the domains of each polynomial within the spline.
         * @return const Array<double>& : the discretised domain.
         */
        const Array<double>& domain() { return discretised_domain; }
        /**
         * @brief The polynomial sequence.
         *
         * This returns a handle to the array of cubic polynomials
         * that make up the spline.
         *
         * @return const Array<CubicPolynomial>& : the polynomial sequence.
         */
        const Array<CubicPolynomial>& polynomials() { return cubic_polynomials; }

        /******************************************
        ** Streaming
        *******************************************/
        /**
         * @brief Streaming output insertion operator for for cubic splines.
         *
         * Streaming output insertion operator for cubic splines.
         *
         * @tparam OutputStream : the type of stream being used.
         *
         * @param ostream : the output stream being used.
         * @param cubic_spline : the cubic spline.
         * @return OutputStream : the output stream.
         */
        template <typename OutputStream>
        friend OutputStream& operator << (OutputStream &ostream, const CubicSpline &cubic_spline);

    private:
        Array<double> discretised_domain;      // N+1 x_i's
        // Would normally use pointers here, but the polynomials have fixed storage
        // size, the copy cost is small and the access times will be faster this way.
        Array<CubicPolynomial> cubic_polynomials;   // N polynomials
};

/*****************************************************************************
** Implementation [CubicSpline][Streaming]
*****************************************************************************/

template <typename OutputStream>
OutputStream& operator << (OutputStream &ostream, const CubicSpline &cubic_spline) {

	ecl_compile_time_concept_check(StreamConcept<OutputStream>);

    for ( unsigned int i = 0; i < cubic_spline.cubic_polynomials.size(); ++i ) {
        ostream << cubic_spline.cubic_polynomials[i] << "\n";
    }
    ostream.flush();
    return ostream;
}

/*****************************************************************************
** BluePrints
*****************************************************************************/

namespace blueprints {

/*****************************************************************************
** Interface [C2CubicSpline]
*****************************************************************************/

/**
 * @brief  Blueprint for generating a cubic spline satisfying C2 constraints.
 *
 * Blueprint for generating a cubic spline on a set of data given the constraint
 * of continuity of y, y' and y''.
 *
 * @sa @ref ecl::CubicSpline "CubicSpline".
 *
 **/
class ECL_PUBLIC C2CubicSpline : public ecl::BluePrint<C2CubicSpline> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::CubicSpline base_type;
        /**
         * @brief Default constructor.
         *
         * Default constructor (only utilised by the blueprint compile time assert).
         */
        C2CubicSpline() {};

        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * The constructor uses the input data sets to generate second derivatives
         * at each via point so that a cubic spline will guarantee continuity.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @param ydot_0 : initial derivative value (boundary constraint).
         * @param ydot_f : final derivative value (boundary constraint).
         * @exception : StandardException : throws if it failed to construct [debug mode only].
         */
        C2CubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set,
                                        const double ydot_0, const double ydot_f) ecl_assert_throw_decl(ecl::StandardException);

        virtual ~C2CubicSpline() {};
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * The constructor uses the input data sets to generate second derivatives
         * at each via point so that a cubic spline will guarantee continuity. This is
         * a special case that does not use boundary derivative values to establish
         * the result. Rather it automatically sets the boundary second derivatives
         * to zero. This is more commonly known as the <i>natural cubic spline</i>.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @exception : StandardException : throws if it failed to construct [debug mode only].
         */
        C2CubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set) ecl_assert_throw_decl(ecl::StandardException);

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new cubic spline generated from the supplied data set.
         *
         * @return CubicSpline : a copy of the generated cubic spline.
         */
        ecl::CubicSpline instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the data set to generate the configuration required for the cubic spline.
         *
         * @param spline : the spline to configure from the supplied data set.
         */
        void apply(ecl::CubicSpline& spline) const;

    private:
        ecl::Array<double> x_data;
        ecl::Array<double> y_data;
        ecl::Array<double> yddot_data;
};

/*****************************************************************************
** Interface [DerivativeHeuristicCubicSpline]
*****************************************************************************/

/**
 * @brief  Blueprint for generating a cubic spline satisfying C2 constraints.
 *
 * Generates a blueprint for creating cubic splines on a set of data
 * satisfying a heuristic that automatically generates y' values at the
 * via points. These are later used to generate the connecting cubics.
 *
 * The heuristic finds the average of the linear slopes
 * connected to each via point. This specifies the desired slope for that
 * via point.
 *
 * @sa @ref ecl::CubicSpline "CubicSpline".
 *
 **/
class ECL_PUBLIC DerivativeHeuristicCubicSpline : public ecl::BluePrint<DerivativeHeuristicCubicSpline> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::CubicSpline base_type;
        /**
         * @brief Default constructor.
         *
         * Default constructor (only utilised by the blueprint compile time assert).
         */
        DerivativeHeuristicCubicSpline() {};

        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * This constructor uses the heuristic to specify the slope at via points.
         * These are stored and later used to generate the connecting cubics.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @param ydot_0 : initial derivative value (boundary constraint).
         * @param ydot_f : final derivative value (boundary constraint).
         *
         * @exception : StandardException : throws if it failed to construct [debug mode only].
         */
        DerivativeHeuristicCubicSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set,
                        const double ydot_0, const double ydot_f) ecl_assert_throw_decl(ecl::StandardException);

        virtual ~DerivativeHeuristicCubicSpline() {};
        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new cubic spline generated from the supplied data set.
         *
         * @return CubicSpline : a copy of the generated cubic spline.
         */
        ecl::CubicSpline instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the data set to generate the configuration required for the cubic spline.
         *
         * @param spline : the spline to configure from the supplied data set.
         */
        void apply(ecl::CubicSpline& spline) const;

    private:
        ecl::Array<double> x_data;
        ecl::Array<double> y_data;
        ecl::Array<double> ydot_data;
};

} // namespace blueprints
} // namespace ecl

#endif /* ECL_GEOMETRY_CUBIC_SPLINE_HPP_ */
