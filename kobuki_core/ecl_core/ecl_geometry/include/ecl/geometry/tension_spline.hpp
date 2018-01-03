/**
 * @file /include/ecl/geometry/tension_spline.hpp
 *
 * @brief Storage container for a tension spline interpolation.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_TENSION_SPLINE_HPP_
#define ECL_GEOMETRY_TENSION_SPLINE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "tension_function.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/macros.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/concepts/containers.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/utilities/blueprints.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

class TensionSpline;

namespace blueprints {

class C2TensionSpline;

} // namespace blueprints

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/
/**
 * @brief Blueprint factory for tension splines.
 *
 * Generates various blueprints that instantiate or configure tension splines from commonly
 * used algorithms.
 *
 * @sa @ref ecl::TensionSpline "TensionSpline".
 */
template<>
class ECL_PUBLIC BluePrintFactory< TensionSpline > {
public:
	/**
	 * @brief Tension spline blueprint satisfying y, y' and y'' continuity with y''_0 = y''_f = 0.
	 *
	 * This is the natural form of the spline interpolation.
	 *
	 * @param x_set : set of data on the domain axis.
	 * @param y_set : set of values on the range axis.
	 * @param tau : the tension parameter.
	 * @return C2TensionSpline : the resulting blueprint.
	 * @exception : StandardException : throws if configuration arguments are not valid [debug mode only].
	 *
	 * @sa @ref ecl::blueprints::C2TensionSpline "C2TensionSpline"
	 */
	static blueprints::C2TensionSpline Natural(const Array<double>& x_set, const Array<double>& y_set,
			const double &tau) ecl_assert_throw_decl(StandardException);

	virtual ~BluePrintFactory() {};
};

/*****************************************************************************
** Interface [TensionSpline]
*****************************************************************************/
/**
 * @brief  Storage container for a tension spline interpolation.
 *
 * These interpolations
 * are very similar to cubic splines, but allow an extra parameter for configuration
 * called the <i>tension</i> of the interpolation. The tension variable effectively
 * tightens the curve among the data points, resulting in a spline interpolation
 * that looks very similar to the linear blended interpolations. When the tension
 * approaches zero, this interpolation defaults to a cubic spline interpolation.
 *
 * @sa @ref ecl::CubicSpline "CubicSpline", @ref splinesGeometry "Math::Splines.
 **/
class ecl_geometry_PUBLIC TensionSpline : public BluePrintFactory< TensionSpline > {
    public:
        /******************************************
        ** Typedefs
        *******************************************/
        typedef BluePrintFactory< TensionSpline > Factory; /**< @brief Generates blueprints for this class. **/

        /******************************************
        ** Constructors
        *******************************************/
        /**
         *  @brief Default constructor.
         *
		 * Don't really need this, but things like vectors and array containers need
		 * it so they can reserve the appropriate storage.
         **/
        TensionSpline () {};
        virtual ~TensionSpline() {};
        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the BluePrintFactory.
         * @code
         * double tension = 4.0;
         * Array<double> x_set(5);
         * Array<double> y_set(5);
         * x_set = 0.0, 1.0, 2.0, 3.0, 4.0;
         * y_set = 1.0, 2.0, 1.0, 3.0, 4.0;
         * TensionSpline tension_spline = TensionSpline::Natural(x_set,y_set,tension);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * tension_spline = TensionSpline::Natural(x_set,y_set,tension);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprints concept (refer to ecl_concepts' documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         *
         * @sa ecl::BluePrintFactory<TensionSpline>.
         */
        template<typename Derived>
        TensionSpline(const BluePrint< Derived > &blueprint) {
            blueprint.implementApply(*this);
        }

        /******************************************
        ** Access
        *******************************************/
        friend class blueprints::C2TensionSpline;

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
         * @return const Array<double>& : the discretised domain.
         */
        const Array<double>& domain() const { return discretised_domain; }

        /******************************************
        ** Streaming
        *******************************************/
        /**
         * @brief Streaming output insertion operator for for tension splines.
         *
         * Streaming output insertion operator for tension splines. This lists in algebraic
         * form the sequence of tension functions constituting the spline.
         *
         * @tparam OutputStream : the type of stream being used.
         *
         * @param ostream : the output stream being used.
         * @param tension_spline : the tension spline.
         * @return OutputStream : the output stream.
         */
        template <typename OutputStream>
        friend OutputStream& operator << (OutputStream &ostream, const TensionSpline &tension_spline);

    private:
        Array<double> discretised_domain;           // N+1 x_i's
        Array<TensionFunction> functions;   // N tension_functions
        double tension;
};

/*****************************************************************************
** Interface [Minimum/Maximum][TensionSpline]
*****************************************************************************/

/*****************************************************************************
** Implementation [TensionSpline][Streaming]
*****************************************************************************/

template <typename OutputStream>
OutputStream& operator << (OutputStream &ostream, const TensionSpline &tension_spline) {

	ecl_compile_time_concept_check(ecl::StreamConcept<OutputStream>);

    for ( unsigned int i = 0; i < tension_spline.functions.size(); ++i ) {
        ostream << tension_spline.functions[i] << "\n";
    }
    ostream.flush();
    return ostream;
}

/*****************************************************************************
** BluePrints
*****************************************************************************/

namespace blueprints {

/*****************************************************************************
** Interface [C2TensionSpline]
*****************************************************************************/

/**
 * @brief  Blueprint for generating a tension spline satisfying C2 constraints.
 *
 * Blueprint for generating a tension spline on a set of data given the constraint
 * of continuity of y, y' and y''.
 *
 * @sa @ref ecl::TensionSpline "TensionSpline".
 *
 **/
class ecl_geometry_PUBLIC C2TensionSpline : public ecl::BluePrint<C2TensionSpline> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::TensionSpline base_type;
        /**
         * @brief Default constructor.
         *
         * Default constructor (only utilised by the blueprint compile time assert).
         */
        C2TensionSpline() {};
        ~C2TensionSpline() {};

        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * The constructor uses the input data sets along with the assumption of
         * zero boundary conditions on the second derivatives to generate a full
         * set of second derivatives at each via point that will guarantee c2
         * continuity of the spline.
         *
         * @param x_set : set of data on the domain axis.
         * @param y_set : set of values on the range axis.
         * @param tau : the tension parameter.
         * @exception : StandardException : throws if configuration arguments are not valid [debug mode only].
         */
        C2TensionSpline(const ecl::Array<double>& x_set, const ecl::Array<double>& y_set, const double &tau) ecl_assert_throw_decl(ecl::StandardException);

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new tension spline generated from the supplied data set.
         *
         * @return TensionSpline : a copy of the generated tension spline.
         */
        base_type instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the data set to generate the configuration required for the tension spline.
         *
         * @param spline : the spline to configure from the supplied data set.
         */
        void apply(base_type& spline) const;

    private:
        ecl::Array<double> x_data;
        ecl::Array<double> y_data;
        ecl::Array<double> yddot_data;
        double tension;
};

} // namespace blueprints
} // namespace ecl

#endif /* ECL_GEOMETRY_TENSION_SPLINE_HPP_ */
