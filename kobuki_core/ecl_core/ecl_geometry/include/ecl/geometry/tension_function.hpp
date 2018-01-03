/**
 * @file /include/ecl/geometry/tension_function.hpp
 *
 * @brief Representations for the tension function..
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_TENSION_FUNCTION_HPP_
#define ECL_GEOMETRY_TENSION_FUNCTION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "function_math.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/macros.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/utilities/blueprints.hpp>
#include <ecl/formatters/floats.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

class TensionFunction;

namespace blueprints {

class TensionSecondDerivativeInterpolation;

} // namespace blueprints

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/
/**
 * @brief Blueprint factory for tension functions.
 *
 * Generates various blueprints that instantiate or configure tension functions with
 * commonly used configurations.
 *
 * @sa @ref ecl::TensionFunction "TensionFunction".
 */
template<>
class ECL_PUBLIC BluePrintFactory< TensionFunction > {
    public:
        /**
         * @brief Blueprint for interpolating a tension function between two end points using second derivatives.
         *
         * Generates a blueprint for creating tension functions generated from
         * value and second derivative constraints on the endpoints.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial value.
         * @param yddot_i : initial curvature.
         * @param x_f : final domain value.
         * @param y_f : final value.
         * @param yddot_f : final curvature.
         *
         * @return TensionSecondDerivativeInterpolation : the resulting blueprint.
         *
         * @sa ecl::blueprints::TensionSecondDerivativeInterpolation
         */
        static blueprints::TensionSecondDerivativeInterpolation Interpolation(const double x_i, const double y_i, const double yddot_i,
                const double x_f, const double y_f, const double yddot_f);

        virtual ~BluePrintFactory() {};
};

/*****************************************************************************
** Interface [TensionFunction]
*****************************************************************************/
/**
 * @brief Representation of a tension function.
 *
 * This is a hyperbolic function
 * often used in spline interpolations that parameterises the 'tension' of
 * a curve interpolation between points. At low tension it behaves like a
 * cubic polynomial, while at high tension, it tends towards a piecewise
 * smooth series of linear functions.
 *
 * <b>Theoretical Notes:</b>
 *
 * Tension functions are derived from the following constraints on
 * an interval [x_i, x_i+1]:
 * @verbatim
  f'''' - tau^2 f'' = 0
  f(x_i)     = y_i
  f(x_i+1)   = y_i+1
  f''(x_i)   = z_i
  f''(x_i+1) = z_i+1
  @endverbatim
 * This yields a C2 continuous function with hyperbolic terms (not a polynomial)
 * that ranges from looking like a cubic interpolation at low tensions
 * (tau -> 0) and a linearly blended interpolation at high tensions.
 **/
class ECL_PUBLIC TensionFunction : public BluePrintFactory< TensionFunction > {
    public:
        /*********************
        ** Constructors
        **********************/
        /**
         * @brief Default constructor.
         *
		 * Don't really need this, but things like vectors and array containers need
		 * it so they can reserve the appropriate storage.
         */
        TensionFunction() {}
        virtual ~TensionFunction() {}
        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the BluePrintFactory.
         * @code
         * TensionFunction f = TensionFunction::Interpolation(2.0,0.0,0.0,3.0,1.0,0.0);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * TensionFunction f;
         * f = TensionFunction::Interpolation(2.0,0.0,0.0,3.0,1.0,0.0);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprints concept (refer to ecl_concepts' documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         *
         * @sa ecl::utilities::BluePrintFactory<TensionFunction>.
         */
        template<typename Derived>
        TensionFunction(const BluePrint< Derived > &blueprint) {
            blueprint.implementApply(*this);
        }

        /*********************
        ** Derivatives
        **********************/
        /**
         * @brief Generates the derivative for a certain tension at the specified point.
         *
         * Calculates the value of the derivative for a certain tension at the specified point.
         *
         * @param tau : the tension parameter.
         * @param x : the point at which you wish to calculate the value for.
         *
         * @return double : the value of the derivative at x for tension tau.
         */
        double derivative(const double &tau, const double &x) const;

        /**
         * @brief Generates the 2nd derivative for a certain tension at the specified point.
         *
         * Calculates the value of the 2nd derivative for a certain tension at the specified point.
         *
         * @param tau : the tension parameter.
         * @param x : the point at which you wish to calculate the value for.
         *
         * @return double : the value of the 2nd derivative at x for tension tau.
         */
        double dderivative(const double &tau, const double &x) const;

        /*********************
        ** Accessors
        **********************/
        /**
         * @brief Calculates the value for a certain tension at the specified point.
         *
         * Calculates the value for a certain tension at the specified point.
         *
         * @param tau : the tension parameter.
         * @param x : the point at which you wish to calculate the value for.
         *
         * @return double : the value of the function at x for tension tau.
         **/
        double operator ()(const double &tau, const double &x) const;

        /*********************
        ** Friends
        **********************/
        friend class blueprints::TensionSecondDerivativeInterpolation;


        template <typename OutputStream>
        friend OutputStream& operator << (OutputStream &ostream, const TensionFunction &function);

    private:
        double z_0, z_f; // yddot_0, yddot_f
        double x_0, x_f;
        double y_0, y_f;
};


/*****************************************************************************
** Streaming Operator [TensionFunction]
*****************************************************************************/
/**
 * @brief Streaming output insertion operator for tension functions.
 *
 * Streaming output insertion operator for tension functions.
 *
 * @tparam OutputStream : the type of stream being used.
 *
 * @param ostream : the output stream being used.
 * @param function : the tension function.
 * @return OutputStream : the output stream.
 */
template <typename OutputStream>
OutputStream& operator << (OutputStream &ostream, const TensionFunction &function)
{
	ecl_compile_time_concept_check(StreamConcept<OutputStream>);

    Format<double> format;
    format.precision(2);
    double h = function.x_f - function.x_0;
    if ( ( function.z_0 == 0.0 ) && ( function.z_f == 0.0 ) ) {
        ostream << "0.0";
    } else {
        ostream << "{";
        if ( function.z_0 != 0.0 ) {
            ostream << format(function.z_0) << "*sinh[tau(";
            ostream << format(function.x_f) << "-x)] + ";
        }
        if ( function.z_f != 0.0 ) {
            ostream << format(function.z_f) << "*sinh[tau(x-";
            ostream << format(function.x_0) << ")]}/[tau^2 sinh(";
            ostream << format(function.x_f - function.x_0) << "*tau)]";
        }
    }
    ostream << "\n";
    ostream << "        + (";
    ostream << format(function.y_0/h) << "-";
    ostream << format(function.z_0/h) << "/tau^2)(";
    ostream << format(function.x_f) << "-x)\n";
    ostream << "                    + (";
    ostream << format(function.y_f/h) << "-";
    ostream << format(function.z_f/h) << "/tau^2)(x-";
    ostream << format(function.x_0) << ")\n";
    ostream.flush();

    return ostream;
}

/*****************************************************************************
** BluePrints
*****************************************************************************/

namespace blueprints {

/*****************************************************************************
** Interface [TensionSecondDerivativeInterpolation]
*****************************************************************************/

/**
 * @brief  Blueprint for interpolating a tension function between end point conditions.
 *
 * Blueprint for interpolating a tension function between two end point conditions. That is,
 * the resulting y(x) should satisfy the following conditions:
 *
 * @code
 *   y(x_i)  = y_i
 *   y(x_f)  = y_f
 *   y''(x_i) = y''_i
 *   y''(x_f) = y''_f
 * @endcode
 *
 * @sa @ref ecl::TensionFunction "TensionFunction".
 *
 **/
class ECL_PUBLIC TensionSecondDerivativeInterpolation : public ecl::BluePrint<TensionSecondDerivativeInterpolation> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::TensionFunction base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the tension function.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial value.
         * @param yddot_i : initial curvature.
         * @param x_f : final domain value.
         * @param y_f : final value.
         * @param yddot_f : final curvature.
         */
        TensionSecondDerivativeInterpolation(const double x_i, const double y_i, const double yddot_i,
                const double x_f, const double y_f, const double yddot_f) :
                    x_initial(x_i),
                    y_initial(y_i),
                    yddot_initial(yddot_i),
                    x_final(x_f),
                    y_final(y_f),
                    yddot_final(yddot_f)
        {}
        virtual ~TensionSecondDerivativeInterpolation() {}

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new tension function generated from the input boundary conditions.
         *
         * @return TensionFunction : a copy of the generated function.
         */
        ecl::TensionFunction instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate the tension function.
         *
         * @param function : the function to configure from the supplied boundary conditions.
         */
        void apply(base_type& function) const;

    private:
        double x_initial, y_initial, yddot_initial;
        double x_final, y_final, yddot_final;
};


}; // namespace blueprints
}; // namespace ecl

#endif /*ECL_GEOMETRY_TENSION_FUNCTION_HPP_*/
