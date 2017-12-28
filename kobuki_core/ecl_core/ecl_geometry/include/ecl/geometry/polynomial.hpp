/**
 * @file /include/ecl/geometry/polynomial.hpp
 *
 * @brief Representations for polynomial functions.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_POLYNOMIALS_HPP_
#define ECL_GEOMETRY_POLYNOMIALS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include "cartesian_point.hpp"
#include "function_math.hpp"
#include "pascals_triangle.hpp"
#include <ecl/config/macros.hpp>
#include <ecl/concepts/macros.hpp>
#include <ecl/concepts/streams.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/utilities/blueprints.hpp>
#include <ecl/formatters/floats.hpp>
#include <iostream>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward declarations
*****************************************************************************/

template <unsigned int N> class Polynomial;

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/

/**
 * @brief Primary template for the @ref ecl::Polynomial "Polynomial" blueprint factories.
 *
 * Primary template for the polynomial blueprint factories. This is in fact empty since it covers
 * polynomials of all orders. Particular polynomial blueprint factories are specified in the
 * specialisations.
 *
 * @tparam N : degree of the polynomials to create blueprints for.
 *
 * @sa @ref ecl::Polynomial "Polynomial", BluePrintFactory< ecl::Polynomial<3> >.
 */
template<unsigned int N>
class BluePrintFactory< Polynomial<N> > {
public:
	BluePrintFactory() {};
	~BluePrintFactory() {};
};

/*****************************************************************************
** Interface [Polynomial]
*****************************************************************************/
/**
 * @brief Representation of a polynomial function of n-th degree.
 *
 * Representation of a polynomial function of n-th order. Use the template
 * parameter to specify the degree of the polynomial. The representation is
 * defined by an array of coefficients which are given in the form
 * a_0 -> a_n where p(x) = a_0 + a_1 x + ....
 *
 * <b>Comma Initialisation:</b>
 *
 * The polynomial uses a an ecl array (ecl_containers) for storage,
 * so comma initialisation is possible.
 * @code
 * // Comma Initialisation
 * Polynomial<5> p;
 * p.coefficients() = 1,2,3,4,5,6;
 * cout << p << endl; // 1.00 + 2.00x + 3.00x^2 + 4.00x^3 + 5.00x^4 + 6.00x^5
 * @endcode
 *
 * <b>Blueprint Initialisation:</b>
 *
 * Lightweight blueprints can also be used to initialise/assign the polynomial:
 *
 * @code
 * CubicPolynomial cubic = CubicPolynomial::Interpolation(2.0,0.0,0.0,3.0,1.0,0.0);
 * @endcode
 *
 * <b>Utility Functions:</b>
 *
 * - Maximum<LinearFunction>
 * - Minimum<LinearFunction>
 * - Maximum<CubicPolynomial>
 * - Minimum<CubicPolynomial>
 *
 * @tparam N : the degree of the polynomial (i.e. a_0 + ... + a_i x^i + ... + a_N x^N)
 *
 * @sa @ref polynomialsGeometry "Math::Polynomials".
 **/
template <unsigned int N>
class ecl_geometry_PUBLIC Polynomial : public BluePrintFactory< Polynomial<N> >, public FunctionMath< Polynomial<N> > {
    public:
        /*********************
        ** Convenience
        **********************/
        typedef Array<double,N+1> Coefficients; /**< @brief The coefficient container storage type. **/

        /*********************
        ** C&D
        **********************/
        /**
         * @brief Default constructor.
         *
         * This does not initialise the polynomial, use with the
         * coefficients() accessor with comma initialisation operator to
         * conveniently set the coefficients.
         **/
        Polynomial() : coeff(Coefficients::Constant(0.0)) {};

        /**
         * @brief Blueprint constructor.
         *
         * Constructor that allows automatic generation from an
         * existing blueprint. This can be used simply in the following manner
         * for any static element belonging to the BluePrintFactory.
         * @code
         * CubicPolynomial cubic = CubicPolynomial::Interpolation(2.0,0.0,0.0,3.0,1.0,0.0);
         * @endcode
         * Since this is not explicit, it will also allow assignment.
         * @code
         * CubicPolynomial cubic; // Alternatively Polynomial<3>
         * cubic = CubicPolynomial::Interpolation(2.0,0.0,0.0,3.0,1.0,0.0);
         * @endcode
         *
         * This will emit a compile time failure if the template argument does
         * not conform to the blueprints concept (refer to ecl_concepts' documentation).
         *
         * @param blueprint : the blue print to use to generate this instance.
         *
         * @sa ecl::BluePrintFactory<LinearFunction>,
         *     ecl::BluePrintFactory<CubicPolynomial>,
         *     ecl::BluePrintFactory<QuinticPolynomial>.
         */
        template<typename Derived>
        Polynomial(const BluePrint< Derived > &blueprint) {
            blueprint.implementApply(*this);
        }
        virtual ~Polynomial() {}

        /*********************
        ** Transform
        **********************/
        /**
         * @brief Horizontal shift transform.
         *
         * Shifts the polynomial along the x axis by the specified offset.
         * A negative offset will shift the polynomial to the left, while
         * a positive offset will shit the polynomial to the right.
         *
         * @code
         * Polynomial<2> quadratic; // x^2
         * quadratic.shift(2);      // x^2 -> (x-2)^2 (graph shifted right by 2)
         * @endcode
         *
         * @param shift : the amount of x-shift (-ve -> left, +ve -> right).
         **/
        void shift_horizontal(const double &shift);
//        void stretch_horizontal(double multiplier);

        /*********************
        ** Derivatives
        **********************/
        /**
         * @brief Generates the derivative polynomial.
         *
         * Calculates and returns a copy of the derivative polynomial.
         *
         * @return Polynomial<N-1> : the derivative polynomial.
         */
        Polynomial<N-1> derivative() const {
            Polynomial<N-1> derivative_polynomial;
            typename Polynomial<N-1>::Coefficients &derivative_coefficients = derivative_polynomial.coefficients();
            for ( unsigned int i = 0; i < N; ++i ) {
                derivative_coefficients[i] = (i+1)*coeff[i+1];
            }
            return derivative_polynomial;
        }
        /**
         * @brief Access the derivative directly.
         *
         * Access the values of the derivative directly.
         *
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the derivative at x.
         */
        double derivative(const double &x) const;
        /**
         * @brief Access the second derivative directly.
         *
         * Access the values of the second derivative directly.
         *
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the second derivative at x.
         */
        double dderivative(const double &x) const;

        /*********************
        ** Accessors
        **********************/
        /**
         * @brief Handle to the coefficient array, use to initialise the polynomial.
         *
         * This returns a handle to the coefficient array. Use this with
         * the comma initialiser to conveniently set the polynomial.
         *
         * @code
         * Polynomial<5> p;
         * p.coefficients() = 1,2,3,4,5,6;
         * cout << p << endl; // 1.00 + 2.00x + 3.00x^2 + 4.00x^3 + 5.00x^4 + 6.00x^5
         * @endcode
         *
         * @return Coefficients& : reference to the co-efficient array.
         **/
        Coefficients& coefficients() { return coeff; };
        /**
         * @brief Non-modifiable handle to the coefficient array.
         *
         * @return const Coefficients& : non-modifiable reference to the co-efficient array.
         **/
        const Coefficients& coefficients() const { return coeff; };

        /**
         * @brief Access the value of the polynomial at a certain point.
         *
         * Access the value of the polynomial at a certain point.
         *
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the polynomial at x.
         **/
        double operator ()(const double &x) const;

        /**
         * @brief Streaming output insertion operator for polynomials.
         *
         * Streaming output insertion operator for polynomials.
         *
         * @tparam OutputStream : the type of stream being used.
         * @tparam Degree : the order of the polynomial being inserted.
         *
         * @param ostream : the output stream being used.
         * @param polynomial : the polynomial
         * @return OutputStream : the output stream.
         */
        template <typename OutputStream, unsigned int Degree>
        friend OutputStream& operator << (OutputStream &ostream, const Polynomial<Degree> &polynomial);

    private:
        Coefficients coeff;
};

/**
 * @brief Specialisation for the zero-th order polynomial.
 *
 * Represents a zero'th order polynomial (scalar). It is necessary to handle this
 * separately as the derivatives do not return lower degree polynomials.
 *
 * @sa Polynomial, @ref polynomialsGeometry "Math::Polynomials".
 **/
template <>
class ECL_PUBLIC Polynomial<0> {
    public:
        /*********************
        ** Convenience
        **********************/
        typedef Array<double,1> Coefficients; /**< @brief The coefficient container storage type. **/

        /*********************
        ** C&D
        **********************/
        /**
         * @brief Default constructor.
         *
         * This initialises the scalar coefficient for the zero'th polynomial
         * to zero.
         **/
        Polynomial() : coeff(Coefficients::Constant(0.0)) {};
        virtual ~Polynomial() {};

        /*********************
        ** Transform
        **********************/
        /**
         * @brief Horizontal shift transform.
         *
         * Normally, shifts the polynomial along the x axis by the specified offset,
         * but in the case of this specialisation, does not change the polynomial.
         **/
        void shift_horizontal(const double& /* shift */) {}; // Does nothing.

        /*********************
        ** Derivatives
        **********************/
        /**
         * @brief Derivative of a zero'th order polynomial is always zero.
         *
         * Derivative of a zero'th order polynomial is always zero.
         *
         * @return Polynomial<0> : the zero polynomial.
         */
        Polynomial<0> derivative() const {
            return Polynomial<0>();
        }
        /**
         * @brief Access the derivative directly (always returns 0).
         *
         * Access the values of the derivative directly (always returns 0)..
         *
         * @return double : derivative of a scalar is always 0.0.
         */
        double derivative(const double& /* x */ ) const {
            return 0.0;
        };
        /**
         * @brief Access the second derivative directly (always returns 0)..
         *
         * Access the values of the second derivative directly (always returns 0)..
         *
         * @return double : 2nd derivative of a scalar is always 0.0.
         */
        double dderivative(const double& /* x */) const {
            return 0.0;
        };

        /*********************
        ** Accessors
        **********************/
        /**
         * @brief Handle to the coefficient array, use to initialise the polynomial.
         *
         * This returns a handle to the coefficient array. Use this with
         * the comma initialiser to conveniently set the polynomial.
         *
         * @code
         * Polynomial<0> p;
         * p.coefficients() = 1;
         * cout << p << endl; // 1.00
         * @endcode
         *
         * @return Coefficients& : reference to the co-efficient array.
         **/
        Coefficients& coefficients() { return coeff; };
        /**
         * @brief Non-modifiable handle to the coefficient array.
         *
         * @return const Coefficients& : non-modifiable reference to the co-efficient array.
         **/
        const Coefficients& coefficients() const { return coeff; };

        /**
         * @brief Access the value of the polynomial at the specified point.
         *
         * Access the value of the polynomial at the specified point.
         *
         * @return double : the value of a scalar is always a_0.
         **/
        double operator ()(const double& /* x */) const {
            return coeff[0];
        };

    private:
        Coefficients coeff;
};

/*****************************************************************************
** Typedefs
*****************************************************************************/

typedef Polynomial<1> LinearFunction;		 /**< @brief Mathematical term for 1st order polynomials. **/
typedef Polynomial<2> QuadraticPolynomial;   /**< @brief Mathematical term for 2nd order polynomials. **/
typedef Polynomial<3> CubicPolynomial;       /**< @brief Mathematical term for 3rd order polynomials. **/
typedef Polynomial<5> QuinticPolynomial;     /**< @brief Mathematical term for 5th order polynomials. **/

/*****************************************************************************
 * Implementation [Polynomial - Transform]
 ****************************************************************************/
template <unsigned int N>
void Polynomial<N>::shift_horizontal(const double &shift)
{
    PascalsTriangle<N> pascals_triangle;
    typename PascalsTriangle<N>::const_iterator iter;
    double tmp;

    for ( unsigned int i = 0; i < N; ++i ) {
        tmp = -1*shift;
        int j = i+1;
        for ( iter = (pascals_triangle.begin(i)+1); iter != pascals_triangle.end(i); ++iter ) { // skip the first one
            coeff[i] += (*iter)*tmp*coeff[j];
            tmp *= (-1*shift);
            ++j;
        }
    }
}
/*****************************************************************************
 * Implementation [Polynomial - Access]
 ****************************************************************************/
template <unsigned int N>
double Polynomial<N>::operator()(const double &x) const
{
    double tmp = x;
    double value = coeff[0];
    for ( unsigned int i = 1; i <= N; ++i ) {
         value += coeff[i]*tmp;
         tmp *= x;
    }
    return value;
}
template <unsigned int N>
double Polynomial<N>::derivative(const double &x) const {

    if ( N > 0 ) {
        return derivative()(x);
    } else {
        return 0.0;
    }
}

template <unsigned int N>
double Polynomial<N>::dderivative(const double &x) const {

    if ( N > 1 ) {
        return derivative().derivative()(x);
    } else {
        return 0.0;
    }
}

/*****************************************************************************
** Streaming Operator [Polynomial]
*****************************************************************************/

template <typename OutputStream, unsigned int Degree>
OutputStream& operator << (OutputStream &ostream, const Polynomial<Degree> &polynomial)
{
	ecl_compile_time_concept_check(StreamConcept<OutputStream>);

	Format<double> format;
    format.precision(2);

    ostream << format(polynomial.coeff[0]);
    for (unsigned int i = 1; i <= Degree; ++i) {
        ostream << " + " << format(polynomial.coeff[i]) << "x^" << i;
    }
    ostream.flush();

    return ostream;
}

/*****************************************************************************
** BluePrints
*****************************************************************************/

namespace blueprints {

/*****************************************************************************
** Interface [LinearInterpolation]
*****************************************************************************/

/**
 * @brief  Blueprint for interpolating a linear function connecting end point conditions.
 *
 * Blueprint for interpolating a linear function connecting two points. That is,
 * the resulting linear function y(x) should satisfy the following conditions:
 *
 * @code
 *   y(x_i)  = y_i
 *   y(x_f)  = y_f
 * @endcode
 *
 * @sa @ref ecl::LinearFunction.
 *
 **/
class ecl_geometry_PUBLIC LinearInterpolation : public ecl::BluePrint<LinearInterpolation> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::LinearFunction base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the linear function.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial polynomial value.
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         */
        LinearInterpolation(const double x_i, const double y_i, const double x_f, const double y_f) :
                    x_initial(x_i),
                    y_initial(y_i),
                    x_final(x_f),
                    y_final(y_f)
        {}

        virtual ~LinearInterpolation() {}

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new linear function generated from the input boundary conditions.
         *
         * @return LinearFunction : a copy of the generated function.
         */
        ecl::LinearFunction instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate coefficients for the given polynomial.
         * @param function : the linear function to configure from the supplied boundary conditions.
         */
        void apply(ecl::LinearFunction& function) const;

    private:
        double x_initial, y_initial;
        double x_final, y_final;
};

/*****************************************************************************
** Interface [LinearPointSlope]
*****************************************************************************/
/**
 * @brief  Blueprint for generating a linear function from slope and point pair.
 *
 * That is, the resulting linear function y(x) should satisfy the following conditions:
 *
 * @code
 *   (y(x_f)-y(x))  = slope*(x_f-x)
 *   y(x_f)  = y_f
 * @endcode
 *
 * @sa @ref ecl::LinearFunction.
 *
 **/
class ecl_geometry_PUBLIC LinearPointSlopeForm : public ecl::BluePrint<LinearPointSlopeForm> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::LinearFunction base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the linear function.
         *
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         * @param s : slope
         */
        LinearPointSlopeForm(const double x_f, const double y_f, const double s) :
                    slope(s),
                    x_final(x_f),
                    y_final(y_f)
        {}

        virtual ~LinearPointSlopeForm() {}

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new linear function generated from the input conditions.
         *
         * @return LinearFunction : a copy of the generated function.
         */
        ecl::LinearFunction instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate coefficients for the given polynomial.
         * @param function : the linear function to configure from the input conditions.
         */
        void apply(ecl::LinearFunction& function) const;

    private:
        double slope;
        double x_final, y_final;
};


/*****************************************************************************
** Interface [CubicDerivativeInterpolation]
*****************************************************************************/

/**
 * @brief  Blueprint for interpolating a cubic polynomial between end point conditions.
 *
 * Blueprint for interpolating a cubic polynomial between two end point conditions. That is,
 * the resulting polynomial y(x) should satisfy the following conditions:
 *
 * @code
 *   y(x_i)  = y_i
 *   y(x_f)  = y_f
 *   y'(x_i) = y'_i
 *   y'(x_f) = y'_f
 * @endcode
 *
 * @sa @ref ecl::Polynomial "Polynomial".
 *
 **/
class ecl_geometry_PUBLIC CubicDerivativeInterpolation : public ecl::BluePrint<CubicDerivativeInterpolation> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::CubicPolynomial base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the cubic.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial polynomial value.
         * @param ydot_i : initial polynomial slope.
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         * @param ydot_f : final polynomial slope.
         */
        CubicDerivativeInterpolation(const double x_i, const double y_i, const double ydot_i,
                const double x_f, const double y_f, const double ydot_f) :
                    x_initial(x_i),
                    y_initial(y_i),
                    ydot_initial(ydot_i),
                    x_final(x_f),
                    y_final(y_f),
                    ydot_final(ydot_f)
        {}

        virtual ~CubicDerivativeInterpolation() {}
        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new cubic generated from the input boundary conditions.
         *
         * @return CubicPolynomial : a copy of the generated cubic.
         */
        ecl::CubicPolynomial instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate coefficients for the given polynomial.
         * @param polynomial : the polynomial to configure from the supplied boundary conditions.
         */
        void apply(ecl::CubicPolynomial& polynomial) const;

    private:
        double x_initial, y_initial, ydot_initial;
        double x_final, y_final, ydot_final;
};
/*****************************************************************************
** Interface [CubicSecondDerivativeInterpolation]
*****************************************************************************/

/**
 * @brief  Blueprint for interpolating a cubic polynomial between end point conditions.
 *
 * Blueprint for interpolating a cubic polynomial between two end point conditions. That is,
 * the resulting polynomial y(x) should satisfy the following conditions:
 *
 * @code
 *   y(x_i)  = y_i
 *   y(x_f)  = y_f
 *   y''(x_i) = y''_i
 *   y''(x_f) = y''_f
 * @endcode
 *
 * @sa @ref ecl::Polynomial "Polynomial".
 *
 **/
class ecl_geometry_PUBLIC CubicSecondDerivativeInterpolation : public ecl::BluePrint<CubicSecondDerivativeInterpolation>  {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::CubicPolynomial base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the cubic.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial polynomial value.
         * @param yddot_i : initial polynomial curvature.
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         * @param yddot_f : final polynomial curvature.
         */
        CubicSecondDerivativeInterpolation(const double x_i, const double y_i, const double yddot_i,
                const double x_f, const double y_f, const double yddot_f) :
                    x_initial(x_i),
                    y_initial(y_i),
                    yddot_initial(yddot_i),
                    x_final(x_f),
                    y_final(y_f),
                    yddot_final(yddot_f)
        {}

        virtual ~CubicSecondDerivativeInterpolation() {}

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new cubic generated from the input boundary conditions.
         *
         * @return CubicPolynomial : a copy of the generated cubic.
         */
        base_type instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate coefficients for the given polynomial.
         * @param polynomial : the polynomial to configure from the supplied boundary conditions.
         */
        void apply(base_type& polynomial) const;

    private:
        double x_initial, y_initial, yddot_initial;
        double x_final, y_final, yddot_final;
};

/*****************************************************************************
** Interface [QuinticInterpolation]
*****************************************************************************/

/**
 * @brief  Blueprint for interpolating a quintic polynomial between end point conditions.
 *
 * Blueprint for interpolating a quintic polynomial between two end point conditions. That is,
 * the resulting polynomial y(x) should satisfy the following conditions:
 *
 * @code
 *   y(x_i)  = y_i
 *   y'(x_i) = y'_i
 *   y''(x_i) = y''_i
 *   y(x_f)  = y_f
 *   y'(x_f) = y'_f
 *   y''(x_f) = y''_f
 * @endcode
 *
 * @sa @ref ecl::Polynomial "Polynomial".
 *
 **/
class ecl_geometry_PUBLIC QuinticInterpolation : public ecl::BluePrint<QuinticInterpolation> {
    public:
        /**
         * @brief Abstract representation of the class to be instantiated/configured.
         **/
        typedef ecl::QuinticPolynomial base_type;
        /**
         * @brief Constructor that properly configures/initialises the blueprint.
         *
         * Constructor that accepts the boundary constraints used to generate the cubic.
         *
         * @param x_i : initial domain value.
         * @param y_i : initial polynomial value.
         * @param ydot_i : initial polynomial slope.
         * @param yddot_i : initial polynomial curvature.
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         * @param ydot_f : final polynomial slope.
         * @param yddot_f : final polynomial curvature.
         */
        QuinticInterpolation(const double x_i, const double y_i, const double ydot_i, const double yddot_i,
                const double x_f, const double y_f, const double ydot_f, const double yddot_f) :
                    x_initial(x_i),
                    y_initial(y_i),
                    ydot_initial(ydot_i),
                    yddot_initial(yddot_i),
                    x_final(x_f),
                    y_final(y_f),
                    ydot_final(ydot_f),
                    yddot_final(yddot_f)
        {}

        virtual ~QuinticInterpolation() {}

        /**
         * @brief Instantiate a copy of the object that is blueprinted.
         *
         * Instantiates a new cubic generated from the input boundary conditions.
         *
         * @return QuinticPolynomial : a copy of the generated cubic.
         */
        ecl::QuinticPolynomial instantiate();

        /**
         * @brief Apply the blueprint to configure an existing object.
         *
         * Apply the boundary conditions to generate coefficients for the given polynomial.
         * @param polynomial : the polynomial to configure from the supplied boundary conditions.
         */
        void apply(ecl::QuinticPolynomial& polynomial) const;

    private:
        double x_initial, y_initial, ydot_initial, yddot_initial;
        double x_final, y_final, ydot_final, yddot_final;
};

}; // namespace blueprints

/*****************************************************************************
** BluePrintFactory
*****************************************************************************/

using blueprints::LinearInterpolation;
using blueprints::LinearPointSlopeForm;
using blueprints::CubicDerivativeInterpolation;
using blueprints::CubicSecondDerivativeInterpolation;
using blueprints::QuinticInterpolation;

/**
 * @brief Blueprint factory for linear functions.
 *
 * Generates various blueprints that instantiate or configure linear functions with
 * commonly used configurations. This class is inherited by the LinearFunction class,
 * consequently it is simpler to access these blueprints via the inheritance mechanisms
 * than to use this class directly. For example,
 *
 * @code
 * LinearFunction function = LinearFunction::Interpolation(0.0,0.0,1.0,2.0);
 * @endcode
 *
 * @sa @ref ecl::LinearFunction "LinearFunction".
 */
template<>
class BluePrintFactory< LinearFunction > {
public:
	/**
	 * @brief Blueprint for an interpolating a linear function connecting two points.
	 *
	 * @param x_i : initial domain value.
	 * @param y_i : initial polynomial value.
	 * @param x_f : final domain value.
	 * @param y_f : final polynomial value.
	 *
	 * @return LinearInterpolation : the resulting blueprint.
	 *
	 * @sa ecl::blueprints::LinearInterpolation
	 */
	static LinearInterpolation Interpolation(const double x_i, const double y_i, const double x_f, const double y_f);
	/**
	 * @brief Blueprint for an generating a linear function from slope/point form.
	 *
	 * @param x_f : final domain value.
	 * @param y_f : final polynomial value.
	 * @param slope : slope of the linear function
	 * @return LinearPointSlopeForm : the resulting blueprint.
	 */
	static LinearPointSlopeForm PointSlopeForm(const double x_f, const double y_f, const double slope);
	virtual ~BluePrintFactory() {}
};

/**
 * @brief Blueprint factory for cubic polynomials.
 *
 * Generates various blueprints that instantiate or configure cubic polynomials with
 * commonly used configurations. This class is inherited by the CubicPolynomial class,
 * consequently it is simpler to access these blueprints via the inheritance mechanisms
 * than to use this class directly. For example,
 *
 * @code
 * CubicPolynomial cubic = CubicPolynomial::DerivativeInterpolation(0.0,0.0,0.0,1.0,2.0,0.0);
 * CubicPolynomial cubic = CubicPolynomial::SecondDerivativeInterpolation(0.0,0.0,0.0,1.0,2.0,0.0);
 * @endcode
 *
 * @sa @ref ecl::CubicPolynomial "CubicPolynomial".
 */
template<>
class BluePrintFactory< CubicPolynomial > {
public:
	/**
	 * @brief Blueprint for an interpolating cubic between two end points using derivatives.
	 *
	 * Generates a blueprint for creating cubic polynomials generated from
	 * value and derivative constraints on the endpoints.
	 *
	 * @param x_i : initial domain value.
	 * @param y_i : initial polynomial value.
	 * @param ydot_i : initial polynomial slope.
	 * @param x_f : final domain value.
	 * @param y_f : final polynomial value.
	 * @param ydot_f : final polynomial slope.
	 *
	 * @return CubicDerivativeInterpolation : the resulting blueprint.
	 *
	 * @sa ecl::blueprints::CubicDerivativeInterpolation
	 */
	static CubicDerivativeInterpolation DerivativeInterpolation(const double x_i, const double y_i, const double ydot_i,
			const double x_f, const double y_f, const double ydot_f);

	/**
	 * @brief Blueprint for an interpolating cubic between two end points using second derivatives.
	 *
	 * Generates a blueprint for creating cubic polynomials generated from
	 * value and second derivative constraints on the endpoints.
	 *
	 * @param x_i : initial domain value.
	 * @param y_i : initial polynomial value.
	 * @param yddot_i : initial polynomial curvature.
	 * @param x_f : final domain value.
	 * @param y_f : final polynomial value.
	 * @param yddot_f : final polynomial curvature.
	 *
	 * @return CubicSecondDerivativeInterpolation : the resulting blueprint.
	 *
	 * @sa ecl::blueprints::CubicSecondDerivativeInterpolation
	 */
	static CubicSecondDerivativeInterpolation SecondDerivativeInterpolation(const double x_i, const double y_i, const double yddot_i,
			const double x_f, const double y_f, const double yddot_f);

	virtual ~BluePrintFactory() {}
};

/**
 * @brief Blueprint factory for quintic polynomials.
 *
 * Generates various blueprints that instantiate or configure quintic polynomials with
 * commonly used configurations. This class is inherited by the QuinticPolynomial class,
 * consequently it is simpler to access these blueprints via the inheritance mechanisms
 * than to use this class directly. For example,
 *
 * @code
 * QuinticPolynomial quintic = QuinticPolynomial::Interpolation(0.0,0.0,0.0,0.0,1.0,2.0,0.0,0.0);
 * @endcode
 *
 * @sa @ref ecl::QuinticPolynomial "QuinticPolynomial".
 */
template<>
class BluePrintFactory< QuinticPolynomial > {
    public:
        /**
         * @brief Blueprint for an interpolating a quintic between two end points using derivatives.
         *
         * Generates a blueprint for creating quintic polynomials generated from
         * the conditions imposed by initial and final points..
         *
         * @param x_i : initial domain value.
         * @param y_i : initial polynomial value.
         * @param ydot_i : initial polynomial slope.
         * @param yddot_i : initial polynomial curvature.
         * @param x_f : final domain value.
         * @param y_f : final polynomial value.
         * @param ydot_f : final polynomial slope.
         * @param yddot_f : final polynomial curvature.
         *
         * @return QuinticInterpolation : the resulting blueprint.
         *
         * @sa ecl::blueprints::QuinticInterpolation
         */
        static QuinticInterpolation Interpolation(const double x_i, const double y_i, const double ydot_i, const double yddot_i,
                const double x_f, const double y_f, const double ydot_f, const double yddot_f);

        virtual ~BluePrintFactory() {}
};


/*****************************************************************************
** Interface [Maximum|Minimum][Polynomial]
*****************************************************************************/
/**
 * @brief Mathematical maximum on a compact interval for linear functions.
 *
 * Mathematical maximum on a compact interval for linear functions (first
 * order polynomials).
 *
 * @sa Maximum, LinearFunction, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Maximum< LinearFunction > {
public:
	/**
	 * @brief Returns the maximum of a linear function.
	 *
	 * Returns the maximum of the specified linear function on the given compact interval.
	 *
	 * @param x_begin : beginning of the compact interval.
	 * @param x_end : end of the compact interval.
	 * @param function : the linear function.
	 * @return double : the maximum.
	 **/
	ECL_PUBLIC double operator()(const double& x_begin, const double& x_end, const LinearFunction &function);
	virtual ~Maximum() {}
};

/**
 * @brief Mathematical maximum on a compact interval for cubic polynomials.
 *
 * Mathematical maximum on a compact interval for cubic polynomials.
 *
 * @sa Maximum, CubicPolynomial, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Maximum<CubicPolynomial> {
public:
	/**
	 * @brief Returns the maximum of a cubic polynomial.
	 *
	 * Returns the maximum of the specified cubic polynomial on the given compact interval.
	 *
	 * @param x_begin : beginning of the compact interval.
	 * @param x_end : end of the compact interval.
	 * @param cubic : the cubic polynomial.
	 * @return double : the maximum.
	 */
	ECL_PUBLIC double operator()(const double& x_begin, const double& x_end, const CubicPolynomial& cubic);
	virtual ~Maximum() {}
};

/**
 * @brief Mathematical minimum on a compact interval for linear functions.
 *
 * Mathematical minimum on a compact interval for linear functions (first
 * order polynomials).
 *
 * @sa Minimum, LinearFunction, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Minimum< LinearFunction > {
public:
	/**
	 * @brief Returns the minimum of a linear function.
	 *
	 * Returns the minimum of the specified linear function on the given compact interval.
	 *
	 * @param x_begin : beginning of the compact interval.
	 * @param x_end : end of the compact interval.
	 * @param function : the linear function.
	 * @return double : the minimum.
	 */
	ECL_PUBLIC double operator()(const double& x_begin, const double& x_end, const LinearFunction &function);
	virtual ~Minimum() {}
};

/**
 * @brief Mathematical minimum on a compact interval for cubic polynomials.
 *
 * Mathematical minimum on a compact interval for cubic polynomials.
 *
 * @sa Minimum, CubicPolynomial, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Minimum<CubicPolynomial> {
public:
	/**
	 * @brief Returns the minimum of a cubic polynomial.
	 *
	 * Returns the minimum of the specified cubic polynomial on the given compact interval.
	 *
	 * @param x_begin : beginning of the compact interval.
	 * @param x_end : end of the compact interval.
	 * @param cubic : the cubic polynomial.
	 * @return double : the minimum.
	 */
	ECL_PUBLIC double operator()(const double& x_begin, const double& x_end, const CubicPolynomial& cubic);
	virtual ~Minimum() {}
};

/*****************************************************************************
** Interface [Intersection][LinearFunction]
*****************************************************************************/
/**
 * @brief Intersection of two linear functions.
 *
 * @sa Intersection, LinearFunction, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Intersection< LinearFunction > {
public:
	Intersection() : last_operation_failed(false) {}
	virtual ~Intersection() {}
	/**
	 * @brief Returns the intersection of two linear functions.
	 *
	 * @param f : linear function.
	 * @param g : linear function.
	 * @return CartesionPoint2d : the intersection point.
	 *
	 * @exception : StandardException : throws if functions are collinear.
	 **/
	ECL_PUBLIC CartesianPoint2d operator()(const LinearFunction& f, const LinearFunction& g) ecl_throw_decl(StandardException);

	/**
	 * @brief Boolean flag identifying if the last operation failed or not.
	 *
	 * Use this if you have disabled exceptions or don't wish to catch
	 * the exception thrown when linear functions are collinear.
	 */
	bool fail() const { return last_operation_failed; }

private:
	bool last_operation_failed;
};

/*****************************************************************************
** Interface [Synthetic Division]
*****************************************************************************/
/**
 * @brief Synthetic division between quadratic and a factor.
 *
 * Does long division of the polynomial by (x-factor)
 */
template<>
class ecl_geometry_PUBLIC Division<QuadraticPolynomial> {
public:
	Division() {};
	virtual ~Division() {};
	/**
	 * @brief Synthetic division by a factor.
	 * @param p : the polynomial
	 * @param factor : of the form (x-factor)
	 * @param remainder : remainder left after the division
	 * @return LinearFunction : result of the division (along with remainder)
	 */
	ECL_PUBLIC LinearFunction operator()(const QuadraticPolynomial &p, const double &factor, double &remainder);
};

/**
 * @brief Synthetic division between cubic and a factor.
 *
 * Does long division of the polynomial by (x-factor)
 */
template<>
class ecl_geometry_PUBLIC Division<CubicPolynomial> {
public:
	Division() {};
	virtual ~Division() {};
	/**
	 * @brief Synthetic division by a factor.
	 * @param p : the polynomial
	 * @param factor : of the form (x-factor)
	 * @param remainder : remainder left after the division
	 * @return QuadraticPolynomial : result of the division (along with remainder)
	 */
	ECL_PUBLIC QuadraticPolynomial operator()(const CubicPolynomial &p, const double &factor, double &remainder);
};

/*****************************************************************************
** Interface [Roots]
*****************************************************************************/
/**
 * @brief X axis intercepts for linear functions.
 *
 * @sa Roots, Linear Function, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Roots<LinearFunction> {
public:
	Roots() {}
	virtual ~Roots() {}
	/**
	 * @brief Returns the root of the specified polynomial.
	 *
	 * @param p : linear function.
	 * @return vector<double> : list of x axis intercepts.
	 **/
	ECL_PUBLIC Array<double> operator()(const LinearFunction& p);
};

/**
 * @brief X axis intercepts for quadratics.
 *
 * @sa Roots, QuadraticPolynomial, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Roots<QuadraticPolynomial> {
public:
	Roots() {}
	virtual ~Roots() {}
	/**
	 * @brief Returns the root of the specified polynomial.
	 *
	 * @param p : quadratic polyonmial.
	 * @return vector<double> : list of x axis intercepts.
	 **/
	ECL_PUBLIC Array<double> operator()(const QuadraticPolynomial& p);
};
/**
 * @brief X axis intercepts for cubic polynomials.
 *
 * @sa Roots, CubicPolynomial, @ref polynomialsGeometry "Math::Polynomials".
 */
template<>
class ecl_geometry_PUBLIC Roots<CubicPolynomial> {
public:
	Roots() {}
	virtual ~Roots() {}
	/**
	 * @brief Returns the root of the specified polynomial.
	 *
	 * @param p : cubic polynomial.
	 * @return vector<double> : list of x axis intercepts.
	 **/
	ECL_PUBLIC Array<double> operator()(const CubicPolynomial& p);
};

/*****************************************************************************
** Interfaces [FunctionMath]
*****************************************************************************/
/**
 * @brief Specialises the function math loader for linear functions.
 *
 * This lets you use the functions directly from within the class, e.g.
 *
 * <code>
 * LinearFunction f = LinearFunction::Interpolation(0.0,0.0,1.0,2.0);
 * LinearFunction g = LinearFunction::Interpolation(0.0,2.0,1.0,0.0);
 * CartsianPoint2d point = LinearFunction::Intersection(f,g);
 * </code>
 */
template <>
class ECL_PUBLIC FunctionMath<LinearFunction> {
public:
	FunctionMath() {}; /** @brief Default constructor. **/
	virtual ~FunctionMath() {};
	/**
	 * @brief Redirections intersection operator for linear functions.
	 *
	 * @sa Intersection<LinearFunction>
	 */
	static CartesianPoint2d Intersection(const LinearFunction &f, const LinearFunction &g) ecl_throw_decl(StandardException) {
		CartesianPoint2d point;
		ecl_try {
			ecl::Intersection< LinearFunction > intersection;
			point = intersection(f,g);
		} ecl_catch( const StandardException &e ) {
			ecl_throw(StandardException(LOC,e));
		}
		return point;
	}
	/**
	 * @brief Root of the linear function.
	 *
	 * @sa Roots<LinearFunction>
	 */
	static Array<double> Roots(const LinearFunction &function) {
		return ecl::Roots<LinearFunction>()(function);
	}
	/**
	 * @brief Redirections the minimum operator for linear functions.
	 *
	 * @sa Minimum<LinearFunction>
	 */
	static double Minimum(const double& x_begin, const double& x_end, const LinearFunction &function) {
		return ecl::Minimum<LinearFunction>()(x_begin, x_end, function);
	}
	/**
	 * @brief Redirections the maximum operator for linear functions.
	 *
	 * @sa Maximum<LinearFunction>
	 */
	static double Maximum(const double& x_begin, const double& x_end, const LinearFunction &function) {
		return ecl::Maximum<LinearFunction>()(x_begin, x_end, function);
	}
};
/**
 * @brief Specialises the function math loader for quadratics.
 *
 * This lets you use the functions directly from within the class.
 *
 * @sa FunctionMath<LinearFunction>
 */
template <>
class ECL_PUBLIC FunctionMath<QuadraticPolynomial> {
public:
	FunctionMath() {}; /** @brief Default constructor. **/
	virtual ~FunctionMath() {};
	/**
	 * @brief Real roots of the quadratic.
	 *
	 * @sa Roots<QuadraticPolynomial>
	 */
	static Array<double> Roots(const QuadraticPolynomial &p) {
		return ecl::Roots<QuadraticPolynomial>()(p);
	}
	/**
	 * @brief Division by a factor (synthetic division algorithm).
	 */
	static LinearFunction Division(const QuadraticPolynomial &p, const double &factor, double &remainder) {
		LinearFunction f(ecl::Division<QuadraticPolynomial>()(p,factor,remainder));
		return f;
	}
};

/**
 * @brief Specialises the function math loader for cubics.
 *
 * This lets you use the functions directly from within the class.
 *
 * @sa FunctionMath<LinearFunction>
 */
template <>
class ECL_PUBLIC FunctionMath<CubicPolynomial> {
public:
	FunctionMath() {}; /** @brief Default constructor. **/
	virtual ~FunctionMath() {};
	/**
	 * @brief Real roots of the cubic.
	 *
	 * @sa Roots<CubicPolynomial>
	 */
	static Array<double> Roots(const CubicPolynomial &p) {
		return ecl::Roots<CubicPolynomial>()(p);
	}
	/**
	 * @brief Division by a factor (synthetic division algorithm).
	 */
	static QuadraticPolynomial Division(const CubicPolynomial &p, const double &factor, double &remainder) {
		QuadraticPolynomial q(ecl::Division<CubicPolynomial>()(p,factor,remainder));
		return q;
	}
	/**
	 * @brief Redirections the minimum operator for cubics.
	 *
	 * @sa Minimum<CubicPolynomial>
	 */
	static double Minimum(const double& x_begin, const double& x_end, const CubicPolynomial &function) {
		return ecl::Minimum<CubicPolynomial>()(x_begin, x_end, function);
	}
	/**
	 * @brief Redirections the maximum operator for cubics.
	 *
	 * @sa Intersection<CubicPolynomial>
	 */
	static double Maximum(const double& x_begin, const double& x_end, const CubicPolynomial &function) {
		return ecl::Maximum<CubicPolynomial>()(x_begin, x_end, function);
	}
};

}; // namespace ecl

#endif /*ECL_GEOMETRY_POLYNOMIALS_HPP_*/
