/**
 * @file /include/ecl/geometry/spline_function.hpp
 *
 * @brief Wrapper for a generic spline function.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_SPLINE_FUNCTION_HPP_
#define ECL_GEOMETRY_SPLINE_FUNCTION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/containers/array.hpp>
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [GenericSplineFunction]
*****************************************************************************/
/**
 * @brief This is a parent class for generic spline functions.
 *
 * This is the parent interface for spline functions.
 *
 * @sa SplineFunction
 **/
class ECL_PUBLIC GenericSplineFunction {
    public:
		virtual ~GenericSplineFunction() {};
        /**
         * @brief Virtual access function for a spline function.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the function at x.
         */
        virtual double operator()(const double &x) const = 0;
        /**
         * @brief Virtual access function for a spline function derivative.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the derivative at x.
         */
        virtual double derivative(const double &x) const = 0;
        /**
         * @brief Virtual access function for a spline function's 2nd derivative.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the 2nd derivative at x.
         */
        virtual double dderivative(const double &x) const = 0;

        /**
         * @brief Get access to the domain for this spline function.
         *
         * Get access to the domain for this spline function.
         *
         * @return const Array<double,2>& : stores domain_begin and domain_end.
         */
        const Array<double,2>& domain() { return time_domain; }

    protected:
        Array<double,2> time_domain;
};
/*****************************************************************************
** Interface [SplineFunction]
*****************************************************************************/
/**
 * @brief  Template wrapper for a generic spline function.
 *
 * Template wrapper for a generic spline function. Didn't want
 * to inherit an interface for all the underlying functions (polynomials,
 * tension functions, splines etc) so this is an indirect way of
 * getting the polymorphism to work. It is not usually used directly, but
 * rather internally in the trajectory classes.
 *
 * @tparam Function : the internally wrapped function.
 **/
template <typename Function>
class ECL_PUBLIC SplineFunction : public GenericSplineFunction {
    public:
        /**
         * @brief Constructor, copies across an already existing spline function type and wraps it.
         *
         * Copies an existing spline function into this wrapper along with domain boundaries.
         *
         * @param time_begin : beginning of the domain the spline function is valid on.
         * @param time_end : end of the domain the spline function is valid on.
         * @param f : the spline function to wrap.
         */
        SplineFunction (const double& time_begin, const double& time_end, const Function& f) : function(f) {
            time_domain << time_begin, time_end;
        };
        virtual ~SplineFunction() {};

        /**
         * @brief Access the value of the spline function at the specified value.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the function at x.
         * @exception : StandardException : throws if input x value is outside the domain [debug mode only].
         */
        double operator()(const double &x) const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( ( x >= time_domain[0] ) && x <= time_domain[1], StandardException(LOC,OutOfRangeError ) );
            return function(x);
        }

        /**
         * @brief Access the value of the spline function's derivative at the specified value.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the derivative at x.
         * @exception : StandardException : throws if input x value is outside the domain [debug mode only].
         */
        double derivative(const double &x) const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( ( x >= time_domain[0] ) && x <= time_domain[1], StandardException(LOC,OutOfRangeError ) );
            return function.derivative(x);
        }

        /**
         * @brief Access the value of the spline function's 2nd derivative at the specified value.
         * @param x : the point at which you wish to calculate the value for.
         * @return double : the value of the 2nd derivative at x.
         * @exception : StandardException : throws if input x value is outside the domain [debug mode only].
         */
        double dderivative(const double &x) const ecl_assert_throw_decl(StandardException) {
            ecl_assert_throw( ( x >= time_domain[0] ) && x <= time_domain[1], StandardException(LOC,OutOfRangeError ) );
            return function.dderivative(x);
        }

    private:
        Function function;
};

}; // namespace ecl


#endif /* SPLINE_FUNCTION_HPP_ */
