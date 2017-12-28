/**
 * @file /include/ecl/geometry/function_math.hpp
 *
 * @brief Functors for evaluating various mathematical properties of functions.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_GEOMETRY_FUNCTION_MATH_HPP_
#define ECL_GEOMETRY_FUNCTION_MATH_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Factory
*****************************************************************************/
/**
 * @brief Used as a parent to load function math into function classes.
 *
 * Used a bit like the blueprint factory for polynomials. Implementation is in
 * the specialisations (if it has any).
 */
template <typename Function>
class ECL_PUBLIC FunctionMath {
public:
	/**
	 * @brief Default empty constructor (loads no math).
	 */
	FunctionMath() {};
	virtual ~FunctionMath() {};
};

/*****************************************************************************
** Maximum/Minimum
*****************************************************************************/

/**
 * @brief Primary template functor for the maximum of a continuous function.
 *
 * Maximum of a continuous function on a bounded interval. This functor
 * is only a primary template, the specialisations do the actual work. Consequently
 * this class cannot be instantiated directly.
 *
 * @sa Maximum<LinearFunction>, Maximum<CubicPolynomial>.
 **/
template <typename Function>
class ECL_LOCAL Maximum {
private:
	/**
	 * @brief Private constructor - prevents instantiation of this primary template.
	 */
	Maximum() {};
	virtual ~Maximum() {};
};


/**
 * @brief Primary template functor for the minimum of a continuous function.
 *
 * Minimum of a continuous function on a bounded interval. This functor
 * is only a primary template, the specialisations do the actual work. Consequently
 * this class cannot be instantiated directly.
 *
 * @sa Minimum<LinearFunction>, Minimum<CubicPolynomial>.
 **/
template <typename Function>
class ECL_LOCAL Minimum {
private:
	/**
	 * @brief Private constructor - prevents instantiation of this primary template.
	 */
	Minimum() {};
	virtual ~Minimum() {};
};


/**
 * @brief Primary template functor for the intersection of like functions.
 **/
template <typename Function>
class ECL_LOCAL Intersection {
private:
	/**
	 * @brief Private constructor - prevents instantiation of this primary template.
	 */
	Intersection() {};
	virtual ~Intersection() {};
};

/**
 * @brief Primary template functor for polynomial division.
 **/
template <typename Function>
class ECL_LOCAL Division {
private:
	/**
	 * @brief Private constructor - prevents instantiation of this primary template.
	 */
	Division() {};
	virtual ~Division() {};
};

/**
 * @brief Primary template functor for the roots of a function (x-axis intercepts).
 **/
template <typename Function>
class ECL_LOCAL Roots {
private:
	/**
	 * @brief Private constructor - prevents instantiation of this primary template.
	 */
	Roots() {};
	virtual ~Roots() {};
};

}; // namespace ecl


#endif /* ECL_GEOMETRY_FUNCTION_MATH_HPP_ */
