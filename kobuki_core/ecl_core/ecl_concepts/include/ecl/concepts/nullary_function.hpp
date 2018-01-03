/**
 * @file /include/ecl/concepts/nullary_function.hpp
 *
 * @brief Defines validating the functionality for the <i>nullary function</i> concept.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_NULLARY_FUNCTION_HPP_
#define ECL_CONCEPTS_NULLARY_FUNCTION_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Concept [Container]
*****************************************************************************/

/**
 * @brief Defines validating functionality for the @ref nullaryFunctionConcept "nullary function concept".
 **/
template <typename Implementation>
class NullaryFunctionConcept {
    public:

        /**
         * @brief Implements a concept test for nullary functions.
         *
         * The following conditions are required by nullary functions:
         *
         * - operator()() : a method which calls the nullary function.
         * - result_type : a typedef for the return type of the nullary function.
         */
		ecl_compile_time_concept_test(NullaryFunctionConcept)
        {
        	typedef typename Implementation::result_type return_type;
        	function();
        	// Unfortunately we can't test for the result type to match a certain value here
        	// as this would then have 2 template arguments with a separating comma. That throws
        	// the compile_time_concept_check macro into thinking it has two macro arguments.
        }

    private:
        // Putting instantiations here actually saves instantiation (which can cause a
        // problem if there is no default constructor).
        Implementation function;
};

}; // namespace ecl

#endif /* ECL_CONCEPTS_NULLARY_FUNCTION_HPP_ */
