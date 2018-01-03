/**
 * @file /include/ecl/concepts/blueprints.hpp
 *
 * @brief Lightweight objects used for custom class instantiation and assignment.
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONCEPTS_BLUEPRINTS_HPP_
#define ECL_CONCEPTS_BLUEPRINTS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Concept [BluePrint]
*****************************************************************************/

/**
 * @brief Defines validating functionality for the @ref blueprintsConcept "blueprint concept".
 **/
template <typename Implementation>
class BluePrintConcept {
public:
	/**
	 * @brief Implements a concept test for blueprints.
	 *
	 * There are three conditions required by blueprints:
	 * - base_type : a typedef for the target class for which they generate instances/configurations.
	 * - base_type instantiate() : they must be able to instantiate a base_type object.
	 * - void apply(base_type&) : they must be able to apply this configuration to an existing object.
	 */
	ecl_compile_time_concept_test(BluePrintConcept)
	{
		target = blue_print.instantiate();
		blue_print.apply(target);
	}

private:
	typename Implementation::base_type target;
	Implementation blue_print;
};

}; // namespace ecl

#endif /* ECL_CONCEPTS_BLUEPRINTS_HPP_ */
