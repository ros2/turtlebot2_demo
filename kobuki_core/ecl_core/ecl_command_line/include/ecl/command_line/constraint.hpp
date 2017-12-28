/**
 * @file /include/ecl/command_line/constraint.hpp
 *
 * @brief TCLAP command line argument parser classes.
 *
 * TCLAP command line argument parser classes.
 *
 * @author Michael E. Smoot, Daniel J. Stonier
 * @date May 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TCLAP_CONSTRAINT_H
#define TCLAP_CONSTRAINT_H

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace ecl {


/**
 * @brief Defines the interaction between an argument and a constraint.
 *
 * The interface that defines the interaction between the Arg and Constraint.
 */
template<class T>
class Constraint
{

	public:
		/**
		 * Returns a description of the Constraint.
		 */
		virtual std::string description() const =0;

		/**
		 * Returns the short ID for the Constraint.
		 */
		virtual std::string shortID() const =0;

		/**
		 * The method used to verify that the value parsed from the command
		 * line meets the constraint.
		 * \param value - The value that will be checked.
		 */
		virtual bool check(const T& value) const =0;

		/**
		 * Destructor.
		 * Silences warnings about Constraint being a base class with virtual
		 * functions but without a virtual destructor.
		 */
		virtual ~Constraint() { ; }
};

}; // namespace ecl

#endif
