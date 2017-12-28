/**
 * @file /include/ecl/command_line/ignore_rest_visitor.hpp
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

#ifndef TCLAP_IGNORE_REST_VISITOR_H
#define TCLAP_IGNORE_REST_VISITOR_H

#include "visitor.hpp"
#include "arg.hpp"

namespace ecl {


/**
 * @brief TClap class indirectly used to define the interface for visitors.
 *
 * A Vistor that tells the CmdLine to begin ignoring arguments after
 * this one is parsed.
 */
class IgnoreRestVisitor: public Visitor
{
	public:

		/**
		 * Constructor.
		 */
		IgnoreRestVisitor() : Visitor() {}

		/**
		 * Sets Arg::_ignoreRest.
		 */
		void visit() { Arg::beginIgnoring();  }
};

}; // namespace ecl


#endif
