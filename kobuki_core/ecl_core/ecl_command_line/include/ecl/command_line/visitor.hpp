/**
 * @file /include/ecl/command_line/visitor.hpp
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

#ifndef TCLAP_VISITOR_H
#define TCLAP_VISITOR_H

namespace ecl {

/**
 * @brief TClap class indirectly used to define the interface for visitors.
 *
 * A base class that defines the interface for visitors.
 */
class Visitor
{
	public:

		/**
		 * Constructor. Does nothing.
		 */
		Visitor() { }

		/**
		 * Destructor. Does nothing.
		 */
		virtual ~Visitor() { }

		/**
		 * Does nothing. Should be overridden by child.
		 */
		virtual void visit() { }
};
}; // namespace ecl


#endif
