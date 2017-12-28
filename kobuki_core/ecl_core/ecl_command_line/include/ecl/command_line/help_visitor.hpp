/**
 * @file /include/ecl/command_line/help_visitor.hpp
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

#ifndef TCLAP_HELP_VISITOR_H
#define TCLAP_HELP_VISITOR_H

#include "cmd_line_interface.hpp"
#include "cmd_line_output.hpp"
#include "visitor.hpp"

namespace ecl {

/**
 * @brief TClap class indirectly used to define the interface for visitors.
 *
 * A Visitor object that calls the usage method of the given CmdLineOutput
 * object for the specified CmdLine object.
 */
class HelpVisitor: public Visitor
{
	protected:

		/**
		 * The CmdLine the output will be generated for.
		 */
		CmdLineInterface* _cmd;

		/**
		 * The output object.
		 */
		CmdLineOutput** _out;

	public:

		/**
		 * Constructor.
		 * \param cmd - The CmdLine the output will be generated for.
		 * \param out - The type of output.
		 */
		HelpVisitor(CmdLineInterface* cmd, CmdLineOutput** out)
				: Visitor(), _cmd( cmd ), _out( out ) { }

		/**
		 * Calls the usage method of the CmdLineOutput for the
		 * specified CmdLine.
		 */
		void visit() { (*_out)->usage(*_cmd); exit(0); }

};

}; // namespace ecl


#endif
