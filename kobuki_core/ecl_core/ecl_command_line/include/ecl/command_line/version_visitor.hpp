/**
 * @file /include/ecl/command_line/version_visitor.hpp
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

#ifndef TCLAP_VERSION_VISITOR_H
#define TCLAP_VERSION_VISITOR_H

#include "cmd_line_interface.hpp"
#include "cmd_line_output.hpp"
#include "visitor.hpp"

namespace ecl {

/**
 * @brief TClap class indirectly used to define the interface for visitors.
 *
 * A Vistor that will call the version method of the given CmdLineOutput
 * for the specified CmdLine object and then exit.
 */
class VersionVisitor: public Visitor
{
	protected:

		/**
		 * The CmdLine of interest.
		 */
		CmdLineInterface* _cmd;

		/**
		 * The output object.
		 */
		CmdLineOutput** _out;

	public:

		/**
		 * Constructor.
		 * \param cmd - The CmdLine the output is generated for.
		 * \param out - The type of output.
		 */
		VersionVisitor( CmdLineInterface* cmd, CmdLineOutput** out )
				: Visitor(), _cmd( cmd ), _out( out ) { }

		/**
		 * Calls the version method of the output object using the
		 * specified CmdLine.
		 */
		void visit() { (*_out)->version(*_cmd); exit(0); }

};

}; // namespace ecl


#endif
