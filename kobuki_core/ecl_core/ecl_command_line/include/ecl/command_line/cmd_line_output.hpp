/**
 * @file /include/ecl/command_line/cmd_line_output.hpp
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

#ifndef TCLAP_CMDLINEOUTPUT_H
#define TCLAP_CMDLINEOUTPUT_H

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace ecl {

class CmdLineInterface;
class ArgException;

/**
 * @brief Tclap class indirectly used by children for standardising outputs.
 *
 * The interface that any output object must implement.
 */
class CmdLineOutput
{

	public:

		/**
		 * Virtual destructor.
		 */
		virtual ~CmdLineOutput() {}

		/**
		 * Generates some sort of output for the USAGE.
		 * \param c - The CmdLine object the output is generated for.
		 */
		virtual void usage(CmdLineInterface& c)=0;

		/**
		 * Generates some sort of output for the version.
		 * \param c - The CmdLine object the output is generated for.
		 */
		virtual void version(CmdLineInterface& c)=0;

		/**
		 * Generates some sort of output for a failure.
		 * \param c - The CmdLine object the output is generated for.
		 * \param e - The ArgException that caused the failure.
		 */
		virtual void failure( CmdLineInterface& c,
						      ArgException& e )=0;

};

}; // namespace ecl

#endif
