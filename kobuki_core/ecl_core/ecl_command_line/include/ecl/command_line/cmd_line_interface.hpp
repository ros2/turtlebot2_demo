/**
 * @file /include/ecl/command_line/cmd_line_interface.hpp
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

#ifndef TCLAP_COMMANDLINE_INTERFACE_H
#define TCLAP_COMMANDLINE_INTERFACE_H

#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <algorithm>


namespace ecl {

class Arg;
class CmdLineOutput;
class XorHandler;

/**
 * @brief Managing interface for
 * The base class that manages the command line definition and passes
 * along the parsing to the appropriate Arg classes.
 */
class CmdLineInterface
{
	public:

		/**
		 * Destructor
		 */
		virtual ~CmdLineInterface() {}

		/**
		 * Adds an argument to the list of arguments to be parsed.
		 * \param a - Argument to be added.
		 */
		virtual void add( Arg& a )=0;

		/**
		 * An alternative add.  Functionally identical.
		 * \param a - Argument to be added.
		 */
		virtual void add( Arg* a )=0;

		/**
		 * Add two Args that will be xor'd.
		 * If this method is used, add does
		 * not need to be called.
		 * \param a - Argument to be added and xor'd.
		 * \param b - Argument to be added and xor'd.
		 */
		virtual void xorAdd( Arg& a, Arg& b )=0;

		/**
		 * Add a list of Args that will be xor'd.  If this method is used,
		 * add does not need to be called.
		 * \param xors - List of Args to be added and xor'd.
		 */
		virtual void xorAdd( std::vector<Arg*>& xors )=0;

		/**
		 * Parses the command line.
		 * \param argc - Number of arguments.
		 * \param argv - Array of arguments.
		 */
		virtual void parse(int argc, char** argv)=0;

		/**
		 * Returns the CmdLineOutput object.
		 */
		virtual CmdLineOutput* getOutput()=0;

		/**
		 * \param co - CmdLineOutput object that we want to use instead.
		 */
		virtual void setOutput(CmdLineOutput* co)=0;

		/**
		 * Returns the version string.
		 */
		virtual std::string& getVersion()=0;

		/**
		 * Returns the program name string.
		 */
		virtual std::string& getProgramName()=0;

		/**
		 * Returns the argList.
		 */
		virtual std::list<Arg*>& getArgList()=0;

		/**
		 * Returns the XorHandler.
		 */
		virtual XorHandler& getXorHandler()=0;

		/**
		 * Returns the delimiter string.
		 */
		virtual char getDelimiter()=0;

		/**
		 * Returns the message string.
		 */
		virtual std::string& getMessage()=0;

		/**
		 * Indicates whether or not the help and version switches were created
		 * automatically.
		 */
		virtual bool hasHelpAndVersion()=0;
};

}; // namespace ecl


#endif
