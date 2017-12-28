/**
 * @file /include/ecl/command_line/arg_exception.hpp
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

#ifndef TCLAP_ARG_EXCEPTION_H
#define TCLAP_ARG_EXCEPTION_H

#include <string>
#include <exception>

namespace ecl {


/**
 * @brief Defines the exception that is thrown whenever a command line is created and parsed.
 *
 * A simple class that defines an argument exception.  Should be caught
 * whenever a CmdLine is created and parsed.
 */
class ArgException : public std::exception
{
	public:

		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source.
		 * \param td - Text describing the type of ArgException it is.
		 * of the exception.
		 */
		ArgException( const std::string& text = "undefined exception",
					  const std::string& id = "undefined",
					  const std::string& td = "Generic ArgException")
			: std::exception(),
			  _errorText(text),
			  _argId( id ),
			  _typeDescription(td)
		{ }

		/**
		 * Destructor.
		 */
		virtual ~ArgException() throw() { }

		/**
		 * Returns the error text.
		 */
		std::string error() const { return ( _errorText ); }

		/**
		 * Returns the argument id.
		 */
		std::string argId() const
		{
			if ( _argId == "undefined" )
				return " ";
			else
				return ( "Argument: " + _argId );
		}

		/**
		 * Returns the arg id and error text.
		 */
		const char* what() const throw()
		{
			static std::string ex;
			ex = _argId + " -- " + _errorText;
			return ex.c_str();
		}

		/**
		 * Returns the type of the exception.  Used to explain and distinguish
		 * between different child exceptions.
		 */
		std::string typeDescription() const
		{
			return _typeDescription;
		}


	private:

		/**
		 * The text of the exception message.
		 */
		std::string _errorText;

		/**
		 * The argument related to this exception.
		 */
		std::string _argId;

		/**
		 * Describes the type of the exception.  Used to distinguish
		 * between different child exceptions.
		 */
		std::string _typeDescription;

};

/**
 * @brief Defines the exception that is thrown when an argument is improperly parsed.
 *
 * Thrown from within the child Arg classes when it fails to properly
 * parse the argument it has been passed.
 */
class ArgParseException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		ArgParseException( const std::string& text = "undefined exception",
					       const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string( "Exception found while parsing " ) +
							std::string( "the value the Arg has been passed." ))
			{ }
};

/**
 * @brief Defines the exception when an argument is improperly specified.
 *
 * Thrown from CmdLine when the arguments on the command line are not
 * properly specified, e.g. too many arguments, required argument missing, etc.
 */
class CmdLineParseException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		CmdLineParseException( const std::string& text = "undefined exception",
					           const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string( "Exception found when the values ") +
							std::string( "on the command line do not meet ") +
							std::string( "the requirements of the defined ") +
							std::string( "Args." ))
		{ }
};

/**
 * @brief Defines the exception that is thrown whenever a conflict in arguments occurs.
 *
 * Thrown from Arg and CmdLine when an Arg is improperly specified, e.g.
 * same flag as another Arg, same name, etc.
 */
class SpecificationException : public ArgException
{
	public:
		/**
		 * Constructor.
		 * \param text - The text of the exception.
		 * \param id - The text identifying the argument source
		 * of the exception.
		 */
		SpecificationException( const std::string& text = "undefined exception",
					            const std::string& id = "undefined" )
			: ArgException( text,
			                id,
							std::string("Exception found when an Arg object ")+
							std::string("is improperly defined by the ") +
							std::string("developer." ))
		{ }

};

}; // namespace ecl


#endif

