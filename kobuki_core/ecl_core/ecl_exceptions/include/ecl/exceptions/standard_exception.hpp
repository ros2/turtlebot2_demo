/**
 * @file /include/ecl/exceptions/standard_exception.hpp
 *
 * @brief Standard exception type, provides code location and error string.
 *
 * Standard exception type for try-catch handling in the ecl. It provides just
 * the code location and an error message.
 *
 * @date April 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_STANDARD_EXCEPTION_HPP_
#define ECL_EXCEPTIONS_STANDARD_EXCEPTION_HPP_

/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include "exception.hpp"
#include <ecl/errors/handlers.hpp>
#include <ecl/errors/macros.hpp>
#include <cstring>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [StandardException]
*****************************************************************************/
/**
 @brief Standard exception type, provides code location and error string.

 This exception class utilises a default set of error flags and messages
 defined by the ecl. They can be coupled with the ecl exception macros
 for throwing to also enable debug-only throws, assure-check throws as
 well as rethrowing.

 @sa Exception, DataException, PosixErrorHandler, @ref errorsExceptions "Exceptions Guide".
 **/
class ecl_exceptions_PUBLIC StandardException : public Exception
{
    public:
        /**
         * @brief Default constructor for standard exceptions.
         * @param loc : use with the LOC macro, identifies the line and file of the code.
         * @param error : enumerated exception error type.
         **/
        StandardException(const char* loc, ErrorFlag error );
        /**
         * Constructor for standard exceptions with a custom message.
         * @param loc : use with the LOC macro, identifies the line and file of the code.
         * @param error : enumerated exception error type.
         * @param msg : extra detail message.
         **/
        StandardException(const char* loc, ErrorFlag error, const std::string &msg );
        /**
         * Constructor for standard exceptions that enables rethrowing of an existing exception up
         * the hierarchy with a new code location stamp.
         * @param loc : use with the LOC macro, identifies the line and file of the code.
         * @param e : a caught standard exception that is to be rethrown.
         **/
        StandardException(const char* loc, const StandardException &e );

        virtual ~StandardException() throw() {}

        /**
         * Default exception handling output function.
         *
         * @return char const* : the output message.
         */
        const char* what() const throw();

        /**
         * Detailed error message associated with the exception.
         *
         * @return std::string : the detailed message
         */
        const std::string& message() const { return detailed_message; };

        const ErrorFlag& flag() const { return error_flag; }  /**< @brief Flag enumerating the type of exception thrown. **/

    private:
        const ErrorFlag error_flag;
        std::string detailed_message;
};

}; // namespace ecl

#endif /* ECL_DISABLE_EXCEPTIONS */
#endif /*ECL_EXCEPTIONS_STANDARD_EXCEPTION_HPP_*/
