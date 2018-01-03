/**
 * @file /include/ecl/exceptions/exception.hpp
 *
 * @brief Macros and exceptions for try-catch handling within the ecl.
 *
 * Macros and exceptions for try-catch handling within the ecl.
 *
 * @date April 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_EXCEPTION_HPP_
#define ECL_EXCEPTIONS_EXCEPTION_HPP_

/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Includes
*****************************************************************************/

#include <exception>
#include <string>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** General exception
*****************************************************************************/
/**
 * @brief Virtual parent class for the ecl exceptions.
 *
 * This is the parent class for all ecl exceptions. It cannot be instantiated
 * directly - it can only be sub-classed.
 *
 * @sa StandardException, DataException, @ref errorsExceptions "Exceptions Guide".
 **/
class ecl_exceptions_PUBLIC Exception : public std::exception
{
    public:
        /**
         * Virtual method for the error message output.
         */
        virtual const char * what() const throw() = 0;
        virtual ~Exception() throw() {} /**< Default destructor. **/

    protected:
        /**
         * Default constructor that does not configure the code location (used internally by
         * the ecl when rethrowing).
         */
        Exception() {}
        /**
         * Configures the code location - make sure you initialise this with the LOC macro.
         * @param loc : use with the LOC macro, identifies the line and file of the code.
         **/
        Exception(const char* loc ) : location(loc) {};

        std::string location;
};

}; // namespace ecl

#endif /* ECL_DISABLE_EXCEPTIONS */
#endif /*ECL_EXCEPTIONS_EXCEPTION_HPP_*/
