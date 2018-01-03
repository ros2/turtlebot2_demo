/**
 * @file /include/ecl/exceptions/posix_error_handler.hpp
 *
 * @brief Exception type that handles posix errno values.
 *
 * Exception type that handles posix errno values and stringifies the error
 * number into a human readable message.
 *
 * @sa @ref errorsExceptions "Exceptions Guide".
 *
 * @date May, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_POSIX_EXCEPTION_HPP_
#define ECL_EXCEPTIONS_POSIX_EXCEPTION_HPP_

/*****************************************************************************
** Disable check
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#ifndef ECL_DISABLE_EXCEPTIONS

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstring> // strerror function
#include <string>
#include <errno.h>
#include "standard_exception.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [PosixErrorHandler]
*****************************************************************************/
/**
 * @brief Provides c++ mechanisms for handling posix errors.
 *
 * This class is a parent template for handling posix errors thrown by
 * a generic base class. Currently it only serves one purpose, that is to
 * redirect posix errors generated via the errno mechanism to StandardException objects.
 * If you wish to customise the output of posix errors for different objects
 * then simply specialise this class for the given object. See
 * PosixErrorHandler<ecl::time::TimeStamp> for an example specialisation.
 *
 * @sa StandardException, @ref errorsExceptions "Exceptions Guide".
 */
template <typename ThrowingClass>
class ecl_exceptions_PUBLIC PosixErrorHandler {
public:
	/**
	 * This static function is the general case response for generating
	 * StandardException objects from a posix errno value using the c
	 * strerror() function. To customise the response, specialise this
	 * class for the required object. See
	 * PosixErrorHandler<ecl::time::TimeStamp> for an example specialisation.
	 *
	 * @param loc : use with the LOC macro, identifies the line and file of the code.
	 */
	static StandardException GenerateStandardException(const char* loc) {
		return StandardException(loc, PosixError, std::string(strerror(errno))+".");
	}
	virtual ~PosixErrorHandler() {}
};

}; // namespace ecl

#endif /* ECL_DISABLE_EXCEPTIONS */
#endif /* ECL_EXCEPTIONS_POSIX_EXCEPTION_HPP_ */
