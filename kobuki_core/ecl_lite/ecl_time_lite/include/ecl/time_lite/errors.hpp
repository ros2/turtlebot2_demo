/**
 * @file /ecl_time_lite/include/ecl/time_lite/errors.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Feb 16, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TIME_LITE_ERRORS_HPP_
#define ECL_TIME_LITE_ERRORS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/errors/handlers.hpp>
#include <ecl/config/macros.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Extends the generic ecl error handler with some time specific error strings.
 */
class ecl_time_lite_PUBLIC  TimeError : public Error {
public:
	/**
	 * @brief Configures the error class with the specified error flag.
	 *
	 * @param flag : the error type.
	 */
	TimeError(const ErrorFlag& flag = UnknownError) : Error(flag) {}
protected:
	virtual const char* outOfRangeErrorString() { return "The input time sec/nsec pair was outside of the permitted range."; }
	virtual const char* argNotSupportedErrorString() { return "The clock specified is not supported on this system."; }
	virtual const char* permissionsErrorString() { return "You do not have permission to set the specified clock."; }
	virtual const char* memoryErrorString() { return "One of the input arguments is not in memory addressable space."; }
	virtual const char* interruptedErrorString() { return "This operation was interrupted by a signal."; }
};

} // namespace ecl

#endif /* ECL_TIME_LITE_ERRORS_HPP_ */
