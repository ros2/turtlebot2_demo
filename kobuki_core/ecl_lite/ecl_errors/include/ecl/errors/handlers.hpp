/**
 * @file /ecl_errors/include/ecl/errors/handlers.hpp
 *
 * @brief Error handling without exceptions.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ERRORS_HANDLERS_HPP_
#define ECL_ERRORS_HANDLERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <cstring>
#include <cstdio>
#include "macros.hpp"
#include "flags.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief The primary error handler for ecl libraries.
 *
 * This class is mostly used as the return type for ecl functions
 * to indicate errors.
 *
 * @code
 * Error f(const int &i) {
 *    if ( i == 3 ) {
 *        return Error(NoError);
 *    else {
 *        return Error(OutOfRangeError);
 *    }
 * }
 *
 * // code
 *
 * Error error = f(5);
 * if ( error.flag() != NoError ) {
 *     std::cout << error.what() << std::endl; // OR
 *     error.print(LOC);   // for a verbose message with code location added.
 * }
 * @endcode
 *
 * Alternatively the class can be inherited and the verbose messages overloaded.
 * Refer to the TimeError class in ecl_time_lite for an example.
 */
class ecl_errors_PUBLIC Error {
public:
	/**
	 * @brief Configure the return type with an error flag.
	 *
	 * @param flag : the type of error that is to be returned.
	 */
	Error(const ErrorFlag& flag = UnknownError) : error_flag(flag) {}
	virtual ~Error() {}

	/**
	 * @brief The flag identifying the error identified with this error handler.
	 * @return ErrorFlag : the error flag.
	 */
	virtual ErrorFlag flag() const { return error_flag; }
	virtual void operator=(const ErrorFlag &error) { error_flag = error; }

//	virtual void debug(const char* loc) {
//		#if !defined(NDEBUG) && !defined(ECL_NDEBUG)
//			printf("%s - %s\n",loc,what());
//		#endif
//	}

	/**
	 * @brief Print a location dependent message to standard out.
	 *
	 * Usage:
	 *
	 * @code
	 * Error error = f(5);
	 * if ( error.flag() != NoError ) {
	 *     error.print(LOC);
	 * }
	 * @endcode
	 * @param loc : use with the LOC macro.
	 */
	virtual void print(const char* loc) {
		printf("%s - %s\n",loc,what());
	}

	/**
	 * @brief A simple string verbosely representing the error that is handled.
	 *
	 * @return const char* : verbose representation of the error.
	 */
	virtual const char* what() const {
		switch (error_flag) {
			case (NoError) : { return noErrorString(); }
			case (OutOfRangeError) : { return outOfRangeErrorString(); }
			case (ConstructorError) : { return constructorErrorString(); }
			case (DestructorError) : { return destructorErrorString(); }
			case (ConversionError) : { return conversionErrorString(); }
			case (OpenError) : { return openErrorString(); }
			case (CloseError) : { return closeErrorString(); }
			case (InvalidArgError) : { return invalidArgErrorString(); }
			case (ConfigurationError) : { return configurationErrorString(); }
			case (ConnectionError) : { return connectErrorString(); }
			case (ReadError) : { return readErrorString(); }
			case (WriteError) : { return writeErrorString(); }
			case (NotInitialisedError) : { return notInitialisedErrorString(); }
			case (PermissionsError) : { return permissionsErrorString(); }
			case (MemoryError) : { return memoryErrorString(); }
			case (UsageError) : { return usageErrorString(); }
			case (RaiiError) : { return raiiErrorString(); }
			case (ArgNotSupportedError) : { return argNotSupportedErrorString(); }
			case (NotSupportedError) : { return notSupportedErrorString(); }
			case (BusyError) : { return busyErrorString(); }
			case (OutOfResourcesError) : { return outOfResourcesErrorString(); }
			case (InterruptedError) : { return interruptedErrorString(); }
			case (BlockingError) : { return blockingErrorString(); }
			case (SystemFailureError) : { return systemFailureErrorString(); }
			case (InvalidObjectError) : { return invalidObjectErrorString(); }
			case (IsLockedError) : { return isLockedErrorString(); }
			case (TimeOutError) : { return isTimeOutErrorString(); }
			case (NotFoundError) : { return notFoundErrorString(); }
			case (ConnectionRefusedError) : { return connectionRefusedString(); }

			default : { return unknownErrorString(); } // also covers the 'UnknownError flag.
		}
	}
protected:
	virtual const char* noErrorString() const { return "No error."; }
	virtual const char* outOfRangeErrorString() const { return "Out of range error - tried to access beyond the range of the object (usually container)."; }
	virtual const char* constructorErrorString() const { return "The constructor did not produce a valid object."; }
	virtual const char* destructorErrorString() const { return "The destructor failed to self destruct."; }
	virtual const char* conversionErrorString() const { return "Conversion from one type to another failed."; }
	virtual const char* openErrorString() const { return "Could not open the requested object."; }
	virtual const char* closeErrorString() const { return "Could not close the requested object."; }
	virtual const char* invalidArgErrorString() const { return "One of the supplied input arguments was invalid."; }
	virtual const char* configurationErrorString() const { return "There was a configuration error."; }
	virtual const char* connectErrorString() const { return "Could not connect."; }
	virtual const char* readErrorString() const { return "Could not read from the object."; }
	virtual const char* writeErrorString() const { return "Could not write to the object."; }
	virtual const char* notInitialisedErrorString() const { return "The object has not been properly initialised yet."; }
	virtual const char* permissionsErrorString() const { return "The caller does not have the required permissions."; }
	virtual const char* memoryErrorString() const { return "There was a problem allocating the requested memory."; }
	virtual const char* usageErrorString() const { return "The object was used incorrectly."; }
	virtual const char* raiiErrorString() const { return "The object is pure RAII style and must be initialised correctly, you may not use the default constructor."; }
	virtual const char* argNotSupportedErrorString() const { return "The combination of input arguments is not supported on this platform."; }
	virtual const char* notSupportedErrorString() const { return "This operation is not supported on this platform."; }
	virtual const char* busyErrorString() const { return "Resources are busy, operation is not permitted"; }
	virtual const char* outOfResourcesErrorString() const { return "Out of resources, cannot proceed."; }
	virtual const char* interruptedErrorString() const { return "This operation was interrupted."; }
	virtual const char* blockingErrorString() const { return "A device marked as blocking, but used as non-blocking, or vice versa."; }
	virtual const char* systemFailureErrorString() const { return "A subsystem has failed mid operation."; }
	virtual const char* invalidObjectErrorString() const { return "Attempted to work on an invalid object."; }
	virtual const char* isLockedErrorString() const { return "Invalidates attempts to work further because an object is locked."; }
	virtual const char* isTimeOutErrorString() const { return "A timeout occured."; }
	virtual const char* notFoundErrorString() const { return "The resource could not be found."; }
	virtual const char* connectionRefusedString() const { return "The connection was refused by the listener at the other end of the connection."; }
	virtual const char* unknownErrorString() const { return "Unknown error."; }

	ErrorFlag error_flag;
};


} // namespace ecl

#endif /* ECL_ERRORS_HANDLERS_HPP_ */
