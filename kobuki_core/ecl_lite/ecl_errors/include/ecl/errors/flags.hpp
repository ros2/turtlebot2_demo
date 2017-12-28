/**
 * @file /ecl_errors/include/ecl/errors/flags.hpp
 *
 * @brief Flags for error types.
 *
 * @date April, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_ERRORS_FLAGS_HPP_
#define ECL_ERRORS_FLAGS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {


/*****************************************************************************
** Error Flags
*****************************************************************************/
/**
 * @brief Enumerated flags for error message handling.
 *
 * These are simple flags to identify the type of error when it occurs. Use
 * with the Error class (and its child classes, e.g. TimeError) to identify
 * errors and get verbose string output when debugging.
 */
enum ErrorFlag {
    NoError = 0,		/**< No error (often a meaningful error state in itself). **/
    UnknownError = -1,        	/**< Unknown error type. **/
    OutOfRangeError = -2, 	/**< Tried to access beyond the range of the object (usually container). **/
    ConstructorError = -3,	/**< An error occurred somewhere inside a class constructor. **/
    DestructorError = -4,     	/**< The destructor failed to self-destruct. **/
    ConversionError = -5,     	/**< A conversion (usually between types) failed. **/
    OpenError = -6,           	/**< Failed to open an input/output device. **/
    CloseError = -7,		/**< Failed to close an input/output device. **/
    InvalidArgError = -8,   	/**< The user entered an invalid argument to this method, c.f. InvalidObjectError. **/
    ConfigurationError = -9,  	/**< There was a configuration error (usually in a configure() or initialise() method). **/
    ConnectionError = -10,     	/**< There was a connection error, usually with input-output devices. **/
    ReadError = -11,           	/**< There was a read error, usually with input-output devices. **/
    WriteError = -12,          	/**< There was a write error, usually with input-output devices. **/
    NotInitialisedError = -13, 	/**< The target object has not yet been initialised, accessing before doing so is not permitted. **/
    PermissionsError = -14,    	/**< The caller does not have the required permissions. **/
    MemoryError = -15,     	/**< There was a problem allocating the requested memory. **/
    UsageError = -16,       	/**< The object was used incorrectly and caused an error. **/
    RaiiError = -17,       	/**< The object is pure RAII style and must be initialised properly. **/
    ArgNotSupportedError = -18, /**< The combination of input arguments is not supported on this platform.**/
    NotSupportedError = -19, 	/**< This operation is not supported on this platform. **/
    BusyError = -20,         	/**< Resources are busy, operation is not permitted. **/
    OutOfResourcesError = -21, 	/**< Out of resources, cannot proceed. **/
    InterruptedError = -22, 	/**< This operation was interrupted. **/
    BlockingError = -23,	/**< A device marked as blocking, but used as non-blocking, or vice versa. **/
    SystemFailureError = -24,	/**< A subcomponent of the system platform has failed (e.g. io subsystem). **/
    InvalidObjectError = -25,   /**< Attempted to work on an invalid object (e.g. dir instead of a file), c.f. InvalidArgError **/
    IsLockedError = -26,        /**< Invalidates attempts to work further because an object is locked. **/
    TimeOutError = -27,         /**< A timeout occured. **/
    NotFoundError = -28,	/**< An object was not found. **/
    ConnectionRefusedError = -29, /**< A connection was refused by the other end (e.g. socket client server connections). **/
    PosixError = -101, 		/**< Deprecating. **/
    InvalidInputError = -103    /**< Deprecated. **/
};


} // namespace ecl

#endif /* ECL_ERRORS_FLAGS_HPP_ */
