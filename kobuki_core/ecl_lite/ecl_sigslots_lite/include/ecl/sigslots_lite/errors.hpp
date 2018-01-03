/**
 * @file /include/ecl/sigslots_lite/errors.hpp
 *
 * @brief Error handling types and classes for lite sigslots.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_LITE_ERRORS_HPP_
#define ECL_SIGSLOTS_LITE_ERRORS_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace lite {
namespace sigslots {

/**
 * @brief Error flags for lite sigslots.
 *
 * This is a subset of the ecl_errors class, reproduced here for
 * semi-compatibility. Try and keep these matched with the ErrorFlag
 * enums in ecl_errors for consistency.
 */
enum ErrorFlag {
	NoError = 0,				/**< No error (often a meaningful error state in itself). **/
	UnknownError = -1,        	/**< Unknown error type. **/
	OutOfResourcesError = -21, 	/**< Out of resources, cannot proceed. **/
};

/**
 * @brief Extends the generic ecl error handler with some time specific error strings.
 *
 * This error handler is used both by ecl_sigslots_lite
 */
class Error {
public:
	/**
	 * @brief Configures the error class with the specified error flag.
	 *
	 * @param flag : the error type.
	 */
	Error(const sigslots::ErrorFlag& flag = UnknownError) : error_flag(flag) {}
	/**
	 * @brief The flag identifying the error identified with this error handler.
	 * @return ErrorFlag : the error flag.
	 */
	virtual sigslots::ErrorFlag flag() const { return error_flag; }
	/**
	 * @brief A simple string verbosely representing the error that is handled.
	 *
	 * @return const char* : verbose representation of the error.
	 */
	virtual const char* what() const {
		switch (error_flag) {
			case (NoError) : { return noErrorString(); }
			case (OutOfResourcesError) : { return outOfResourcesErrorString(); }
			default : { return unknownErrorString(); }
		}
	}
private:
	const char* noErrorString() const { return "No error."; }
	const char* unknownErrorString() const { return "Unknown error."; }
	const char* outOfResourcesErrorString() const { return "You cannot add any more to this signal/slot interface (capacity is already fully utilised)."; }
	ErrorFlag error_flag;

};

} // namespace sigslots
} // namespace lite
} // namespace ecl

#endif /* ECL_SIGSLOTS_LITE_ERRORS_HPP_ */
