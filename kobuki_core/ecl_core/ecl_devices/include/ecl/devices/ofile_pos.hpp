/**
 * @file /include/ecl/devices/ofile_pos.hpp
 *
 * @brief Posix interface for synchronous file io.
 *
 * @date September 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_OFILE_POS_HPP_
#define ECL_DEVICES_OFILE_POS_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#if defined(ECL_IS_POSIX)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio> // FILE calls
#include <string>
#include <ecl/concepts/containers.hpp>
#include <ecl/exceptions/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "detail/exception_handler_pos.hpp"
#include "modes.hpp"
#include "traits.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [Output File]
*****************************************************************************/
/**
 * @brief The standard output file device for the ecl.
 *
 * This device uses the underlying posix FILE streams for synchronised
 * file output using the usual write
 * interface with your preference of RAII/non-RAII style interface. In either
 * case, the class automatically does the cleanup for you in the destructor.
 *
 * This is the class to use as your standard, generic logger.
 *
 * <b>Usage</b>:
 *
 * The serial device does both input and output, using the usual read/write
 * interface with your preference of RAII/non-RAII style interface. In either
 * case, the serial class does the cleanup for you in the destructor.
 *
 * Files can be opened either as new files or to be appended to using the
 * appropriate enum
 *
 * @code
 * File file_raii("raii.txt",New); // RAII
 * File file_nonraii;
 * file_nonraii.open("nonraii.txt",Append); // non-RAII
 * @endcode
 *
 * Writing:
 *
 * @code
 * file.write("Dude\n",5);
 * @endcode
 *
 * <b>Performance</b>:
 *
 * Run the bench_files.cpp for a comparison of different techniques. It is
 * definitely faster than both standard posix open/close because it makes
 * use of system buffers, and the c++ ifstreams (these are really slow).
 *
 * <b>Error Handling</b>:
 *
 * OFile provides detailed error messages via exceptions. Note that
 * everything but open() throws exceptions in debug mode only (for
 * performance reasons).
 *
 * If exceptions are disabled, check the result of each function call
 * (usually a boolean or a check for '-1') and then ascertain the
 * last error via the error() method.
 *
 * @sa @ref WriteMode "WriteMode".
 */
class OFile {
public:
	/*********************
	** C&D
	**********************/
	/**
	 * @brief Non-RAII style constructor, doesn't open a file.
	 *
	 * Sometimes its more convenient to open manually rather than inside
	 * constructors. Use this with the open() command to do so. You can
	 * check for open status via the isOpen accessor.
	 */
	OFile();
	/**
	 * @brief Opens a file for writing, RAII style.
	 *
	 * Accepts the filename, write mode (New or Append) and attempts to open
	 * the file for writing. It will throw an exception if the open attempt fails.
	 * Refer to open() for more details.
	 *
	 * @param file_name : name of the file to open.
	 * @param mode : mode of writing, either New or Append.
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	OFile(const std::string &file_name, const WriteMode &mode = New) ecl_throw_decl(StandardException);

	/**
	 * @brief Synchronises the file buffers with the hard disk and cleans up.
	 */
	virtual ~OFile();

	/*********************
	** Open/Close
	**********************/
	/**
	 * @brief Status flag indicating if the file is open/closed.
	 *
	 * @return bool : true if open, false otherwise.
	 */
	virtual bool open() { return ( file != NULL ) ? true : false; }

	/**
	 * @brief Opens the file for writing.
	 *
	 * Configures and opens the file for writing in either New or Append mode.
	 * Note, that you don't need to do this if you constructed this class with the RAII
	 * style constructor as it gets automatically called.
	 *
	 * This function will either throw, or return false and set the error() flag
	 * if exceptions are disabled. Error flag values for this function include:
	 *
	 * - InvalidArgError
	 * - PermissionsError
	 * - OutOfResourcesError
	 * - InterruptedError
	 * - MemoryError
	 * - UsageError
	 * - SystemFailureError
	 * - InvalidObjectError
	 *
	 * @sa devices::open_error, devices::open_exception
	 *
	 * @param file_name : name of the file to open.
	 * @param mode : mode of writing, either New or Append.
	 * @return success or failure (check error()) of the opening.
	 *
	 * @exception StandardException : throws if the connection failed to open.
	 */
	virtual bool open(const std::string &file_name, const WriteMode &mode = New) ecl_throw_decl(StandardException);

	/**
	 * @brief Closes the file.
	 *
	 * The destructor automatically handles this, so in most cases its redundant.
	 * However there are times when a file must be manually closed.
	 *
	 * This function will either throw, or return false and set the error() flag
	 * if exceptions are disabled. Error flag values for this function include:
	 *
	 * - InvalidArgError
	 * - SystemFailureError,
	 * - InterruptedError
	 *
	 * @sa devices::close_error, devices::close_exception
	 *
	 * @exception StandardException : throws if closing/cleaning up failed.
	 */
	virtual bool close() ecl_throw_decl(StandardException);

	/*********************
	** Utility Methods
	**********************/
	/**
	 * @brief The name of the output file.
	 * @return string : the file name.
	 */
	virtual const std::string& filename() const { return name; }


	/*********************
	** Output Methods
	**********************/
	/**
	 * @brief Write a character to the file buffer.
	 *
	 * This function will either throw, or return -1 and set the error() flag
	 * if exceptions are disabled. Error flag values for this function include:
	 *
	 * - BlockingError
	 * - PermissionsError
	 * - OutOfRangeError
	 * - MemoryError
	 * - OutOfResourcesError
	 * - SystemFailureError
	 * - InterruptedError
	 * - InvalidObjectError
	 *
	 * @sa devices::write_error, devices::write_exception
	 *
	 * @param c : the character to write.
	 * @return long: the number of bytes written (-1 on error).
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	virtual long write(const char &c) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Write a character string to the buffer.
	 *
	 * This function will either throw, or return -1 and set the error() flag
	 * if exceptions are disabled. Error flag values for this function include:
	 *
	 * - BlockingError
	 * - PermissionsError
	 * - OutOfRangeError
	 * - MemoryError
	 * - OutOfResourcesError
	 * - SystemFailureError
	 * - InterruptedError
	 * - InvalidObjectError
	 *
	 * @sa devices::write_error, devices::write_exception
	 *
	 * @param s : points to the beginning of the character string
	 * @param n : the number of characters to write.
	 * @return long: the number of bytes written (-1 on error).
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char* s, unsigned long n) ecl_debug_throw_decl(StandardException);

	/**
	 * @brief Write a byte array to the buffer.
	 *
	 * This is the preferred way to write as it avoids char pointers. This
	 * works for both signed and unsigned char types and the size to write
	 * is referenced from the container's size. Ecl stencils are a good fit
	 * for this api.
	 *
	 * This function will either throw, or return -1 and set the error() flag
	 * if exceptions are disabled. Error flag values for this function include:
	 *
	 * - BlockingError
	 * - PermissionsError
	 * - OutOfRangeError
	 * - MemoryError
	 * - OutOfResourcesError
	 * - SystemFailureError
	 * - InterruptedError
	 * - InvalidObjectError
	 *
	 * @sa devices::write_error, devices::write_exception
	 *
	 * @param byte_array : the byte array to write to the file.
	 * @return long: the number of bytes written (-1 on error).
	 * @tparam ByteArray : a container of bytes satisfying the ByteContainer Concept (see ecl_concepts).
	 *
	 */
	template <typename ByteArray>
	long write(const ByteArray &byte_array) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Flush the internal buffer.
	 *
	 * Flushes the userspace buffers to kernel/disk.
	 *
	 * This function will either throw, or return false and set the error() flag
	 * to UnknownError (the underlying fflush function isn't very verbose)
	 * if exceptions are disabled.
	 *
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	virtual bool flush() ecl_debug_throw_decl(StandardException);

	/**
	 * @brief Reports on the error state of the last operation.
	 */
	const Error& error() const { return error_handler; }

private:
	/*********************
	** Variables
	**********************/
	int file_descriptor;
    FILE *file;
    std::string name;
    Error error_handler;
};

/*****************************************************************************
** Template Implementation
*****************************************************************************/

template <typename ByteArray>
long OFile::write(const ByteArray&byte_array) ecl_debug_throw_decl(StandardException) {

    ecl_compile_time_concept_check(ecl::ByteContainerConcept<ByteArray>);
    if ( !open() ) {
    	ecl_debug_throw(ecl::StandardException(LOC, OpenError, std::string("File ") + name + std::string(" is not open for writing.")));
    	error_handler = OpenError;
    	return -1;
    }
    size_t written = fwrite(&byte_array[0],byte_array.size(),1,file);
    if ( written == 0) {
    	ecl_debug_throw(ecl::StandardException(LOC, WriteError, std::string("Could not write to ") + name + std::string(".")));
    	error_handler = WriteError;
    	return -1;
    }
    error_handler = NoError;
    return byte_array.size()*written; // fwrite returns the number of 'items' written, not bytes!
}


/*****************************************************************************
** Traits [OFile]
*****************************************************************************/

/**
 * @brief File sink (output device) trait.
 *
 * Specialisation for the file sink (output device) trait.
 */
template <>
class is_sink<OFile> : public True {};

} // namespace ecl


#endif /* ECL_IS_POSIX */
#endif /* ECL_DEVICES_OFILE_POS_HPP_ */
