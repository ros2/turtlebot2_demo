/**
 * @file /include/ecl/devices/shared_file.hpp
 *
 * @brief Output file specially suited for logging across threads.
 *
 * Output file specially suited for logging across threads.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_SHARED_FILE_HPP_
#define ECL_DEVICES_SHARED_FILE_HPP_

/*****************************************************************************
** Cross Platform Functionality
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <string>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/threads/mutex.hpp>
#include "detail/character_buffer.hpp"
#include "ofile.hpp"
#include "traits.hpp"
#include "modes.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Definition
*****************************************************************************/

class SharedFile;

/*****************************************************************************
** Interfaces
*****************************************************************************/

namespace devices {

/*****************************************************************************
** Interface [SharedFileCommon]
*****************************************************************************/
/**
 * @brief Interface distributed to all members of a shared output file.
 *
 * Contains the objects required to be distributed amongst all members
 * of a shared output file.
 */
class SharedFileCommon {
public:
    SharedFileCommon() : error_handler(NoError) {};
	/**
	 * @brief Automatically opens a file and initialises the count.
	 *
	 * Automatically opens a file and initialises the count.
	 *
	 * @param name : file name.
	 * @param mode : writing mode (either New or Append).
	 */
    SharedFileCommon(const std::string &name, ecl::WriteMode mode) ecl_throw_decl(StandardException);
    virtual ~SharedFileCommon() {}

    friend class ecl::SharedFile;
    friend class SharedFileManager;

private:
    unsigned int count;
    ecl::Mutex mutex;
    OFile file;
	Error error_handler;
};

class SharedFileManager {
public:
	static SharedFileCommon* RegisterSharedFile(const std::string &name, ecl::WriteMode mode = New) ecl_throw_decl(StandardException);
	static bool DeRegisterSharedFile(const std::string &name) ecl_throw_decl(StandardException);
private:
	static ecl::Mutex mutex;
	static std::map<std::string,SharedFileCommon*> opened_files;
};

} // namespace devices

/*****************************************************************************
** Interface [SharedFile]
*****************************************************************************/
/**
 * @brief Multi-instance (also thread-safe) output file device.
 *
 * This allows multiple instantiations/connections (uniquely identified by string file name)
 * to an OFile device. Each instantiation has its own buffer that protects
 * data from multiple sources getting unnecessarily tangled. It is also thread-safe.
 *
 * <b>Usage:</b>
 *
 * Usage is exactly the same as that for regular OFile devices except that no new
 * files are made if the same file name is used (instead, instantiations share
 * this file). Everything else happens under the hood and cleanup occurs in the
 * destructors.
 *
 * @sa OFile.
 */
class ecl_devices_PUBLIC SharedFile {
public:
	/*********************
	** C&D
	**********************/
	/**
	 * @brief Non-RAII style constructor, doesn't open a file.
	 *
	 * Sometimes its more convenient to open manually rather than inside
	 * constructors. Use this with the open() command to do so. You can
	 * check for open status via the open accessor.
	 */
	SharedFile() {};
	/**
	 * @brief Either opens a file for writing or a link to an existing shared instance.
	 *
	 * If a file by this name is not yet being shared, then it opens it directly.
	 * Otherwise it opens a link to the shared instance. Note, you may ignore
	 * the second argument (mode) if you are simply opening a second link to the
	 * shared instance.
	 *
	 * @param name : filename.
	 * @param mode : mode for writing (New, Append), this must be the same for all instances.
	 *
	 * @exception StandardException : throws if the file could not be opened.
	 */
	SharedFile(const std::string &name, WriteMode mode = New) ecl_throw_decl(StandardException);
	/**
	 * @brief Automatic cleaner for shared files.
	 *
	 * Automatically cleans up and closes io handles.
	 */
	virtual ~SharedFile();

	/**
	 * @brief Opens the file for writing.
	 *
	 * Configures and opens the file for writing in either New or Append mode.
	 * Note, that you don't need to do this if you constructed this class with the RAII
	 * style constructor as it gets automatically called.
	 *
	 * Error handling the same for OFile's open() call.
	 *
	 * @param name : name of the file to open.
	 * @param mode : mode of writing, either New or Append.
	 * @exception StandardException : throws if the connection failed to open.
	 * @sa OFile
	 */
	bool open(const std::string &name, WriteMode mode = New) ecl_throw_decl(StandardException);

	/*********************
	** Shared File Methods
	**********************/
	/**
	 * @brief Denotes how many instances are currently sharing this file.
	 *
	 * Denotes how many instances are currently sharing this file.
	 *
	 * @return unsigned int : the number of shared instances.
	 */
	unsigned int count() { return shared_instance->count; }

	/*********************
	** OutputDevice Methods
	**********************/
	/**
	 * @brief Status flag indicating if the file is open/closed.
	 *
	 * @return bool : true if open, false otherwise.
	 */
	bool open() { return shared_instance->file.open(); }
	/**
	 * @brief Write a character to the buffer.
	 *
	 * Write a character to the buffer. It will automatically flush (write to
	 * the shared ofile instance) if the buffer exceeds its capacity.
	 *
	 *  Error handling the same for OFile's write function.
	 *
	 * @param c : the character to write.
	 * @return long : the number of bytes written.
	 *
	 * @exception StandardException : throws from the underlying file if writing returned an error [debug mode only].
	 * @sa OFile
	 **/
	long write(const char &c) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Write a character string to the buffer.
	 *
	 * Write a character string to the buffer. It will automatically flush (write to
	 * the shared ofile instance) if the buffer exceeds its capacity.
	 *
	 *  Error handling the same for OFile's write function.
	 *
	 * @param s : points to the beginning of the character string.
	 * @param n : the number of characters to write.
	 * @return long: the number of bytes written.
	 *
	 * @exception StandardException : throws from the underlying file if flushing returned an error [debug mode only].
	 * @sa OFile
	 **/
	long write(const char* s, unsigned long n) ecl_debug_throw_decl(StandardException);
	/**
	 * @brief Flush the internal buffer.
	 *
	 * This writes to the shared file. Error handling the same for OFile's write function.
	 *
	 * @exception StandardException : throws from the underlying file if flushing returned an error [debug mode only].
	 * @sa OFile
	 **/
	bool flush() ecl_debug_throw_decl(StandardException);

	const Error& error() const { return shared_instance->error_handler; }

private:
	devices::SharedFileCommon* shared_instance;
	devices::CharBuffer buffer;
};

/*****************************************************************************
** Traits [SharedFile]
*****************************************************************************/
/**
 * @brief File sink (output device) trait.
 *
 * Specialisation for the file sink (output device) trait.
 */
template <>
class is_sink<SharedFile> : public True {};

} // namespace ecl

#endif /* ECL_DEVICES_SHARED_FILE_HPP_ */
