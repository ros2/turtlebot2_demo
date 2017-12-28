/**
 * @file /include/ecl/devices/console.hpp
 *
 * @brief Configures ecl devices for standard output/input.
 *
 * @date October 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_CONSOLE_HPP_
#define ECL_DEVICES_CONSOLE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "detail/character_buffer.hpp"
#include "traits.hpp"
#include <ecl/errors/handlers.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [OConsole]
*****************************************************************************/

/**
 * @brief Device for writing to standard output.
 *
 * Device for writing to standard output. It creates a wrapper class around
 * the stdout handle from cstdio to implement an ecl output device. Do not
 * use this directly, it serves only as the device handle for the console
 * stream classes.
 **/
class ecl_devices_PUBLIC OConsole {
public:
	/**
	 * @brief Default constructor.
	 */
	OConsole() : error_handler(NoError) {}
	virtual ~OConsole() {}

	/*********************
	** Output Device API
	**********************/
	/**
	 * @brief Dummy handle to satisfy the output device concept.
	 *
	 * For console devices this does nothing - the standard output is always
	 * open. It is just a dummy handle built to ensure the output device
	 * concept is satisfied.
	 * @return bool : always returns true (it is always open).
	 */
	bool open() { return true; }

	/**
	 * @brief Write a character to the buffer.
	 *
	 * Write a character to the buffer. It will automatically flush if the
	 * buffer exceeds its capacity.
	 *
	 * @param c : the character to write.
	 * @return long : the number of bytes written.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char &c) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Write a character string to the buffer.
	 *
	 * Write a character string to the buffer. It will automatically flush if the
	 * buffer exceeds its capacity.
	 *
	 * @param s : points to the beginning of the character string
	 * @param n : the number of characters to write.
	 * @return long: the number of bytes written.
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	long write(const char* s, unsigned long n) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Flush the internal buffer.
	 *
	 * Flushes the userspace buffers to standard output.
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	void flush() ecl_assert_throw_decl(StandardException);

	/**
	 * @brief Returns the most recent error status.
	 *
	 * Use this to check on the status of the object after a method has been called.
	 * Mostly this is here to serve as a graceful fallback for when exceptions
	 * are disabled.
	 * @return
	 */
	const Error& error() const { return error_handler; }
private:
	devices::CharStringBuffer buffer;
	Error error_handler;
};

/*****************************************************************************
** Interface [EConsole]
*****************************************************************************/

/**
 * @brief Device for writing o standard error.
 *
 * Device for writing to standard error. Do not
 * use this directly, it serves only as the device handle for the console
 * stream classes.
 **/
class ecl_devices_PUBLIC EConsole {
public:
	/**
	 * @brief Default constructor.
	 */
	EConsole() : error_handler(NoError) {}
	virtual ~EConsole() {}

	/*********************
	** Output Device API
	**********************/
	/**
	 * @brief Dummy handle to satisfy the output device concept.
	 *
	 * For console devices this does nothing - the standard output is always
	 * open. It is just a dummy handle built to ensure the output device
	 * concept is satisfied.
	 * @return bool : always returns true (it is always open).
	 */
	bool open() { return true; }

	/**
	 * @brief Write a character to the buffer.
	 *
	 * Write a character to the buffer. It will automatically flush if the
	 * buffer exceeds its capacity.
	 *
	 * @param c : the character to write.
	 * @return long : the number of bytes written.
	 * @exception StandardException : throws if writing returned an error [debug mode only].
	 **/
	long write(const char &c) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Write a character string to the buffer.
	 *
	 * Write a character string to the buffer. It will automatically flush if the
	 * buffer exceeds its capacity.
	 *
	 * @param s : points to the beginning of the character string
	 * @param n : the number of characters to write.
	 * @return long: the number of bytes written.
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	long write(const char* s, unsigned long n) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Flush the internal buffer.
	 *
	 * Flushes the userspace buffers to standard output.
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	void flush() ecl_assert_throw_decl(StandardException);

	/**
	 * @brief Returns the most recent error status.
	 *
	 * Use this to check on the status of the object after a method has been called.
	 * Mostly this is here to serve as a graceful fallback for when exceptions
	 * are disabled.
	 * @return
	 */
	const Error& error() const { return error_handler; }

private:
	devices::CharStringBuffer buffer;
	Error error_handler;
};

/*****************************************************************************
** Interface [IConsole]
*****************************************************************************/

/**
 * @brief Device for reading from standard output.
 *
 * Device for reading from standard output. It creates a wrapper class around
 * the stdout handle from cstdio to implement an ecl output device. Do not
 * use this directly, it serves only as the device handle for the console
 * stream classes.
 *
 * One peculiarity of this device, is that it will strip newlines from the
 * incoming input. This is a characteristic exclusive to the input console
 * device because of its interactive nature with the user.
 **/
class ecl_devices_PUBLIC IConsole {
public:
	/**
	 * @brief Default constructor.
	 */
	IConsole() : error_handler(NoError) {}
	virtual ~IConsole() {}
	/*********************
	** Input Device API
	**********************/
	/**
	 * @brief Dummy handle to satisfy the input device concept.
	 *
	 * For console devices this does nothing - the standard input is always
	 * open. It is just a dummy handle built to ensure the input device
	 * concept is satisfied.
	 * @return bool : always returns true (it is always open).
	 */
	bool open() { return true; }
	/**
	 * @brief Read a character from standard input.
	 *
	 * Reads a single character from standard input.
	 *
	 * @param c : character to read into from standard input.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char &c) ecl_assert_throw_decl(StandardException);
	/**
	 * @brief Reads a character string from standard input.
	 *
	 * Reads a character string from standard input.
	 *
	 * @param s : character string to read into from standard input.
	 * @param n : the number of bytes to read.
	 * @exception StandardException : throws if reading returned an error [debug mode only].
	 **/
    long read(char *s, const unsigned long &n) ecl_assert_throw_decl(StandardException);

//    long remaining() {
//        long pos = ftell(stdin); // get current position marker.
//        fseek(stdin,0,SEEK_END); // move pointer to the end of the file.
//        long size = ftell(stdin);
//        fseek(stdin,pos,SEEK_SET);
//
//        return size;
//    }
	/**
	 * @brief Returns the most recent error status.
	 *
	 * Use this to check on the status of the object after a method has been called.
	 * Mostly this is here to serve as a graceful fallback for when exceptions
	 * are disabled.
	 * @return
	 */
	const Error& error() const { return error_handler; }
private:
	Error error_handler;
};


/*****************************************************************************
** Traits [Console]
*****************************************************************************/
/**
 * @brief Console source (input device) trait.
 *
 * Specialisation for the console source (input device) trait.
 *
 */
template <>
class is_source<IConsole> : public True {

};

/**
 * @brief Console sink (output device) trait.
 *
 * Specialisation for the console sink (output device) trait.
 */
template <>
class is_sink<OConsole> : public True {};

/**
 * @brief Error console sink (output device) trait.
 *
 * Specialisation for the error console sink (output device) trait.
 */
template <>
class is_sink<EConsole> : public True {};

} // namespace ecl

#endif /* ECL_DEVICES_CONSOLE_HPP_ */
