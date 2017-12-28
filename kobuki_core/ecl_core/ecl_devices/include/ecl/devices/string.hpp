/**
 * @file /include/ecl/devices/string.hpp
 *
 * @brief A virtual string device.
 *
 * @date December 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_STRING_HPP_
#define ECL_DEVICES_STRING_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include "traits.hpp"
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [String]
*****************************************************************************/
/**
 * @brief A virtual device for manipulating strings.
 *
 * Device for streaming to and from a string. Do not use this class directly,
 * rather use the string stream class instead. It has flexible memory storage
 * and will grow as needed to (similar to the c++ string class) but the
 * growing algorithm isn't really intelligent - just jumps in leaps of 256.
 *
 * Reading and writing are simplified. Two separate pointers are used to
 * designate writing and reading locations on the internal buffer.
 *
 * Reading does not remove or modify the internal
 * buffer, it simply shifts the read location pointer along the string.
 *
 * Writing always appends and reading has nothing to do whatsoever with the
 * state of the write pointer.
 **/
class ecl_devices_PUBLIC String {
public:
	/**
	 * @brief Initialises the string device.
	 *
	 * Configures the strings buffer with the input character string.
	 * If unspecified, it leaves the internal buffer empty.
	 *
	 * @param str : the input string to store in the buffer.
	 **/
	explicit String(const char* str = "");
	/**
	 * @brief Cleans up memory allocations.
	 *
	 * Cleans up the memory allocated to the buffer.
	 */
	virtual ~String();

	/*********************
	** String Interface
	**********************/
	/**
	 * @brief Character string representation of the device's contents.
	 *
	 * This returns a null terminates string representing the contents of the
	 * device's internal buffer.
	 *
	 * @return const char* : pointer to the device's internal buffer.
	 */
	const char* c_str();
	/**
	 * @brief String representation of the device's contents.
	 *
	 * This generates a c++ style string representing the contents of the
	 * device's internal buffer.
	 *
	 * @return string : string representation of the device's internal buffer.
	 */
	std::string str();
	/**
	 * @brief Clears the device's internal buffers.
	 *
	 * Clears the internal character buffer and resets read/write location
	 * pointers.
	 */
	void clear();

	/******************************************
	** Device Source Interface
	*******************************************/
	/**
	 * @brief Read a character from the string device.
	 *
	 * Reads a single character from the string device.
	 *
	 * @param c : character to read into from the string device.
	 * @return long : number of bytes written.
	 **/
	long read(char &c);
	/**
	 * @brief Reads a character string from the string device.
	 *
	 * Reads a character string from the string device.
	 *
	 * @param s : character string to read into from the string device.
	 * @param n : the number of bytes to read.
	 **/
	long read(char* s, unsigned long n);
	/**
	 * @brief Specifies the number of characters remaining to be read..
	 *
	 * Determines the number of unread characters left in the internal
	 * buffer.
	 *
	 * @return unsigned long : number of characters remaining.
	 */
	unsigned long remaining();

	/******************************************
	** Device Sink Interface
	*******************************************/
	/**
	 * @brief Write a character to the buffer.
	 *
	 * Write a character to the buffer. It will automatically
	 * grow the buffer if necessary.
	 *
	 * @param c : the character to write.
	 * @return long : the number of bytes written.
	 **/
	long write(char c);
	/**
	 * @brief Write a character string to the buffer.
	 *
	 * Write a character string to the buffer. It will automatically
	 * grow the buffer if necessary.
	 *
	 * @param s : points to the beginning of the character string
	 * @param n : the number of characters to write.
	 * @return long: the number of bytes written.
	 * @exception StandardException : throws if flushing returned an error [debug mode only].
	 **/
	long write(const char* s, unsigned long n);

	/******************************************
	** Device Seekable Interface
	*******************************************/
	/**
	 * @brief Number of characters stored in the buffer.
	 *
	 * This returns the size of the buffer - note that this is different
	 * to the number of characters remaining to be read. It is the sum
	 * of both read and unread parts.
	 *
	 * @return unsigned long : number of characters stored in the buffer.
	 */
	unsigned long size();
	/**
	 * @brief Unused, but required api for an output device.
	 *
	 * This is not used for string devices, but necessary for the
	 * concept definition of an output device.
	 */
	void flush() {};

	/*********************
	** Device Interface
	**********************/
	/**
	 * @brief Unused, but required api for an ecl device.
	 *
	 * This is not used for string devices, but necessary for the
	 * concept definition of an ecl device.
	 *
	 * @return bool : always returns true.
	 */
	bool open() { return true; }; /**< Redundant api for the string device. **/
	/**
	 * @brief Unused, but required api for an ecl device.
	 *
	 * This is not used for string devices, but necessary for the
	 * concept definition of an ecl device.
	 *
	 * @return bool : always returns true.
	 */
	bool isOpen() { return true; }; /**< Redundant api for the string device. **/

private:
	unsigned long buffer_length; // Actual reserved memory is this +1
	char *buffer;
	char* buffer_cur_write;
	char* buffer_cur_read;
	/**
	 * @brief Grow the buffer by the specified amount.
	 *
	 * Grows the buffer when necessary. This currently just grows it by 256
	 * bytes whenever room runs out (doesn't actually grow, creates a new
	 * section in memory and copies everything over).
	 *
	 * @param no_bytes : the number of bytes to grow by.
	 */
	void grow(int no_bytes = 256);

};

/*****************************************************************************
** Traits
*****************************************************************************/

/**
 * @brief String sink (output device) trait.
 *
 * Specialisation for the serial sink (output device) trait.
 */
template <>
class is_sink<String> : public True {};

/**
 * @brief String sink (input device) trait.
 *
 * Specialisation for the serial sink (input device) trait.
 */
template <>
class is_source<String> : public True {};

/**
 * @brief String sourcesink (input-output device) trait.
 *
 * Specialisation for the serial sourcesink (input-output device) trait.
 */
template <>
class is_sourcesink<String> : public True {};

} // namespace ecl


#endif /* ECL_DEVICES_STRING_HPP_ */
