/**
 * @file /include/ecl/devices/detail/character_buffer.hpp
 *
 * @brief A generic buffer for devices.
 *
 * @date September, 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_DEVICES_BUFFER_HPP_
#define ECL_DEVICES_BUFFER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cstring>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

/*****************************************************************************
** Interface [CharBuffer]
*****************************************************************************/
/**
 * @brief Simple char string buffer class for use internally by the devices.
 *
 * This accommodates buffers which always reserve at least one extra
 * byte for the end of string character.
 *
 * @sa @ref ecl::OConsole "OConsole".
 */
class CharBuffer {
public:
	/*********************
	** Static Variables
	**********************/
	static const unsigned int buffer_size = 4096;

	/*********************
	** C&D
	**********************/
	CharBuffer() : fill_point_marker(0) {}
	virtual ~CharBuffer() {}

	unsigned int remaining() { return ( buffer_size - fill_point_marker); }
	unsigned int size() const { return fill_point_marker; }
	bool full() const;
	long append(const char &c);
	long append(const char* s, unsigned long n);
	void clear();
	char* c_ptr() { return contents; }

private:
	/*********************
	** Variables
	**********************/
	unsigned int fill_point_marker; // Index pointing to where you would write the next char
	char contents[buffer_size];
};

/*****************************************************************************
** Interface [CharStringBuffer]
*****************************************************************************/
/**
 * @brief Simple char string buffer class for use internally by the devices.
 *
 * This accommodates buffers which always reserve at least one extra
 * byte for the end of string character.
 *
 * @sa @ref ecl::OConsole "OConsole".
 */
class CharStringBuffer {
public:
	/*********************
	** Static Variables
	**********************/
	static const unsigned int buffer_size = 4095; // Array is actually this + 1

	/*********************
	** C&D
	**********************/
	CharStringBuffer() : fill_point_marker(0) {
		contents[buffer_size] = '\0';
	}

	virtual ~CharStringBuffer() {}

	unsigned int remaining() { return ( buffer_size - fill_point_marker); }
	bool full() const;
	long append(const char &c);
	long append(const char* s, unsigned long n);
	const char* c_str();
	void clear();

private:
	/*********************
	** Variables
	**********************/
	unsigned int fill_point_marker; // Index pointing to where you would write the next char
	char contents[buffer_size+1];
};

} // namespace devices
} // namespace ecl


#endif /* ECL_DEVICES_BUFFER_HPP_ */
