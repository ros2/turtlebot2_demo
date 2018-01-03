/**
 * @file /src/lib/detail/character_buffer.cpp
 *
 * @brief Implementation for device buffers.
 *
 * @date November 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../../include/ecl/devices/detail/character_buffer.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
namespace devices {

/*****************************************************************************
** Implementation [CharBuffer]
*****************************************************************************/


bool CharBuffer::full() const {
	if ( fill_point_marker == buffer_size ) {
		return true;
	} else {
		return false;
	}
}

long CharBuffer::append(const char& c) {

	if ( full() ) {
		return 0;
	} else {
		contents[fill_point_marker] = c;
		fill_point_marker++;
		return 1;
	}
}

long CharBuffer::append(const char* s, unsigned long n) {

	if ( n <= remaining() ) {
		memcpy(&contents[fill_point_marker], s, n);
		fill_point_marker += n;
		return n;
	} else {
		unsigned int num_to_copy = remaining();
		memcpy(&contents[fill_point_marker], s, num_to_copy);
		fill_point_marker += num_to_copy;
		return num_to_copy;
	}
}

void CharBuffer::clear() {
	fill_point_marker = 0;
}


/*****************************************************************************
** Implementation [CharStringBuffer]
*****************************************************************************/

bool CharStringBuffer::full() const {
	if ( fill_point_marker == buffer_size ) {
		return true;
	} else {
		return false;
	}
}

long CharStringBuffer::append(const char& c) {

	if ( full() ) {
		return 0;
	} else {
		contents[fill_point_marker] = c;
		fill_point_marker++;
		return 1;
	}
}

long CharStringBuffer::append(const char* s, unsigned long n) {

	if ( n <= remaining() ) {
		memcpy(&contents[fill_point_marker], s, n);
		fill_point_marker += n;
		return n;
	} else {
		unsigned int num_to_copy = remaining();
		memcpy(&contents[fill_point_marker], s, num_to_copy);
		fill_point_marker += num_to_copy;
		return num_to_copy;
	}
}

void CharStringBuffer::clear() {
	fill_point_marker = 0;
}

const char* CharStringBuffer::c_str() {
	contents[fill_point_marker] = '\0';
	return &contents[0];
}


} // namespace devices
} // namespace ecl

