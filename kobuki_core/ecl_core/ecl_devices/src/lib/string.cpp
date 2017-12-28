/**
 * @file /src/lib/string.cpp
 *
 * @brief Implementation of a virtual string device.
 *
 * @date December 2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <cstring>
#include <string>
#include "../../include/ecl/devices/string.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [String]
*****************************************************************************/
String::String(const char* str)
{
    buffer_length = strlen(str) + 1;
    buffer = new char[buffer_length]; // Need +1 so we can attach \0 to produce c_str()
    memcpy(buffer,str,buffer_length-1);
    buffer_cur_write = buffer + buffer_length - 1;
    buffer_cur_read = buffer;
}
String::~String() {
    delete [] buffer;
}
const char* String::c_str() {
    *buffer_cur_write = '\0'; // Null terminate
    return buffer;
}
std::string String::str() {
	std::string s;
    s.assign(buffer,size());
    return s;
}

void String::grow(int no_bytes) {
    char* new_buffer;
    unsigned long cur_pos = buffer_cur_read - buffer;
    unsigned long cur_end = buffer_cur_write - buffer;
    new_buffer = new char[size()+no_bytes+1];
    memcpy(new_buffer,buffer,size());
    delete [] buffer;
    buffer = new_buffer;
    buffer_cur_read = buffer+cur_pos;
    buffer_cur_write = buffer+cur_end;
}

/*****************************************************************************
** Implementation [String][Source Interface]
*****************************************************************************/
long String::read(char &c) {
    if ( remaining() != 0 ) {
        c = *buffer_cur_read;
        ++buffer_cur_read;
        return 1;
    } else {
        return 0;
    }
}
/**
 * Read a character string from the buffer.
 * @param s : points to the beginning of the character string
 * @param n : the number of characters to read
 * @return long : number of characters read, negative on error.
 **/
long String::read(char* s, unsigned long n)
{
    unsigned long rem = remaining();

    if ( rem > n ) {
        memcpy(s,buffer_cur_read,n);
        buffer_cur_read += n;
        return n;
    } else if ( rem != 0 ) {
        memcpy(s,buffer_cur_read,rem);
        buffer_cur_read += rem;
        return rem;
    } else { // rem = 0;
        return 0;
    }
}
/**
 * Remaining bytes (from the pointer's current position in the
 * buffer until its end).
 * @return unsigned long : remaining bytes in the string's buffer.
 **/
unsigned long String::remaining()
{
    return buffer_cur_write - buffer_cur_read;
};

/**
 * Clear the contents of the string device (clears and
 * deletes the buffer).
 **/
void String::clear()
{
    delete [] buffer;
    buffer = new char[1];
    buffer_length = 0;
    buffer_cur_write = buffer;
    buffer_cur_read = buffer;
};

/*****************************************************************************
** Implementation [String][Sink Interface]
*****************************************************************************/
/**
 * Write a character to the buffer - we always append.
 * @param c : the character.
 * @return long : number of characters written, negative on error.
 **/
long String::write(char c)
{
    // Remember that the last position in the buffer is for the char string terminator
    if ( buffer_cur_write-buffer == static_cast<long>(buffer_length) -1) {
        grow();
    }
    *buffer_cur_write = c;
    ++buffer_cur_write;
    return 1;
}
/**
 * Write a character string to the buffer - we always append.
 * @param s : points to the beginning of the character string
 * @param n : the number of characters to write.
 * @return long : number of characters written, negative on error.
 **/
long String::write(const char* s, unsigned long n)
{
    // Remember that the last position in the buffer is for the char string terminator
    if ( buffer_cur_write-buffer > static_cast<long>(buffer_length) - 1 - static_cast<long>(n) ) {
        grow(n+256);
    }
    memcpy(buffer_cur_write,s,n);
    buffer_cur_write += n;
    return n;
}


/*****************************************************************************
** Implementation [String][Seekable Interface]
*****************************************************************************/
/**
 * Size of the string (bytes).
 * @return long : string size (bytes)
 **/
unsigned long String::size()
{
    return buffer_cur_write - buffer;
}


} // namespace ecl
