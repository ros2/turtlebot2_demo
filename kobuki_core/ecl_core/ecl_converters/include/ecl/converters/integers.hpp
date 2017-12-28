/**
 * @file /include/ecl/converters/integers.hpp
 *
 * @brief Integer conversion functors (shims).
 *
 *  A variety of functors (shims) that do type conversion to integer types.
 *
 * @date March 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONVERTERS_CONVERTERS_INTEGERS_HPP_
#define ECL_CONVERTERS_CONVERTERS_INTEGERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <sstream>
#include <ecl/exceptions/standard_exception.hpp>
#include "converter.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Conversion from a string to an integer type.
 *
 * Very quick conversions to an int type. These are partially based on the
 * stlsoft techniques.
 **/
template <>
class Converter<int,std::string> : public converters::ConverterBase {
public:
	virtual ~Converter() {};
	/**
	 * @brief Converts a string to an integer type.
	 *
	 * This throws an exception and/or configures the error()
	 * function for this converter if the input does not consist
	 * purely of the appropriate digits for an integer.
	 *
	 * @param input : the input string to be converted.
	 * @return int : the integer representation, INT_MIN on error.
	 * @exception StandardException : throws if conversion failed (debug mode only).
	 **/
	int operator()(const std::string &input) ecl_debug_throw_decl(StandardException) {
		int i;
		std::istringstream stream(input);
		char c;
		if ( !( stream >> i ) || (stream.get(c))) { // Second part checks that there isn't leftover characters
			ecl_debug_throw(StandardException(LOC,ConversionError));
			error_handler = ConversionError;
		}
		return i;
	}
};
/**
 * @brief Conversion from a char to an integer type.
 *
 * This is of course restricted to characters ['0'-'9'].
 *
 * @exception StandardException : throws if character is outside the range ['0'-'9'].
 **/
template <>
class Converter<int,char> : public converters::ConverterBase {
public:
	/**
	 * @brief Converts a char to an integer type.
	 *
	 * This throws an exception and/or configures the error()
	 * function for this converter if the input is not a char digit ['0'-'9'].
	 *
	 * @param c : the input char to be converted [0-9].
	 * @return int : the integer representation of the digit (-1 on failure).
	 * @exception StandardException : throws if conversion failed (debug mode only).
	 **/
	int operator()(char c) ecl_debug_throw_decl(StandardException) {
		if ( ( c < '0' ) || (c > '9') ) {
			ecl_debug_throw(StandardException(LOC,ConversionError));
			error_handler = ConversionError;
		}
		return static_cast<int>(c - '0');
	}
};

/**
 * @brief Conversion from a byte array to an integer type.
 *
 * Converts from a byte array to an integer type. Be sure that the length of
 * the byte array is large enough for the required transformation to an integer.
 *
 * @sa Converter
 **/
template <>
class Converter<int,unsigned char*> {
private:
	Converter() {
		// This class is being depracated! Use the byte array converters instead (converters/from_byte_array.hpp)
	}
public:
//	/**
//	 * Converts a byte array to an integer type. The bytes are ordered from
//	 * least significant to most significant (little-endian). Warning: the
//	 * byte array must be the same size as the integer that it is being
//	 * converted to (i.e. usually 4 bytes).
//	 *
//	 * @param byte_array : the input string to be converted.
//	 * @return int : the integer representation of the digit.
//	 **/
//	int operator()(const unsigned char* byte_array) {
//		int value = 0;
//		for (unsigned int i = 0; i < sizeof(int); ++i ) {
//			value |= *(byte_array+i) << 8*i;
//		}
//		return value;
//	}
};
/**
 * @brief Conversion from a char array to an integer type.
 *
 * Converts from a char string to an integer type. Be sure that the length of
 * the char array is large enough for the required transformation to an integer.
 *
 * @sa Converter
 **/
template <>
class Converter<int,char*> : public converters::ConverterBase {
private:
	Converter() {
		// This class is being depracated! Use the byte array converters instead (converters/from_byte_array.hpp)
	}
public:
// NEW
//        /**
//         * Converts a c string to an integer type. This converts
//         * ascii characters to an integer, not bytes to integer. For that
//         * see the byte array converters in this package.
//         *
//         * @todo Add checks to ensure the range of int is not exceeded.
//         *
//         * @param c_str : the input string to be converted.
//         * @return int : the integer representation of the digit.
//         *
//         * @exception : throws in debug mode if a non-digit char is found (debug mode only).
//         **/
//        int operator()(const char* c_str) ecl_debug_throw_decl(StandardException) {
//        	int value = 0;
//        	bool negative_flag = false;
//        	unsigned int starting_index = 0;
//        	if ( c_str[0] == '-' ) {
//        		negative_flag = true;
//        		starting_index = 1;
//        	}
//        	size_t length = ::strlen(c_str);
//        	int multiplier = 1;
//        	for ( unsigned int i = length-1; i >= starting_index; --i ) {
//                if ( ( c_str[i] < '0' ) || (c_str[i] > '9') ) {
//                    ecl_debug_throw(StandardException(LOC,ConversionError));
//                    error_handler = ConversionError;
//                }
//                value += multiplier*static_cast<int>(c_str[i] - '0');
//                multiplier *= 10;
//        	}
//        	if ( negative_flag) {
//        		value *= -1;
//        	}
//            return value;
//        }
// OLD
//	/**
//	 * Converts a char array to an integer type. The bytes are ordered from
//	 * least significant to most significant (little-endian). Warning: the
//	 * byte array must be the same size as the integer that it is being
//	 * converted to (i.e. usually 4 bytes).
//	 *
//	 * @param byte_array : the input string to be converted.
//	 * @return int : the integer representation of the digit.
//	 **/
//	int operator()(const char* byte_array) ecl_debug_throw_decl(StandardException) {
//		if ( ::strlen(byte_array) != sizeof(int) ) {
//			ecl_debug_throw(StandardException(LOC,InvalidInputError,"Input char string length is too large/small for conversion to int."));
//		}
//		int value = 0;
//		for (unsigned int i = 0; i < sizeof(int); ++i ) {
//			value |= static_cast<unsigned char>(*(byte_array+i)) << 8*i;
//		}
//	return value;
//	}
};

/**
 * @brief Conversions to integers.
 *
 * This groups all the string converters of fundamental types into a single
 * cohesive superclass. At times, this is more convenient than using the
 * string converters individually.
 *
 * @sa Converter
 **/
template <>
class Converter<int,void>  :
        public Converter<int,std::string>,
        public Converter<int,char>
//        public Converter<int,unsigned char*>,
//        public Converter<int,char*>
{
    public:
        using Converter<int,std::string>::operator();
        using Converter<int,char>::operator();
//        using Converter<int,unsigned char*>::operator();
//        using Converter<int,char*>::operator();
};


} // namespace ecl

#endif /* ECL_CONVERTERS_INTEGERS_HPP_ */
