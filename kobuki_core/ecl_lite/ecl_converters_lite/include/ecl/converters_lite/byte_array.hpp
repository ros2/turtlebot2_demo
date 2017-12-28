/**
 * @file include/ecl/converters_lite/byte_array.hpp
 *
 * @brief Conversions between integers and byte arrays.
 *
 * @date March 2011.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONVERTERS_LITE_BYTE_ARRAY_HPP_
#define ECL_CONVERTERS_LITE_BYTE_ARRAY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/portable_types.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** From Byte Array
*****************************************************************************/
/**
 * @brief Converts an array of four char into a 32 bit integer.
 *
 * The bytes are ordered from least significant to most significant
 * (little-endian).
 *
 * Warning: be careful to ensure that there is always
 * at least 4 elements in the char string available for conversion.
 * There is no way of catching this error except via segfault.
 *
 * @param value : the output integer
 * @param byte_array : input array of characters.
 */
void from_byte_array(int32 &value, const char* byte_array) {
	value = 0;
	for (unsigned int i = 0; i < 4; ++i ) {
		value |= static_cast<unsigned char>(*(byte_array+i)) << 8*i;
	}
}

/**
 * @brief Converts a array of four unsigned char into a 32 bit integer.
 *
 * The bytes are ordered from least significant to most significant
 * (little-endian).
 *
 * Warning: be careful to ensure that there is always
 * at least 4 elements in the char string available for conversion.
 * There is no way of catching this error except via segfault.
 *
 * @param value : the output integer
 * @param byte_array : input array of unsigned char.
 */
void from_byte_array(int32 &value, const unsigned char* byte_array) {
	value = 0;
	for (unsigned int i = 0; i < 4; ++i ) {
		value |= static_cast<unsigned char>(*(byte_array+i)) << 8*i;
	}
}

/**
 * @brief Converts a array of four char into a 32 bit integer.
 *
 * The bytes are ordered from least significant to most significant
 * (little-endian).
 *
 * Warning: be careful to ensure that there is always
 * at least 4 elements in the char string available for conversion.
 * There is no way of catching this error except via segfault.
 *
 * @param value : the output integer
 * @param byte_array : input array of char.
 */
void from_byte_array(uint32 &value, const char* byte_array) {
	value = 0;
	for (unsigned int i = 0; i < 4; ++i ) {
		value |= static_cast<unsigned char>(*(byte_array+i)) << 8*i;
	}
}

/**
 * @brief Converts a array of four unsigned char into a 32 bit unsigned int.
 *
 * The bytes are ordered from least significant to most significant
 * (little-endian).
 *
 * Warning: be careful to ensure that there is always
 * at least 4 elements in the char string available for conversion.
 * There is no way of catching this error except via segfault.
 *
 * @param value : the output integer
 * @param byte_array : input array of unsigned char.
 */
void from_byte_array(uint32 &value, const unsigned char* byte_array) {
	value = 0;
	for (unsigned int i = 0; i < 4; ++i ) {
		value |= static_cast<unsigned char>(*(byte_array+i)) << 8*i;
	}
}

} // namespace ecl

#endif /* ECL_CONVERTERS_LITE_BYTE_ARRAY_HPP_ */
