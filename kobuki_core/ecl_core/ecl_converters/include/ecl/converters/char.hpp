/**
 * @file /include/ecl/converters/char.hpp
 *
 * @brief Type conversions to a readable (ascii) char type.
 *
 * A variety of functors that do type conversion to a readable (ascii) char
 * type.
 *
 * @date April 2009
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONVERTERS_CHAR_HPP_
#define ECL_CONVERTERS_CHAR_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "converter.hpp"
#include <ecl/exceptions/standard_exception.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Character Converter Interface
*****************************************************************************/
/**
 * @brief Converts unsigned ints into readable (ascii) characters.
 *
 * @sa Converter
 **/
template <>
class Converter<char,unsigned int> : public converters::ConverterBase {
public:
	virtual ~Converter() {}

	/**
	 * Converts a single unsigned int into a char type. A similar function is used in
	 * stlsoft.
	 *
	 * This throws an exception and/or configures the error()
	 * function for this converter if the input digit is not a char digit ['0'-'9']
	 *
	 * @param input : the input digit to be converted.
	 * @return char : the character representation of the digit.
	 *
	 * @exception : throws if input is not a char digit ['0'-'9'] (debug mode only).
	 **/
	char operator()(const unsigned int &input) ecl_debug_throw_decl(StandardException) {
		if ( input > 9 ) {
			ecl_debug_throw(StandardException(LOC,ConversionError));
			error_handler = ConversionError;
		}
		return ('0'+ input%10);    // Copy the char back.
	}
};
/**
 * @brief Converts ints into readable (ascii) characters.
 *
 * @sa Converter
 **/
template <>
class Converter<char,int> : public converters::ConverterBase {
public:
	virtual ~Converter() {}

	/**
	 * Converts a single int into a char type. A similar function is used in
	 * stlsoft.
	 *
	 * This throws an exception and/or configures the error()
	 * function for this converter if the input digit is not a char digit ['0'-'9']
	 *
	 * @param input : the input digit to be converted.
	 * @return char : the character representation of the digit.
	 *
	 * @exception : throws if input is not a char digit ['0'-'9'] (debug mode only).
	 **/
	char operator()(const int &input) ecl_debug_throw_decl(StandardException) {
		if ( ( input < 0 ) || (input > 9) ) {
			ecl_debug_throw(StandardException(LOC,OutOfRangeError));
			error_handler = OutOfRangeError;
		}
		return ('0'+ input%10);    // Copy the char back.
	}
};

/*****************************************************************************
 * Char Converter Family
 ****************************************************************************/
/**
 * @brief Family of converters to char.
 *
 * This groups all the char converters of fundamental types into a single
 * cohesive superclass.
 *
 * @sa Converter
 **/
template <>
class Converter<char,void>  :
    public Converter<char,int>,
    public Converter<char,unsigned int>
{
    public:
        virtual ~Converter() {}

        using Converter<char,int>::operator();
        using Converter<char,unsigned int>::operator();
};

}; // Namespace ecl


#endif /*ECL_CONVERTERS_CHAR_HPP_*/
