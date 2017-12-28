/**
 * @file /include/ecl/converters/string.hpp
 *
 * @brief Type conversions to c++ strings.
 *
 * A variety of functors that do type conversion to c++ strings. These
 * are convenient, but slow ( unlike the char* converters).
 *
 * It could probably use extensions for the other integral types. That would
 * require making a decent base class and inheriting across all the
 * integral types.
 *
 * @date April 2009
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONVERTERS_STRINGS_HPP_
#define ECL_CONVERTERS_STRINGS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include "converter.hpp"
#include "char.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** String Converter Interface
*****************************************************************************/
/**
 * @brief Converts int to string, convenient but slow.
 *
 * Slightly slower but convenient (no need for preallocating buffers)
 * conversion of int to a string. This
 * implementation is based on a technique similarly used in the stlsoft
 * libraries.
 *
 * @sa Converter
 **/
template <>
class Converter<std::string,int> : public converters::ConverterBase {
public:
	/**
	 * @brief Converts an integer into a string.
	 *
	 * This is slow in comparison to the
	 * buffered character string functions. There is no
	 * need for error handling here as it will always work.
	 *
	 * @param input : the integer to be converted.
	 * @return string : the string representation of the integer.
	 **/
	std::string operator ()(const int &input)
	{
		std::string s;
		bool negative = false;
		char *char_string;
		#if ECL_SIZE_OF_INT == 4
			char_string = buffer + 11;
		#elif ECL_SIZE_OF_INT == 8
			char_string = buffer + 20;
		#else // just make sure we have lots of space
			char_string = buffer + 30;
		#endif
		*char_string = '\0';
		int remaining = input;
		unsigned int lsd;
		/*********************
		** Fix sign
		**********************/
		if (remaining < 0 ) {
			negative = true;
			remaining *= -1;
		}
		do
		{
			--char_string;
			lsd = remaining%10;
			remaining /= 10;
			*char_string = Converter<char,unsigned int>()(lsd);
		} while ( remaining > 0);
		if ( negative ) {
			--char_string;
			*char_string = '-';
		}
		return s.assign(char_string);
	};

    virtual ~Converter() {}

private:
	#if ECL_SIZE_OF_INT == 4
		char buffer[12];
	#elif ECL_SIZE_OF_INT == 8
		char buffer[21];
	#else // just make sure we have lots of space
		char buffer[31];
#endif
};

/**
 * @brief Convenient, but slow conversions to text format with strings.
 *
 * This groups all the string converters of fundamental types into a single
 * cohesive superclass. At times, this is more convenient than using the
 * string converters individually.
 *
 * @sa Converter
 **/
template <>
class Converter<std::string,void>  :
        public Converter<std::string,int>
{
public:
    virtual ~Converter() {}

	using Converter<std::string,int>::operator();
};


}; // Namespace ecl


#endif /*ECL_CONVERTERS_STRINGS_HPP_*/
