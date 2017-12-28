/**
 * @file /ecl_converters/src/examples/float_converters.cpp
 *
 * @brief Demo for the float style precision converters
 *
 * @date March 2011
 **/

#include <iostream>
#include "../../include/ecl/converters/char_strings.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {


    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Preassigned Buffer" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

	char buf[4];
	ecl::Converter<char*,float> converter(buf, buf+4);
	std::cout << "converter(2.134): " << converter(2.134) << std::endl;
	std::cout << "converter(2.134,2): " << converter(2.134,2) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Internal Default Buffer" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "The default internal buffer is size 250, but lets set 125." << std::endl;
	ecl::Converter<char*,float> converter2(125);
	std::cout << "converter(2.134): " << converter2(2.134) << std::endl;
	std::cout << "converter(2.134,2): " << converter2(2.134,2) << std::endl;
	ecl::Converter<char*,double> converter3;
	std::cout << "converter(2.134): " << converter3(2.134) << std::endl;
	std::cout << "converter(2.134,2): " << converter3(2.134,2) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Playing with strings" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::string s0(converter2(2.134,2));
    std::string s1 = converter2(2.134,2);
	std::cout << "String creation: " << s0 << std::endl;
	std::cout << "String creation: " << s1 << std::endl;

	ecl::Converter<char*,int> int_converter;
	std::cout << int_converter(3) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                       Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;
    return 0;
}
