/**
 * @file /src/test/formatters.cpp
 *
 * @brief Test program for the formatters.
 *
 * Can't really feasibly do a unit test on the formatters, so this is
 * more just a coverage test.
 *
 * @date May 2009
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <string>
#include "../../include/ecl/formatters/common.hpp"
#include "../../include/ecl/formatters/floats.hpp"
#include "../../include/ecl/formatters/number.hpp"
#include "../../include/ecl/formatters/strings.hpp"

/*****************************************************************************
** Using
*****************************************************************************/

using std::string;
using ecl::Format;
using ecl::Dec;
using ecl::Fixed;
using ecl::Hex;
using ecl::CentreAlign;
using ecl::LeftAlign;
using ecl::NoAlign;
using ecl::RightAlign;

/*****************************************************************************
** Main
*****************************************************************************/

int main() {

    unsigned char c = 69;
    Format<unsigned char> format(8,RightAlign,Hex); // Initialisation

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Temporary Formatting [Number]" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "NoAlign    : " << format(c,8,NoAlign,Hex) << std::endl;
    std::cout << "LeftAlign  : " << format(c,8,LeftAlign,Hex) << std::endl;
    std::cout << "CentreAlign: " << format(c,8,CentreAlign,Hex) << std::endl;
    std::cout << "RightAlign : " << format(c,8,RightAlign,Hex) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Temporary Formatting [Number]" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "NoAlign    : " << format(c,8,NoAlign,Hex) << std::endl;
    std::cout << "LeftAlign  : " << format(c,8,LeftAlign,Hex) << std::endl;
    std::cout << "CentreAlign: " << format(c,8,CentreAlign,Hex) << std::endl;
    std::cout << "RightAlign : " << format(c,8,RightAlign,Hex) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "               Permanent Formatting [Number]" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "NoAlign    : " << format(8,NoAlign,Hex)(c) << std::endl;
    std::cout << "LeftAlign  : " << format(8,LeftAlign,Hex)(c) << std::endl;
    std::cout << "CentreAlign: " << format(8,CentreAlign,Hex)(c) << std::endl;
    std::cout << "RightAlign : " << format(8,RightAlign,Hex)(c) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "             Single Parameter Formatting [Number]" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "Width      : " << format.width(14)(c) << std::endl;
    std::cout << "Align      : " << format.align(CentreAlign)(c) << std::endl;
    std::cout << "Base       : " << format.base(Dec)(c) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Integral Types" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "char          : " << Format<unsigned char>(8,RightAlign,Dec)('c') << std::endl;
    std::cout << "short         : " << Format<short>(8,RightAlign,Dec)(123) << std::endl;
    std::cout << "unsigned short: " << Format<unsigned short>(8,RightAlign,Dec)(123) << std::endl;
    std::cout << "int           : " << Format<int>(8,RightAlign,Dec)(-111123) << std::endl;
    std::cout << "unsigned int  : " << Format<unsigned int>(8,RightAlign,Dec)(111123) << std::endl;
    std::cout << "long          : " << Format<long>(8,RightAlign,Dec)(-1111123) << std::endl;
    std::cout << "unsigned long : " << Format<unsigned long>(8,RightAlign,Dec)(1111123) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   String Formats" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Format<string> sformat(10,RightAlign);
    std::cout << "Strings: " << sformat(string("Dude")) << std::endl;
    std::cout << "Strings: " << sformat(string("Dudette")) << std::endl;
    std::cout << "Strings: " << sformat(string("Dudonimous")) << std::endl;
    std::cout << "CString: " << sformat("Dude") << std::endl;
    std::cout << "TmpFrmt: " << sformat("Dude",10,CentreAlign) << std::endl;
    std::cout << "PrmFrmt: " << sformat(10,CentreAlign)("Dude") << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Float Formats" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Format<float> fformat;
    float f = -12235.135;
    fformat.base(Fixed);
    fformat.width(15);
    fformat.precision(3);
    fformat.align(LeftAlign);
    fformat.align(CentreAlign);
    fformat.align(RightAlign);
    fformat(2,15,RightAlign,Fixed);
    fformat(2,15);
    std::cout << "TmpFrmt: " << fformat(f,2,15,CentreAlign,Fixed) << std::endl;
    std::cout << "TmpFrmt: " << fformat(f,2,15) << std::endl;
    std::cout << "Format : " << fformat(f) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                   Double Formats" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    Format<double> dformat;
    double d = 134.2352;
    dformat.base(Fixed); // Format setters
    dformat.width(15);
    dformat.precision(3);
    dformat.align(LeftAlign);
    dformat.align(CentreAlign);
    dformat.align(RightAlign);
    dformat(2,15,RightAlign,Fixed); // Combo format operators
    dformat(4,15);
    std::cout << "TmpFrmt: " << dformat(d,2,15,CentreAlign,Fixed) << std::endl;
    std::cout << "TmpFrmt: " << dformat(d,3,15) << std::endl;
    std::cout << "Format : " << dformat(d) << std::endl;

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                      Passed" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    return 0;
}
