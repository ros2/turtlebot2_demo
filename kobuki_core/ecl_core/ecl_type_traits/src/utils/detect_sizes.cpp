/**
 * @file /src/utils/detect_sizes.cpp
 *
 * @brief Sends a stream of data regarding math type sizes to stdout.
 *
 * @date April 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <iomanip>
#include <iostream>
#include <ecl/config/portable_types.hpp>
#include <ecl/type_traits/numeric_limits.hpp>

/*****************************************************************************
** using
*****************************************************************************/

using ecl::int8;
using ecl::uint8;
using ecl::int16;
using ecl::uint16;
using ecl::int32;
using ecl::uint32;
using ecl::int64;
using ecl::uint64;
using ecl::numeric_limits;

/*****************************************************************************
** Main
*****************************************************************************/

int main() {

    std::cout << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "                       numeric_limits<>" << std::endl;
    std::cout << "***********************************************************" << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<char>::bytes " << numeric_limits<char>::bytes << std::endl;
    std::cout << "numeric_limits<char>::bits " << numeric_limits<char>::bits << std::endl;
    std::cout << "numeric_limits<char>::min " << static_cast<int>(numeric_limits<char>::minimum) << std::endl;
    std::cout << "numeric_limits<char>::max " << static_cast<int>(numeric_limits<char>::maximum) << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<unsigned char>::bytes " << numeric_limits<unsigned char>::bytes << std::endl;
    std::cout << "numeric_limits<unsigned char>::bits " << numeric_limits<unsigned char>::bits << std::endl;
    std::cout << "numeric_limits<unsigned char>::min " << static_cast<int>(numeric_limits<unsigned char>::minimum) << std::endl;
    std::cout << "numeric_limits<unsigned char>::max " << static_cast<int>(numeric_limits<unsigned char>::maximum) << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<short>::bytes " << numeric_limits<short>::bytes << std::endl;
    std::cout << "numeric_limits<short>::bits " << numeric_limits<short>::bits << std::endl;
    std::cout << "numeric_limits<short>::min " << numeric_limits<short>::minimum << std::endl;
    std::cout << "numeric_limits<short>::max " << numeric_limits<short>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<unsigned short>::bytes " << numeric_limits<unsigned short>::bytes << std::endl;
    std::cout << "numeric_limits<unsigned short>::bits " << numeric_limits<unsigned short>::bits << std::endl;
    std::cout << "numeric_limits<unsigned short>::min " << numeric_limits<unsigned short>::minimum << std::endl;
    std::cout << "numeric_limits<unsigned short>::max " << numeric_limits<unsigned short>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<int>::bytes " << numeric_limits<int>::bytes << std::endl;
    std::cout << "numeric_limits<int>::bits " << numeric_limits<int>::bits << std::endl;
    std::cout << "numeric_limits<int>::min " << numeric_limits<int>::minimum << std::endl;
    std::cout << "numeric_limits<int>::max " << numeric_limits<int>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<unsigned int>::bytes " << numeric_limits<unsigned int>::bytes << std::endl;
    std::cout << "numeric_limits<unsigned int>::bits " << numeric_limits<unsigned int>::bits << std::endl;
    std::cout << "numeric_limits<unsigned int>::min " << numeric_limits<unsigned int>::minimum << std::endl;
    std::cout << "numeric_limits<unsigned int>::max " << numeric_limits<unsigned int>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<long>::bytes " << numeric_limits<long>::bytes << std::endl;
    std::cout << "numeric_limits<long>::bits " << numeric_limits<long>::bits << std::endl;
    std::cout << "numeric_limits<long>::min " << numeric_limits<long>::minimum << std::endl;
    std::cout << "numeric_limits<long>::max " << numeric_limits<long>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<unsigned long>::bytes " << numeric_limits<unsigned long>::bytes << std::endl;
    std::cout << "numeric_limits<unsigned long>::bits " << numeric_limits<unsigned long>::bits << std::endl;
    std::cout << "numeric_limits<unsigned long>::min " << numeric_limits<unsigned long>::minimum << std::endl;
    std::cout << "numeric_limits<unsigned long>::max " << numeric_limits<unsigned long>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<long long>::bytes " << numeric_limits<long long>::bytes << std::endl;
    std::cout << "numeric_limits<long long>::bits " << numeric_limits<long long>::bits << std::endl;
    std::cout << "numeric_limits<long long>::min " << numeric_limits<long long>::minimum << std::endl;
    std::cout << "numeric_limits<long long>::max " << numeric_limits<long long>::maximum << std::endl;
    std::cout << std::endl;

    std::cout << "numeric_limits<unsigned long long>::bytes " << numeric_limits<unsigned long long>::bytes << std::endl;
    std::cout << "numeric_limits<unsigned long long>::bits " << numeric_limits<unsigned long long>::bits << std::endl;
    std::cout << "numeric_limits<unsigned long long>::min " << numeric_limits<unsigned long long>::minimum << std::endl;
    std::cout << "numeric_limits<unsigned long long>::max " << numeric_limits<unsigned long long>::maximum << std::endl;

//    std::cout << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << "                       Integer<>" << std::endl;
//    std::cout << "***********************************************************" << std::endl;
//    std::cout << std::endl;
//
//    Integer<8>::type int_8 = 0x03;
//    Integer<8>::utype uint_8 = 0x03;
//    std::cout << "Integer<8>::type:  " << int_8 << std::endl;
//    std::cout << "Integer<8>::utype: " << uint_8 << std::endl;
//    std::cout << "Integer<8>::min "  << static_cast<int>(Integer<8>::min) << std::endl;
//    std::cout << "Integer<8>::one "  << static_cast<int>(Integer<8>::one) << std::endl;
//    std::cout << "Integer<8>::max "  << static_cast<int>(Integer<8>::max) << std::endl;
//    std::cout << "Integer<8>::umin " << static_cast<int>(Integer<8>::umin) << std::endl;
//    std::cout << "Integer<8>::uone " << static_cast<int>(Integer<8>::uone) << std::endl;
//    std::cout << "Integer<8>::umax " << static_cast<int>(Integer<8>::umax) << std::endl;
//    std::cout << std::endl;
//
//    Integer<16>::type int_16 = 0x03;
//    Integer<16>::utype uint_16 = 0x03;
//    std::cout << "Integer<16>::type:  " << int_16 << std::endl;
//    std::cout << "Integer<16>::utype: " << uint_16 << std::endl;
//    std::cout << "Integer<16>::min " << Integer<16>::min << std::endl;
//    std::cout << "Integer<16>::one " << Integer<16>::one << std::endl;
//    std::cout << "Integer<16>::max " << Integer<16>::max << std::endl; // This doesn't output the correct value.
//    std::cout << "Integer<16>::umin " << Integer<16>::umin << std::endl;
//    std::cout << "Integer<16>::uone " << Integer<16>::uone << std::endl;
//    std::cout << "Integer<16>::umax " << Integer<16>::umax << std::endl;
//    std::cout << std::endl;
//
//    Integer<32>::type int_32 = 0x03;
//    Integer<32>::utype uint_32 = 0x03;
//    std::cout << "Integer<32>::type:  " << int_32 << std::endl;
//    std::cout << "Integer<32>::utype: " << uint_32 << std::endl;
//    std::cout << "Integer<32>::min " << Integer<32>::min << std::endl;
//    std::cout << "Integer<32>::one " << Integer<32>::one << std::endl;
//    std::cout << "Integer<32>::max " << Integer<32>::max << std::endl; // This doesn't output the correct value.
//    std::cout << "Integer<32>::umin " << Integer<32>::umin << std::endl;
//    std::cout << "Integer<32>::uone " << Integer<32>::uone << std::endl;
//    std::cout << "Integer<32>::umax " << Integer<32>::umax << std::endl;
//    std::cout << std::endl;
//
//    Integer<64>::type int_64 = 0x03;
//    Integer<64>::utype uint_64 = 0x03;
//    std::cout << "Integer<64>::type:  " << int_64 << std::endl;
//    std::cout << "Integer<64>::utype: " << uint_64 << std::endl;
//    std::cout << "Integer<64>::min " << Integer<64>::min << std::endl;
//    std::cout << "Integer<64>::one " << Integer<64>::one << std::endl;
//    std::cout << "Integer<64>::max " << Integer<64>::max << std::endl; // This doesn't output the correct value.
//    std::cout << "Integer<64>::umin " << Integer<64>::umin << std::endl;
//    std::cout << "Integer<64>::uone " << Integer<64>::uone << std::endl;
//    std::cout << "Integer<64>::umax " << Integer<64>::umax << std::endl;
//    std::cout << std::endl;

    return 0;
}
