/**
 * @file src/utilities/detect_endian.cpp
 *
 * @brief Simple utility used to detect endianness.
 *
 * @date Jul 10, 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../../include/ecl/config/endianness.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	if ( ecl::is_big_endian() ) {
		std::cout << "Platform is big endian." << std::endl;
	} else {
		std::cout << "Platform is little endian." << std::endl;
	}
	return 0;
}

