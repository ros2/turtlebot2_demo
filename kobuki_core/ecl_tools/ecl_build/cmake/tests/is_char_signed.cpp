/**
 * @file /ecl_build/cmake/tests/is_char_signed.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Mar 12, 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <climits>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	char c = -1;
	if ( (CHAR_MIN == SCHAR_MIN) && (CHAR_MAX == SCHAR_MAX) ) {
		std::cout << "signed";
	} else {
		std::cout << "unsigned";
	}
	return 0;
}


