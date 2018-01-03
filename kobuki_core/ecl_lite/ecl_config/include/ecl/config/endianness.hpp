/**
 * @file /ecl_config/include/ecl/config/endianness.hpp
 *
 * @brief Determines the endianness of the platform.
 *
 * @date July, 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_ENDIANNESS_HPP_
#define ECL_CONFIG_ENDIANNESS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "portable_types.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {
	/**
	 * @brief Determines if your platform is big endian (at compile time).
	 *
	 * This is a nice function that I found on stack overflow - the best
	 * thing about it, is that a good compiler will see that its a const
	 * function and thus store it in your program as a constant as well
	 * as excluding any code that fails this test (within the usual if/else
	 * condition check). Thus, effectively giving it the same power as
	 * a macro. Brilliant!
	 *
	 * @return bool : true if big endian, false otherwise.
	 */
	inline bool is_big_endian() {
	    union {
	        uint32 i;
	        char c[4];
	    } bint = {0x01020304};

	    return bint.c[0] == 1;
	}

} // namespace ecl

#endif /* ECL_CONFIG_ENDIANNESS_HPP_ */
