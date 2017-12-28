/**
 * @file /ecl_config/include/ecl/config/char_sign.hpp
 *
 * @brief Tests in runtime to see if char is signed or not.
 *
 * @date March, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_CHAR_SIGN_HPP_
#define ECL_CONFIG_CHAR_SIGN_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <climits>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/**
 * @brief Determines if your platform char type is signed or otherwise.
 *
 * This is a useful runtime check, if you need a compile time check, ecl's
 * cmake probling sets a macro in ecl/config/ecl.hpp.
 *
 * @return bool : true if signed, false otherwise.
 */
inline bool is_char_signed() {

    return ((CHAR_MIN == SCHAR_MIN) && (CHAR_MAX == SCHAR_MAX));
}


} // namespace ecl

#endif /* ECL_CONFIG_CHAR_SIGN_HPP_ */
