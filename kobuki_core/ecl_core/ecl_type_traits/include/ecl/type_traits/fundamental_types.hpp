/**
 * @file /ecl_type_traits/include/ecl/type_traits/fundamental_types.hpp
 *
 * @brief Trait types for fundamental types.
 *
 * @date 29/07/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TYPE_TRAITS_FUNDAMENTAL_TYPES_HPP_
#define ECL_TYPE_TRAITS_FUNDAMENTAL_TYPES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>
#include <ecl/mpl/bool.hpp>
#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Integers
*****************************************************************************/
/**
 * @brief Default action for detection of a fundamental integral type (false).
 *
 * Sets the default value (false) for detection of a fundamental integral type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_integral : public False {};


/**
 * @brief Integral trait for char types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<char> : public True {};

/**
 * @brief Integral trait for unsigned char types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<unsigned char> : public True {};

/**
 * @brief Integral trait for short types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<short> : public True {};

/**
 * @brief Integral trait for unsigned short types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<unsigned short> : public True {};

/**
 * @brief Integral trait for int types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<int> : public True {};

/**
 * @brief Integral trait for unsigned int types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<unsigned int> : public True {};

/**
 * @brief Integral trait for long types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<long> : public True {};

/**
 * @brief Integral trait for unsigned long types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<unsigned long> : public True {};

/**
 * @brief Integral trait for long long types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<long long> : public True {};

/**
 * @brief Integral trait for unsigned long long types.
 *
 * Specialisation for the is_a_integral trait.
 */
template <>
class is_integral<unsigned long long> : public True {};

/*****************************************************************************
** Unsigned Integers
*****************************************************************************/
/**
 * @brief Default action for detection of an unsigned integral type (false).
 *
 * Sets the default value (false) for detection of an unsigned integral type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_unsigned : public False {};

/**
 * @brief Unsigned trait for unsigned char types.
 */
template <>
class is_unsigned<unsigned char> : public True {};

/**
 * @brief Unsigned trait for unsigned short types.
 */
template <>
class is_unsigned<unsigned short> : public True {};

/**
 * @brief Unsigned trait for unsigned int types.
 */
template <>
class is_unsigned<unsigned int> : public True {};

/**
 * @brief Unsigned trait for unsigned long types.
 */
template <>
class is_unsigned<unsigned long> : public True {};

/**
 * @brief Unsigned trait for unsigned long long types.
 */
template <>
class is_unsigned<unsigned long long> : public True {};

/*****************************************************************************
** Signed Integers
*****************************************************************************/
/**
 * @brief Default action for detection of a signed integral type (false).
 *
 * Sets the default value (false) for detection of signed integral type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_signed : public False {};

/**
 * @brief Unsigned trait for signed char types.
 */
template <>
class is_signed<char> : public True {};

/**
 * @brief Unsigned trait for signed short types.
 */
template <>
class is_signed<short> : public True {};

/**
 * @brief Unsigned trait for signed int types.
 */
template <>
class is_signed<int> : public True {};

/**
 * @brief Unsigned trait for signed long types.
 */
template <>
class is_signed<long> : public True {};

/**
 * @brief Unsigned trait for signed long long types.
 */
template <>
class is_signed<long long> : public True {};


/*****************************************************************************
** Floats
*****************************************************************************/

/**
 * @brief Default action for detection of a fundamental float type (false).
 *
 * Sets the default value (false) for detection of a fundamental float type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_float : public False {};

/**
 * @brief Float trait for float types.
 *
 * Specialisation for the is a float trait for float types.
 */
template <>
class is_float<float> : public True {};

/**
 * @brief Float trait for double types.
 *
 * Specialisation for the is a float trait for double types.
 */
template <>
class is_float<double> : public True {};

/**
 * @brief Float trait for long double types.
 *
 * Specialisation for the is a float trait for double types.
 */
template <>
class is_float<long double> : public True {};

/*****************************************************************************
** Bytes
*****************************************************************************/

/**
 * @brief Default action for detection of a fundamental signed byte type (false).
 *
 * Sets the default value (false) for detection of a fundamental signed byte type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_signed_byte : public False {};

/**
 * @brief Signed byte trait for signed char types.
 *
 * Specialisation for the is a signed byte trait for signed char types.
 */
template <>
class is_signed_byte<signed char> : public True {};

/**
 * @brief Signed byte trait for char types if typedef'd to signed char.
 *
 * Specialisation for the is a signed byte trait for char types. This
 * depends on whether the platform has typedef'd char to signed char
 * or not.
 */
template <>
class is_signed_byte<char> : public Bool<std::numeric_limits<char>::is_signed> {};

/**
 * @brief Default action for detection of a fundamental unsigned byte type (false).
 *
 * Sets the default value (false) for detection of a fundamental unsigned byte type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_unsigned_byte : public False {};

/**
 * @brief Unsigned byte trait for unsigned char types.
 *
 * Specialisation for the is a unsigned byte trait for unsigned char types.
 */
template <>
class is_unsigned_byte<unsigned char> : public True {};

/**
 * @brief Unsigned byte trait for char types if typedef'd to unsigned char.
 *
 * Specialisation for the is a unsigned byte trait for char types. This
 * depends on whether the platform has typedef'd char to unsigned char
 * or not.
 */
template <>
class is_unsigned_byte<char> : public Bool<!std::numeric_limits<char>::is_signed> {};

/**
 * @brief Default action for detection of a fundamental byte type (false).
 *
 * Sets the default value (false) for detection of a fundamental byte type.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_byte : public False {};

/**
 * @brief Byte trait for signed char types.
 *
 * Specialisation for the is a byte trait for signed char types.
 */
template <>
class is_byte<signed char> : public True {};

/**
 * @brief Byte trait for char types.
 *
 * Specialisation for the is a byte trait for char types.
 */
template <>
class is_byte<char> : public True {};

/**
 * @brief Byte trait for unsigned char types.
 *
 * Specialisation for the is a byte trait for unsigned char types.
 */
template <>
class is_byte<unsigned char> : public True {};

} // namespace ecl

#endif /* ECL_TYPE_TRAITS_FUNDAMENTAL_TYPES_HPP_ */
