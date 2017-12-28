/**
 * @file /ecl_type_traits/include/ecl/type_traits/numeric_limits.hpp
 *
 * @brief Expand on the std numeric limits classes.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_TYPE_TRAITS_NUMERIC_LIMITS_HPP_
#define ECL_TYPE_TRAITS_NUMERIC_LIMITS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>
#include <climits>
#include <ecl/config/portable_types.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Parent Template
*****************************************************************************/
/**
 * @brief Expands the std numeric_limits class.
 *
 * This (the non-specialized template) simply returns the same
 * functionality (via inheritance) as the std::numeric_limits class.
 *
 * - http://www.cplusplus.com/reference/std/limits/numeric_limits/
 *
 * Specialisations of this class expand its definitions.
 *
 * Note that since this class and all its specialisations inherit
 * the std::numeric_limits class, it
 * is only really usable for fundamental c++ types. To extend
 * non-fundamental types however, it is perfectly ok to specialise
 * and add your own numeric_limits functionality.
 *
 * Also note that once the future c++0x standard is in, support
 * in std::numeric_limits will be better and we can do away with much
 * of the implementation in the derived classes here.
 *
 * @tparam T : the type to check (currently must be a fundamental type).
 */
template <typename T>
class ecl_type_traits_PUBLIC numeric_limits : public std::numeric_limits<T> {
public:
	numeric_limits() {}
private:
};

/*****************************************************************************
** Specialisations
*****************************************************************************/
/**
 * @brief Expands the std numeric_limits class for char.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<char> : public std::numeric_limits<char> {
public:
	static const char one = 1;
	static const uint16 bits = ECL_SIZE_OF_CHAR*8;
	static const uint16 bytes = ECL_SIZE_OF_CHAR;
	static const char minimum = CHAR_MIN;
	static const char maximum = CHAR_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for unsigned char.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<unsigned char> : public std::numeric_limits<unsigned char> {
public:
	static const unsigned char one = 1U;
	static const uint16 bits = ECL_SIZE_OF_CHAR*8;
	static const uint16 bytes = ECL_SIZE_OF_CHAR;
	static const unsigned char minimum = 0U;
	static const unsigned char maximum = UCHAR_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for short.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<short> : public std::numeric_limits<short> {
public:
	static const short one = 1;
	static const uint16 bits = ECL_SIZE_OF_SHORT*8;
	static const uint16 bytes = ECL_SIZE_OF_SHORT;
	static const short minimum = SHRT_MIN;
	static const short maximum = SHRT_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for unsigned short.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<unsigned short> : public std::numeric_limits<unsigned short> {
public:
	static const unsigned short one = 1U;
	static const uint16 bits = ECL_SIZE_OF_SHORT*8;
	static const uint16 bytes = ECL_SIZE_OF_SHORT;
	static const unsigned short minimum = 0U;
	static const unsigned short maximum = USHRT_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for int.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<int> : public std::numeric_limits<int> {
public:
	static const int one = 1;
	static const uint16 bits = ECL_SIZE_OF_INT*8;
	static const uint16 bytes = ECL_SIZE_OF_INT;
	static const int minimum = INT_MIN;
	static const int maximum = INT_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for unsigned int.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<unsigned int> : public std::numeric_limits<unsigned int> {
public:
	static const unsigned int one = 1U;
	static const uint16 bits = ECL_SIZE_OF_INT*8;
	static const uint16 bytes = ECL_SIZE_OF_INT;
	static const unsigned int minimum = 0U;
	static const unsigned int maximum = UINT_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};


/**
 * @brief Expands the std numeric_limits class for long.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<long> : public std::numeric_limits<long> {
public:
	static const long one = 1L;
	static const uint16 bits = ECL_SIZE_OF_LONG*8;
	static const uint16 bytes = ECL_SIZE_OF_LONG;
	static const long minimum = LONG_MIN;
	static const long maximum = LONG_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for unsigned long.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<unsigned long> : public std::numeric_limits<unsigned long> {
public:
	static const unsigned long one = 1UL;
	static const uint16 bits = ECL_SIZE_OF_LONG*8;
	static const uint16 bytes = ECL_SIZE_OF_LONG;
	static const unsigned long minimum = 0UL;
	static const unsigned long maximum = ULONG_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for long long.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<long long> : public std::numeric_limits<long long> {
public:
	static const long long one = 1LL;
	static const uint16 bits = ECL_SIZE_OF_LONG_LONG*8;
	static const uint16 bytes = ECL_SIZE_OF_LONG_LONG;
	static const long long minimum = LLONG_MIN;
	static const long long maximum = LLONG_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for unsigned long long.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<unsigned long long> : public std::numeric_limits<unsigned long long> {
public:
	static const unsigned long long one = 1ULL;
	static const uint16 bits = ECL_SIZE_OF_LONG_LONG*8;
	static const uint16 bytes = ECL_SIZE_OF_LONG_LONG;
	static const unsigned long long minimum = 0ULL;
	static const unsigned long long maximum = ULLONG_MAX;

	typedef float Precision;
	static const Precision dummy_precision;
};

/**
 * @brief Expands the std numeric_limits class for float.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<float> : public std::numeric_limits<float> {
public:
	static const uint16 bits = ECL_SIZE_OF_FLOAT*8;
	static const uint16 bytes = ECL_SIZE_OF_FLOAT;

	typedef float Precision; /**< @brief Type used to specify precisions. **/
	static const float dummy_precision; /**< @brief Default precision. **/

	static const float minimum;
	static const float maximum;
};

/**
 * @brief Expands the std numeric_limits class for float.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<double> : public std::numeric_limits<double> {
public:
	static const uint16 bits = ECL_SIZE_OF_DOUBLE*8;
	static const uint16 bytes = ECL_SIZE_OF_DOUBLE;

	typedef double Precision; /**< @brief Type used to specify precisions. **/
	static const double dummy_precision; /**< @brief Default precision. **/

	static const double minimum;
	static const double maximum;
};

/**
 * @brief Expands the std numeric_limits class for float.
 *
 * @sa ecl::numeric_limits
 */
template <>
class ecl_type_traits_PUBLIC numeric_limits<long double> : public std::numeric_limits<long double> {
public:
	static const uint16 bits = ECL_SIZE_OF_LONG_DOUBLE*8;
	static const uint16 bytes = ECL_SIZE_OF_LONG_DOUBLE;

	typedef long double Precision; /**< @brief Type used to specify precisions. **/
	static const long double dummy_precision; /**< @brief Default precision. **/

	static const long double minimum;
	static const long double maximum;
};


} // namespace ecl

#endif /* ECL_TYPE_TRAITS_NUMERIC_LIMITS_HPP_ */
