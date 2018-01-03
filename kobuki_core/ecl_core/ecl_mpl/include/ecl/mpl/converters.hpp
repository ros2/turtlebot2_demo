/**
 * @file /ecl_mpl/include/ecl/mpl/converters.hpp
 *
 * @brief Type converters.
 *
 * @date February, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_MPL_CONVERTERS_HPP_
#define ECL_MPL_CONVERTERS_HPP_

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Signed to Unsigned Template MetaConversions
*****************************************************************************/

/** @brief Primary template for signed to unsigned type metafunctions. **/
template <typename Signed> struct Unsigned {};

/** @brief Metafunction for char to unsigned char type conversions. **/
template <> struct Unsigned<char> { typedef unsigned char type; /**< The unsigned equivalent. **/ };
/** @brief Metafunction for short to unsigned short type conversions. **/
template <> struct Unsigned<short> { typedef unsigned short type; /**< The unsigned equivalent. **/};
/** @brief Metafunction for int to unsigned int type conversions. **/
template <> struct Unsigned<int> { typedef unsigned int type; /**< The unsigned equivalent. **/};
/** @brief Metafunction for long to unsigned long type conversions. **/
template <> struct Unsigned<long> { typedef unsigned long type; /**< The unsigned equivalent. **/};
/** @brief Metafunction for long long to unsigned long long type conversions. **/
template <> struct Unsigned<long long> { typedef unsigned long long type; /**< The unsigned equivalent. **/};

/** @brief Metafunction returning same type for unsigned chars. **/
template <> struct Unsigned<unsigned char> { typedef unsigned char type; /**< The unsigned equivalent. **/ };
/** @brief Metafunction returning same type for unsigned shorts. **/
template <> struct Unsigned<unsigned short> { typedef unsigned short type; /**< The unsigned equivalent. **/};
/** @brief Metafunction returning same type for unsigned ints. **/
template <> struct Unsigned<unsigned int> { typedef unsigned int type; /**< The unsigned equivalent. **/};
/** @brief Metafunction returning same type for unsigned longs. **/
template <> struct Unsigned<unsigned long> { typedef unsigned long type; /**< The unsigned equivalent. **/};
/** @brief Metafunction returning same type for unsigned long longs. **/
template <> struct Unsigned<unsigned long long> { typedef unsigned long long type; /**< The unsigned equivalent. **/};

} // namespace ecl

#endif /* ECL_MPL_CONVERTERS_HPP_ */
