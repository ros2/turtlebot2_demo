/**
 * @file /ecl_containers/include/ecl/containers/converters.hpp
 *
 * @brief Some simple converters for ecl container types.
 *
 * @date August 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_ARRAY_CONVERTERS_HPP_
#define ECL_CONTAINERS_ARRAY_CONVERTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifdef ECL_MEM_CHECK_ARRAYS
  #include "array_mem_check.hpp"
  #include "array_dynamic_mem_check.hpp"
#else
  #include "array_no_mem_check.hpp"
  #include "array_dynamic_no_mem_check.hpp"
#endif
#include <ecl/converters/to_byte_array.hpp>
#include <ecl/converters/from_byte_array.hpp>
#include <ecl/converters/converter.hpp>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Byte Array Converters
*****************************************************************************/

/**
 * @brief Specialisation for integral to char Array conversions.
 *
 * Note this will only work for dynamic containers as we can't possibly know the size
 * of the container beforehand.
 */
template <typename Integral>
class ECL_PUBLIC Converter <Array<char>, Integral > : public converters::IntegralToByteArray< Array<char>, Integral > {};

/**
 * @brief Specialisation for integral to signed char Array conversions.
 *
 * Note this will only work for dynamic containers as we can't possibly know the size
 * of the container beforehand.
 */
template <typename Integral>
class ECL_PUBLIC Converter <Array<signed char>, Integral > : public converters::IntegralToByteArray< Array<signed char>, Integral > {};


/**
 * @brief Specialisation for integral to signed char Array conversions.
 *
 * Note this will only work for dynamic containers as we can't possibly know the size
 * of the container beforehand.
 */
template <typename Integral>
class ECL_PUBLIC Converter <Array<unsigned char>, Integral > : public converters::IntegralToByteArray< Array<unsigned char>, Integral > {};
/**
 * @brief Specialisation for char Array container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Array<char,Size> > : public converters::FromByteArray< Integral, Array<char,Size> > {};

/**
 * @brief Specialisation for the unsigned char Array container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Array<unsigned char,Size> > : public converters::FromByteArray< Integral, Array<unsigned char,Size> > {};
/**
 * @brief Specialisation for signed char Array container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Array<signed char,Size> > : public converters::FromByteArray< Integral, Array<signed char,Size> > {};

} // namespace ecl

#endif /* ECL_CONTAINERS_ARRAY_CONVERTERS_HPP_ */
