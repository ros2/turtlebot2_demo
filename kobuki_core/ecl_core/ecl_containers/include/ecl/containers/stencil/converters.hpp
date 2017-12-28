/**
 * @file /include/ecl/containers/stencil/converters.hpp
 *
 * @brief Some simple converters for ecl stencils.
 *
 * @date August 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_STENCIL_CONVERTERS_HPP_
#define ECL_CONTAINERS_STENCIL_CONVERTERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/converters/from_byte_array.hpp>
#include <ecl/converters/to_byte_array.hpp>
#include "stencil.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forware Declarations
*****************************************************************************/

template<typename Type, std::size_t Size> class Array;

/*****************************************************************************
** Interface [Array]
*****************************************************************************/

/**
 * @brief Specialisation for integral to char Stencil conversions.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter < Stencil< Array<char,Size> >, Integral > : public converters::IntegralToByteArray< Stencil< Array<char,Size> >, Integral > {};

/**
 * @brief Specialisation for integral to unsigned char Stencil conversions.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter < Stencil< Array<unsigned char,Size> >, Integral > : public converters::IntegralToByteArray< Stencil< Array<unsigned char,Size> >, Integral > {};

/**
 * @brief Specialisation for integral to signed char Stencil conversions.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter < Stencil< Array<signed char,Size> >, Integral > : public converters::IntegralToByteArray< Stencil< Array<signed char,Size> >, Integral > {};

/**
 * @brief Specialisation for char Stencil container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Stencil< Array<char,Size> > > : public converters::FromByteArray< Integral, Stencil< Array<char,Size> > > {};

/**
 * @brief Specialisation for unsigned char Stencil container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Stencil< Array<unsigned char,Size> > > : public converters::FromByteArray< Integral, Stencil< Array<unsigned char,Size> > > {};

/**
 * @brief Specialisation for signed char Stencil container based FromByteArray converter.
 */
template <typename Integral, std::size_t Size>
class ECL_PUBLIC Converter <Integral, Stencil< Array<signed char,Size> > > : public converters::FromByteArray< Integral, Stencil< Array<signed char,Size> > > {};

/**
 * @brief Specialisation for unsigned int to vector based unsigned char Stencil container.
 */
template <>
class ECL_PUBLIC Converter< Stencil< std::vector<unsigned char> >, unsigned int > : public converters::IntegralToByteArray< Stencil< std::vector<unsigned char> >, unsigned int > {};

/**
 * @brief Specialisation for int to vector based unsigned char Stencil container.
 */
template <>
class ECL_PUBLIC Converter< Stencil< std::vector<unsigned char> >, int > : public converters::IntegralToByteArray< Stencil< std::vector<unsigned char> >, int > {};

/**
 * @brief Specialisation for long to vector based unsigned char Stencil container.
 */
template <>
class ECL_PUBLIC Converter< Stencil< std::vector<unsigned char> >, long > : public converters::IntegralToByteArray< Stencil< std::vector<unsigned char> >, long > {};

/**
 * @brief Specialisation for long long to vector based unsigned char Stencil container.
 */
template <>
class ECL_PUBLIC Converter< Stencil< std::vector<unsigned char> >, long long > : public converters::IntegralToByteArray< Stencil< std::vector<unsigned char> >, long long > {};

/**
 * @brief Specialisation for const unsigned char* Stencil container based FromByteArray converter.
 */
template <typename Integral>
class ECL_PUBLIC Converter <Integral, Stencil< const unsigned char* > > : public converters::FromByteArray< Integral, Stencil< const unsigned char* > > {};


} // namespace ecl

#endif /* ECL_CONTAINERS_STENCIL_CONVERTERS_HPP_ */
