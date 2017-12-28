/**
 * @file /include/ecl/converters/from_byte_array.hpp
 *
 * @brief Converts byte arrays to integral types.
 *
 * @date August 2010
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONVERTERS_CONVERTERS_FROM_BYTE_ARRAY_HPP_
#define ECL_CONVERTERS_CONVERTERS_FROM_BYTE_ARRAY_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include <ecl/config/macros.hpp>
#include <ecl/concepts/containers.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/mpl/converters.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include <ecl/type_traits/numeric_limits.hpp>
#include "converter.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{
/**
 * @cond DO_NOT_DOXYGEN
 */
namespace converters
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/

/**
 * @brief Byte array to integral type converter.
 *
 * This class is not meant to be used directly, rather it is a
 * simple base class for specialisations. This is kept separate because it
 * the generic template type calls will clash with the default
 * converter template specification. Specialise this for
 * the various classes we actually want to use
 * (i.e. see Converter<Integral,vector<char>> etc.)
 */
template<typename Integral, typename ByteArray>
class ECL_PUBLIC FromByteArray : public ConverterBase
{
public:
  /**
   * Converts from a byte array container to an integral type.
   * The bytes are ordered from least significant to most
   * significant (little-endian).
   *
   * This function has compile time asserts to check that
   * the 'to' type is integral, also a compile time concept
   * check that the byte array is a byte container and also
   * handles the error case when the byte array size doesn't
   * exactly match the integral type size (throws an
   * exception in debug mode and flags error() with a
   * ConversionError.
   *
   * @param byte_array : the input byte array container.
   * @return int : the integer representation of the digit.
   * @exception StandardException: throws if byte array size doesn't match integral type size (in bytes) [debug mode only].
   **/
  Integral operator()(const ByteArray &byte_array) ecl_debug_throw_decl(ecl::StandardException)
  {
    /*********************
     ** Checks
     **********************/
    ecl_compile_time_concept_check(ByteContainerConcept<ByteArray>);
    ecl_compile_time_assert(is_integral<Integral>::value);
    if (byte_array.size() > ecl::numeric_limits<Integral>::bytes)
    {
      ecl_debug_throw(
          StandardException(LOC,ConversionError,"The byte array is too long for the integral type specified."));
      error_handler = ConversionError;
    }
    /*********************
     ** Method
     **********************/
    typename ecl::Unsigned<Integral>::type unsigned_value = 0;
    for (unsigned int i = 0; i < byte_array.size(); ++i)
    {
      unsigned_value |= static_cast<unsigned char>(byte_array[i]) << 8 * i;
    }
    if (ecl::is_signed<Integral>::value)
    {
      Integral value = static_cast<Integral>(unsigned_value);
      return value;
    }
    else
    {
      return unsigned_value;
    }
  }
  /**
   * @brief Required virtual destructor.
   */
  virtual ~FromByteArray() {}
};

} // namespace converters

/**
 * @endcond
 */

/**
 * @brief Specialisation for the vector<char> container based FromByteArray converter.
 */
template<typename Integral>
class ECL_PUBLIC Converter<Integral, std::vector<char> > :
    public converters::FromByteArray<Integral, std::vector<char> >
{
};
/**
 * @brief Specialisation for the vector<unsigned char> container based FromByteArray converter.
 */
template<typename Integral>
class ECL_PUBLIC Converter<Integral, std::vector<unsigned char> > : public converters::FromByteArray<Integral,
    std::vector<unsigned char> >
{
};
/**
 * @brief Specialisation for the vector<signed char> container based FromByteArray converter.
 */
template<typename Integral>
class ECL_PUBLIC Converter<Integral, std::vector<signed char> > : public converters::FromByteArray<Integral,
    std::vector<signed char> >
{
};

} // namespace ecl

#endif /* ECL_CONVERTERS_CONVERTERS_FROM_BYTE_ARRAY_HPP_ */
