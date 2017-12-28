/**
 * @file /include/ecl/converters/to_byte_array.hpp
 *
 * @brief Convert to byte arrays.
 *
 * @date March 2010
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONVERTERS_TO_BYTE_ARRAY_HPP_
#define ECL_CONVERTERS_TO_BYTE_ARRAY_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <vector>
#include <ecl/concepts/containers.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/exceptions/macros.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include <ecl/type_traits/numeric_limits.hpp>
#include "converter.hpp"
#include <iostream>
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
 * @brief Integral to byte array converter.
 *
 * This class is not meant to be used directly, rather it is a
 * simple base class for specialisations. This is kept separate because it
 * the generic template type calls will clash with the default
 * converter template specification. Specialise this for
 * the various classes we actually want to use
 * (i.e. see Converter<Integral,vector<char>> etc.)
 */
template<typename ByteArray, typename Integral>
class IntegralToByteArray : public ConverterBase
{
public:
  /**
   * @brief Convert from integral to byte array.
   *
   * This is a bit different to most converters as it takes in both arguments. The first argument is
   * needed to supply the necessary byte array storage. We don't want to waste time
   * copying the data out, and also we will sometimes want to overlay on part of an
   * underlying buffer (via ecl_container stencils).
   *
   * The bytes are ordered from least significant to most
   * significant (little-endian).
   *
   * Note this throws an exception and/or sets the converter error flag if the byte array
   * storage does not match the requirements of the integral type to be converted.
   *
   * @param byte_array : the underlying array storage, it must match the integral type size.
   * @param input : the integral to be converted.
   * @return ByteArray& : convenience handle to the byte array.
   * @exception : throws if the byte array storage size does not match the integral byte size [debug mode only].
   */
  const ByteArray& operator()(ByteArray &byte_array, const Integral &input) ecl_debug_throw_decl(ecl::StandardException)
  {
    /*********************
     ** Checks
     **********************/
    ecl_compile_time_concept_check(ecl::ContainerConcept<ByteArray>);
    ecl_compile_time_concept_check(ecl::ByteContainerConcept<ByteArray>);
    ecl_compile_time_assert(is_integral<Integral>::value);
    /*********************
     ** Configuration
     **********************/
    if (byte_array.size() != ecl::numeric_limits<Integral>::bytes)
    {
      std::cout << "Failed size check" << std::endl;
      ecl_debug_throw(
          StandardException(LOC,ConversionError,"The specified byte array size does not match the byte size of the integral type."));
      error_handler = ConversionError;
      return byte_array;
    }

    /*********************
     ** Method
     **********************/
    for (unsigned int i = 0; i < ecl::numeric_limits<Integral>::bytes; ++i)
    {
      byte_array[i] = static_cast<typename ByteArray::value_type>(input >> 8 * i);
//            Einstein was bit 'AND'ing a 0xff for dslam - is this useful?
//            byte_array[i] = static_cast<typename ByteArray::value_type>( input >> 8*i & 0xff);
    }
    return byte_array;
  }
  /**
   * @brief Required virtual destructor.
   */
  virtual ~IntegralToByteArray() {}
};

} // namespace converters
/**
 * @endcond
 */

/*****************************************************************************
 ** Integral to Byte Array Converters
 *****************************************************************************/
/**
 * @brief Converts int to a byte array of chars.
 **/
template<>
class Converter<std::vector<char>, int> : public converters::IntegralToByteArray<std::vector<char>, int>
{
};

/**
 * @brief Converts int to a byte array of unsigned chars.
 **/
template<>
class Converter<std::vector<unsigned char>, int> : public converters::IntegralToByteArray<std::vector<unsigned char>,
    int>
{
};

/**
 * @brief Converts int to a byte array of signed chars.
 **/
template<>
class Converter<std::vector<signed char>, int> : public converters::IntegralToByteArray<std::vector<signed char>, int>
{
};

/**
 * @brief Converts unsigned int to a byte array of chars.
 **/
template<>
class Converter<std::vector<char>, unsigned int> : public converters::IntegralToByteArray<std::vector<char>,
    unsigned int>
{
};

/**
 * @brief Converts unsigned int to a byte array of unsigned chars.
 **/
template<>
class Converter<std::vector<unsigned char>, unsigned int> : public converters::IntegralToByteArray<
    std::vector<unsigned char>, unsigned int>
{
};

/**
 * @brief Converts unsigned int to a byte array of signed chars.
 **/
template<>
class Converter<std::vector<signed char>, unsigned int> : public converters::IntegralToByteArray<
    std::vector<signed char>, unsigned int>
{
};

/**
 * @brief Converts long to a byte array of chars.
 **/
template<>
class Converter<std::vector<char>, long> : public converters::IntegralToByteArray<std::vector<char>, long>
{
};

/**
 * @brief Converts long to a byte array of unsigned chars.
 **/
template<>
class Converter<std::vector<unsigned char>, long> : public converters::IntegralToByteArray<std::vector<unsigned char>,
    long>
{
};

/**
 * @brief Converts long to a byte array of signed chars.
 **/
template<>
class Converter<std::vector<signed char>, long> : public converters::IntegralToByteArray<std::vector<signed char>, long>
{
};

/**
 * @brief Converts unsigned long to a byte array of chars.
 **/
template<>
class Converter<std::vector<char>, unsigned long> : public converters::IntegralToByteArray<std::vector<char>,
    unsigned long>
{
};

/**
 * @brief Converts unsigned long to a byte array of unsigned chars.
 **/
template<>
class Converter<std::vector<unsigned char>, unsigned long> : public converters::IntegralToByteArray<
    std::vector<unsigned char>, unsigned long>
{
};

/**
 * @brief Converts unsigned long to a byte array of signed chars.
 **/
template<>
class Converter<std::vector<signed char>, unsigned long> : public converters::IntegralToByteArray<
    std::vector<signed char>, unsigned long>
{
};

/**
 * @brief Converts long long to a byte array of chars.
 **/
template<>
class Converter<std::vector<char>, long long> : public converters::IntegralToByteArray<std::vector<char>, long long>
{
};

/**
 * @brief Converts long long to a byte array of unsigned chars.
 **/
template<>
class Converter<std::vector<unsigned char>, long long> : public converters::IntegralToByteArray<std::vector<unsigned char>,
long long>
{
};

/**
 * @brief Converts long long to a byte array of signed chars.
 **/
template<>
class Converter<std::vector<signed char>, long long> : public converters::IntegralToByteArray<std::vector<signed char>, long long>
{
};

/*****************************************************************************
 ** Byte Converter Interface
 *****************************************************************************/
/**
 * @brief Converts a char string of human readable hexes to a byte array.
 *
 * Parses a string of hexes into a character byte array.
 *
 * @sa Converter
 **/
template<>
class Converter<std::vector<char>, char*> : public converters::ConverterBase
{
public:
  /**
   * Converts a string of human readable hexes into a byte array.
   *
   * The string should contain bytes written as a '0x' followed
   * by two hex digits. e.g.
   *
   * @code
   * 0x32 0x64 0x64  0x54
   * @endcode
   *
   * This throws an exception and/or configures the error()
   * function if an unexpected char is discovered.
   *
   * @param input : the character string to be converted.
   * @return vector<char> : the byte array.
   * @exception StandardException : throws if unexpected char is discovered (debug mode only).
   **/
  std::vector<char> operator ()(const char* input) ecl_debug_throw_decl(StandardException)
  {

    std::vector<char> bytes;
    state = waiting_zero;

    while (*input != '\0')
    {
      switch (state)
      {
        case (waiting_zero):
        {
          if (*input == '0')
          {
            state = waiting_x;
          }
          else if (*input == ' ')
          {
            // ignore whitespace
          }
          else
          {
            ecl_debug_throw(StandardException(LOC,ConversionError));
            error_handler = ConversionError;
            bytes.clear();
            return bytes;
          }
          break;
        }
        case (waiting_x):
        {
          if (*input == 'x')
          {
            state = waiting_first_digit;
          }
          else
          {
            ecl_debug_throw(StandardException(LOC,ConversionError));
            error_handler = ConversionError;
            bytes.clear();
            return bytes;
          }
          break;
        }
        case (waiting_first_digit):
        {
          if ((*input >= '0') && (*input <= '9'))
          {
            byte = (*input - '0') << 4;
            state = waiting_second_digit;
          }
          else if ((*input >= 'a') && (*input <= 'f'))
          {
            byte = ((*input - 'a') + 0x0A) << 4;
            state = waiting_second_digit;
          }
          else if ((*input >= 'A') && (*input <= 'F'))
          {
            byte = ((*input - 'A') + 0x0A) << 4;
            state = waiting_second_digit;
          }
          else
          {
            ecl_debug_throw(StandardException(LOC,ConversionError));
            error_handler = ConversionError;
            bytes.clear();
            return bytes;
          }
          break;
        }
        case (waiting_second_digit):
        {
          if ((*input >= '0') && (*input <= '9'))
          {
            byte += *input - '0';
            bytes.push_back(byte);
          }
          else if ((*input >= 'a') && (*input <= 'f'))
          {
            byte += (*input - 'a') + 0x0A;
            bytes.push_back(byte);
          }
          else if ((*input >= 'A') && (*input <= 'F'))
          {
            byte += (*input - 'A') + 0x0A;
            bytes.push_back(byte);
          }
          else
          {
            ecl_debug_throw(StandardException(LOC,ConversionError));
            error_handler = ConversionError;
            bytes.clear();
            return bytes;
          }
          state = waiting_zero;
          break;
        }
        default:
        {
          state = waiting_zero;
          break;
        }
      }
      ++input;
    }
    return bytes;
  }
  ;
  virtual ~Converter()
  {
  }

private:
  unsigned int state;
  char byte;
  static const unsigned int waiting_zero = 0;
  static const unsigned int waiting_x = 1;
  static const unsigned int waiting_first_digit = 2;
  static const unsigned int waiting_second_digit = 3;
};

/*****************************************************************************
 * Byte Array Converter Family
 ****************************************************************************/
/**
 * @brief Family of converters to byte arrays of vector<char> type.
 **/
template<>
class Converter<std::vector<char>, void> : public Converter<std::vector<char>, char*>, public Converter<
                                               std::vector<char>, int>,
                                           public Converter<std::vector<char>, unsigned int>, public Converter<
                                               std::vector<char>, long>,
                                           public Converter<std::vector<char>, unsigned long>
{
public:
  virtual ~Converter()
  {
  }

  using Converter<std::vector<char>, char*>::operator();
  using Converter<std::vector<char>, int>::operator();
  using Converter<std::vector<char>, unsigned int>::operator();
  using Converter<std::vector<char>, long>::operator();
  using Converter<std::vector<char>, unsigned long>::operator();
};

/**
 * @brief Family of converters to byte arrays of vector<unsigned char> type.
 **/
template<>
class Converter<std::vector<unsigned char>, void> : public Converter<std::vector<unsigned char>, int>,
                                                    public Converter<std::vector<unsigned char>, unsigned int>,
                                                    public Converter<std::vector<unsigned char>, long>,
                                                    public Converter<std::vector<unsigned char>, unsigned long>
{
public:
  virtual ~Converter()
  {
  }

  using Converter<std::vector<unsigned char>, int>::operator();
  using Converter<std::vector<unsigned char>, unsigned int>::operator();
  using Converter<std::vector<unsigned char>, long>::operator();
  using Converter<std::vector<unsigned char>, unsigned long>::operator();
};

/**
 * @brief Family of converters to byte arrays of vector<signed char> type.
 **/
template<>
class Converter<std::vector<signed char>, void> : public Converter<std::vector<signed char>, int>,
                                                  public Converter<std::vector<signed char>, unsigned int>,
                                                  public Converter<std::vector<signed char>, long>,
                                                  public Converter<std::vector<signed char>, unsigned long>
{
public:
  virtual ~Converter()
  {
  }

  using Converter<std::vector<signed char>, int>::operator();
  using Converter<std::vector<signed char>, unsigned int>::operator();
  using Converter<std::vector<signed char>, long>::operator();
  using Converter<std::vector<signed char>, unsigned long>::operator();
};

} // namespace ecl

#endif /* ECL_CONVERTERS_TO_BYTE_ARRAY_HPP_ */
