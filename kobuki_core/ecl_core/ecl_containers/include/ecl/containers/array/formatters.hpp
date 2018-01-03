/**
 * @file /ecl_containers/include/ecl/containers/array/formatters.hpp
 *
 * @brief Text formatters for streaming array classes.
 *
 * @date September 2010
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONTAINERS_ARRAY_FORMATTERS_HPP_
#define ECL_CONTAINERS_ARRAY_FORMATTERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/formatters/common.hpp>
#include <ecl/formatters/number.hpp>
#include <ecl/formatters/floats.hpp>
#include "../common/formatters.hpp"
#include "../array.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{
namespace formatters
{

/*****************************************************************************
 ** Using
 *****************************************************************************/

/**
 * @brief Pseudo formatter for integral type arrays.
 *
 * These do nothing but pass the array back for streaming. Do not use this
 * class directly, rather call it via either the Array or Format classes
 * explicitly. This makes the code more readable. e.g.
 *
 * @code
 * // fixed array
 * Array<int, 5>::Formatter format_array_1;
 * Format< Array<int,4> > format_array_2;    // same thing
 *
 * // dynamic array
 * Array<int>::Formatter float_array_dynamic_formatter_1;
 * Format< Array<int> > format_array_dynamic_formatter_2;
 * @endcode
 *
 * @tparam Type : the value type of the fixed size Array.
 * @tparam N : the size of the container to be formatted.
 *
 * @todo Specialise the formatting for these integral type arrays so that they return
 * formatting possibilities equivalent to their base type.
 */
template<typename Type, size_t N>
class ECL_PUBLIC ArrayFormatter
{
public:
  virtual ~ArrayFormatter()
  {}
  /**
   * Pseudo formatter method, simply returns the underlying array which
   * has its own stream operator.
   * @param array : the array to be formatted.
   * @return Array<Type,N> : just returns the input array.
   */
  ecl::Array<Type,N>& operator()(ecl::Array<Type,N> &array) {
    return array;
  }
};

/*****************************************************************************
 ** Interface [ByteArrayFormatter]
 *****************************************************************************/

/**
 * @brief Parent template for the byte array formatters.
 *
 * We dont accept arguments for this class, it is simply
 * designed to present byte arrays in hex format byte by byte. If you
 * want specialised formatting for the character array, iterate over it
 * with the Format<char> (or signed/unsigned char) class.
 *
 * It will also accept stencils which have an underlying array of the
 * required type.
 *
 * Do not use this class directly, rather call it via either the
 * Array or Format classes explicitly. This makes the code more readable. e.g.
 *
 * @code
 * Array<char>::Formatter format_array_1;
 * Format< Array<char,4> > format_array_2;    // same thing
 * @endcode
 *
 * @tparam Byte : byte type, signed char/char/unsigned char.
 * @tparam N : the size of the container to be formatted.
 */
template<typename Byte, size_t N>
class ECL_PUBLIC ByteArrayFormatter
{
public:
  /**
   * Default constructor that simply sets up the formatter for use. There
   * is only one formatting style for this - a byte by byte view of the
   * array in hex format, so we don't accept any arguments. If you want
   * specialised formatting for your array, iterate it element by element
   * with the Format<signed char> class.
   **/
  ByteArrayFormatter() : ready_to_format(false)
  {};
  /**
   * The format operator. Use this when inserting into a stream. It
   * returns a template on which the stream will perform the formatting
   * to create the requested output.
   * @param array : the array to format.
   * @return Format<Array<signed char,N>>& : this formatter readied for use by a stream.
   */
  ByteArrayFormatter<Byte,N>& operator()(const Array<Byte,N> &array)
  {
    begin_iterator = array.begin();
    end_iterator = array.end();
    ready_to_format = true;
    return *this;
  }
  /**
   * The format operator. Use this when inserting into a stream. It
   * returns a template on which the stream will perform the formatting
   * to create the requested output.
   * @param stencil : a stencil on an underlying array of the requisite type.
   * @return Format< Array<signed char,N>>& : this formatter readied for use by a stream.
   */
  ByteArrayFormatter<Byte,N>& operator()(const Stencil< Array<Byte,N> > &stencil)
  {
    begin_iterator = stencil.begin();
    end_iterator = stencil.end();
    ready_to_format = true;
    return *this;
  }
  /**
   * @brief Formats on a window of the array.
   *
   * Format over the window specified.
   *
   * @param begin_iter : point in the array to begin formatting from.
   * @param end_iter : point in the array to begin formatting to.
   * @return Format<Array<signed char,N>>& : this formatter readied for use by a stream.
   *
   * @exception StandardException : throws if indices are out of range [debug mode only].
   */
  ByteArrayFormatter<Byte,N>& operator()(typename Array<Byte,N>::const_iterator begin_iter,
      typename Array<Byte,N>::const_iterator end_iter
  ) ecl_assert_throw_decl(StandardException)
  {

    begin_iterator = begin_iter;
    end_iterator = end_iter;
    ready_to_format = true;
    return *this;
  }

  virtual ~ByteArrayFormatter()
  {}
  /**
   * Insertion operator for sending the formatter (for character arrays) to an output stream.
   * @tparam OutputStream : the type of the output stream to be inserted into.
   * @tparam M : the size of the container to be formatted.
   * @param ostream : the output stream.
   * @param formatter : the formatter to be inserted.
   * @return OutputStream : continue streaming with the updated output stream.
   *
   * @exception StandardException : throws if the formatter is used multiply in one stream operation [debug mode only].
   */
  template <typename OutputStream, typename CharType, size_t M>
  friend OutputStream& operator << (OutputStream& ostream, const ByteArrayFormatter<CharType,M> &formatter) ecl_assert_throw_decl(StandardException);

private:
  typename Array<Byte,N>::const_iterator begin_iterator;
  typename Array<Byte,N>::const_iterator end_iterator;
  bool ready_to_format;
};

/*****************************************************************************
 ** Implementation [ByteArrayFormatter]
 *****************************************************************************/

template <typename OutputStream, typename CharType, size_t M>
OutputStream& operator <<(OutputStream& ostream, const ByteArrayFormatter<CharType, M> &formatter)
ecl_assert_throw_decl(StandardException)
{
  ecl_assert_throw(formatter.ready_to_format, StandardException(LOC,UsageError,"The formatter cannot print any data - "
          "either there is no data available, or you have tried to use the "
          "formatter more than once in a single streaming operation. "
          "C++ produces unspecified results when functors are used multiply "
          "in the same stream sequence, so this is not permitted here.") );

  ecl::Format<CharType> format(-1, ecl::NoAlign, ecl::Hex);
  typename ecl::Array<CharType, M>::const_iterator iter;
ostream  << "[ ";
  for ( iter = formatter.begin_iterator; iter != formatter.end_iterator; ++iter )
  {
    ostream << format(*iter) << " ";
  }
  ostream << "]";
  ostream.flush();
  return ostream;
}

/*****************************************************************************
 ** Specialisations [ArrayFormatter][Char Types]
 *****************************************************************************/
/**
 * @brief Convenience formatter for viewing a signed char array in hex format.
 *
 * We dont accept a wide range of arguments for this class, it is simply
 * designed to present byte arrays in hex format byte by byte. If you
 * want specialised formatting for the byte array, iterate over it
 * with the Format<char> (or signed/unsigned char) class.
 *
 * It will also accept stencils which have an underlying array of the
 * required type.
 *
 * @tparam N : the size of the container to be formatted.
 */
template<size_t N>
class ECL_PUBLIC ArrayFormatter< signed char,N > : public ByteArrayFormatter<signed char, N>
{};

/**
 * @brief Convenience formatter for viewing a char array in hex format.
 *
 * We dont accept a wide range of arguments for this class, it is simply
 * designed to present byte arrays in hex format byte by byte. If you
 * want specialised formatting for the byte array, iterate over it
 * with the Format<char> (or signed/unsigned char) class.
 *
 * It will also accept stencils which have an underlying array of the
 * required type.
 *
 * @tparam N : the size of the container to be formatted.
 */
template<size_t N>
class ECL_PUBLIC ArrayFormatter< char,N > : public ByteArrayFormatter<char, N>
{};

/**
 * @brief Convenience formatter for viewing an unsigned char array in hex format.
 *
 * We dont accept a wide range of arguments for this class, it is simply
 * designed to present byte arrays in hex format byte by byte. If you
 * want specialised formatting for the byte array, iterate over it
 * with the Format<char> (or signed/unsigned char) class.
 *
 * It will also accept stencils which have an underlying array of the
 * required type.
 *
 * @tparam N : the size of the container to be formatted.
 */
template<size_t N>
class ECL_PUBLIC ArrayFormatter< unsigned char,N > : public ByteArrayFormatter<unsigned char, N>
{
};

/**
 * @brief Formatter for fixed size float arrays.
 *
 * Allows precision settings for formatting of fixed size float arrays.
 *
 * @tparam N : the size of the fixed size float array to be formatted.
 *
 * @sa @ref ecl::formatters::FloatContainerFormatter "FloatContainerFormatter".
 */
template<size_t N>
class ECL_PUBLIC ArrayFormatter< float,N > : public FloatContainerFormatter< Array<float,N> >
{
public:
  /**
   * @brief Default constructor.
   *
   * Optionally accepts a precision setting for the contents of the array.
   *
   * @param p : the number of decimal places of precision [default : 2].
   * @param w : width [default : no width constraint]
   */
  ArrayFormatter(const int p=2, const int w=-1) : FloatContainerFormatter< Array<float,N> >(p,w)
  {};
  virtual ~ArrayFormatter()
  {}
};

/**
 * @brief Formatter for fixed size double arrays.
 *
 * Allows precision settings for formatting of fixed size double arrays.
 *
 * @tparam N : the size of the fixed size double array to be formatted.
 *
 * @sa @ref ecl::formatters::FloatContainerFormatter "FloatContainerFormatter".
 */
template<size_t N>
class ECL_PUBLIC ArrayFormatter< double,N > : public FloatContainerFormatter< Array<double,N> >
{
public:
  /**
   * @brief Default constructor.
   *
   * Optionally accepts a precision setting for the contents of the array.
   *
   * @param p : the number of decimal places of precision [default : 2].
   * @param w : width [default : no width constraint]
   */
  ArrayFormatter(const int p=2, const int w=-1) : FloatContainerFormatter< Array<double,N> >(p,w)
  {};
  virtual ~ArrayFormatter()
  {}
};

} // namespace formatters
} // namespace ecl

#endif /* ECL_CONTAINERS_ARRAY_FORMATTERS_HPP_ */
