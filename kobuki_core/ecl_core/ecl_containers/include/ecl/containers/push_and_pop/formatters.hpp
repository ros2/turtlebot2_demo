/**
 * @file /ecl_containers/include/ecl/containers/push_and_pop/formatters.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date Jun 4, 2012
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_PUSH_AND_POP_FORMATTERS_HPP_
#define ECL_CONTAINERS_PUSH_AND_POP_FORMATTERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/formatters/common.hpp>
#include <ecl/formatters/number.hpp>
#include <ecl/formatters/floats.hpp>
#include "../common/formatters.hpp"
#include "../push_and_pop.hpp"

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
 * PushAndPop<int, 5>::Formatter format_1;
 * Format< PushAndPop<int,4> > format_2;    // same thing
 *
 * // dynamic array
 * PushAndPop<int>::Formatter float_dynamic_formatter_1;
 * Format< PushAndPop<int> > format_dynamic_formatter_2;
 * @endcode
 *
 * @tparam Type : the value type of the fixed size Array.
 * @tparam N : the size of the container to be formatted.
 *
 * @todo Specialise the formatting for these integral type arrays so that they return
 * formatting possibilities equivalent to their base type.
 */
template<typename Type, size_t N>
class ECL_PUBLIC PushAndPopFormatter
{
public:
  virtual ~PushAndPopFormatter()
  {}
  /**
   * Pseudo formatter method, simply returns the underlying array which
   * has its own stream operator.
   * @param array : the array to be formatted.
   * @return PushAndPop<Type,N> : just returns the input array.
   */
  ecl::PushAndPop<Type,N>& operator()(const ecl::PushAndPop<Type,N> &container) {
    return container;
  }
};

/*****************************************************************************
 ** Interface [ByteArrayFormatter]
 *****************************************************************************/

/**
 * @brief Parent template for the byte push and pop formatters.
 *
 * We dont accept arguments for this class, it is simply
 * designed to present byte arrays in hex format byte by byte. If you
 * want specialised formatting, iterate over it
 * with the Format<char> (or signed/unsigned char) class.
 *
 * Do not use this class directly, rather call it via either the
 * PushAndPop or Format classes explicitly. This makes the code more readable. e.g.
 *
 * @code
 * PushAndPop<unsigned char>::Formatter format_array_1;
 * Format< PushAndPop<unsigned char,4> > format_array_2;    // same thing
 * @endcode
 *
 * @tparam Byte : byte type, signed char/char/unsigned char.
 * @tparam N : the size of the container to be formatted.
 */
template<typename Byte, size_t N>
class ECL_PUBLIC BytePushAndPopFormatter
{
public:
  /**
   * Default constructor that simply sets up the formatter for use. There
   * is only one formatting style for this - a byte by byte view of the
   * array in hex format, so we don't accept any arguments. If you want
   * specialised formatting for your array, iterate it element by element
   * with the Format<signed char> class.
   **/
  BytePushAndPopFormatter() : ready_to_format(false)
  {};
  /**
   * The format operator. Use this when inserting into a stream. It
   * returns a template on which the stream will perform the formatting
   * to create the requested output.
   * @param array : the array to format.
   * @return Format<PushAndPop<signed char,N>>& : this formatter readied for use by a stream.
   */
  BytePushAndPopFormatter<Byte,N>& operator()(const ecl::PushAndPop<Byte,N> &push_and_pop)
  {
    push_and_pop_container = &push_and_pop;
    ready_to_format = true;
    return *this;
  }

  virtual ~BytePushAndPopFormatter()
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
  friend OutputStream& operator << (OutputStream& ostream, const BytePushAndPopFormatter<CharType,M> &formatter) ecl_assert_throw_decl(StandardException);

private:
  const typename ecl::PushAndPop<Byte,N> *push_and_pop_container;
  bool ready_to_format;
};

/*****************************************************************************
 ** Implementation [ByteArrayFormatter]
 *****************************************************************************/

template <typename OutputStream, typename CharType, size_t M>
OutputStream& operator <<(OutputStream& ostream, const BytePushAndPopFormatter<CharType, M> &formatter)
ecl_assert_throw_decl(StandardException)
{
  ecl_assert_throw(formatter.ready_to_format, StandardException(LOC,UsageError,"The formatter cannot print any data - "
          "either there is no data available, or you have tried to use the "
          "formatter more than once in a single streaming operation. "
          "C++ produces unspecified results when functors are used multiply "
          "in the same stream sequence, so this is not permitted here.") );

  ecl::Format<CharType> format(-1, ecl::NoAlign, ecl::Hex);
  ostream  << "[ ";
  for ( unsigned int i = 0; i < formatter.push_and_pop_container->size(); ++i ) {
    ostream << format((*(formatter.push_and_pop_container))[i]) << " ";
  }
  ostream << "]";
  ostream.flush();
  return ostream;
}

/*****************************************************************************
 ** Specialisations [PushAndPopFormatter][Char Types]
 *****************************************************************************/

/**
 * @brief Convenience formatter for viewing an unsigned char push and pop containers in hex format.
 *
 * We dont accept a wide range of arguments for this class, it is simply
 * designed to present byte push and pop containers in hex format byte by byte. If you
 * want specialised formatting for the byte array, iterate over it
 * with the Format<unsigned char> class.
 *
 * @tparam N : the size of the container to be formatted.
 */
template<size_t N>
class ECL_PUBLIC PushAndPopFormatter< unsigned char,N >
{
public:
  typename ecl::formatters::BytePushAndPopFormatter<unsigned char, N>& operator()(const ecl::PushAndPop<unsigned char, N> &container) {
    return formatter(container);
  }

private:
  typename ecl::formatters::BytePushAndPopFormatter<unsigned char, N> formatter;
};


/*****************************************************************************
** Not Yet Implemented - formatters for signed, char types
*****************************************************************************/

} // namespace formatters
} // namespace ecl


#endif /* ECL_CONTAINERS_PUSH_AND_POP_FORMATTERS_HPP_ */
