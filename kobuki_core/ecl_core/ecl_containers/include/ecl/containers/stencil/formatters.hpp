/**
 * @file /ecl_containers/include/ecl/containers/stencil/formatters.hpp
 *
 * @brief Text formatters for streaming stencil classes.
 *
 * @date March 11
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONTAINERS_STENCIL_FORMATTERS_HPP_
#define ECL_CONTAINERS_STENCIL_FORMATTERS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cmath>
#include <ecl/config/macros.hpp>
#include <ecl/formatters/common.hpp>
#include <ecl/formatters/number.hpp>
#include <ecl/formatters/floats.hpp>
#include <ecl/mpl/enable_if.hpp>
#include <ecl/type_traits/fundamental_types.hpp>
#include "../common/formatters.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

/*****************************************************************************
 ** Forward Declarations
 *****************************************************************************/

template<typename Container>
  class Stencil;

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace formatters
{

/*****************************************************************************
 ** Using
 *****************************************************************************/

/**
 * @brief Pseudo formatter for stencils.
 *
 * These do nothing but pass the stencil back for streaming which should have
 * its own << operator.
 *
 * Do not use this class directly, rather call it via either the Array or
 * Format classes explicitly. This makes the code more readable. e.g.
 *
 * @code
 * Stencil< Array<int, 5> >::Formatter format_1;
 * Format< Stencil< Array<int,4> > >   format_2;    // same thing
 * @endcode
 *
 * @tparam ValueType : the underlying container's element type.
 * @tparam Container : the underlying container storage.
 * @tparam Enable : dummy template parameter used for mpl.
 */
template<typename ValueType, typename Container, typename Enable = void>
class ECL_PUBLIC StencilFormatter
{
public:
  virtual ~StencilFormatter()
  {}
  /**
   * Pseudo formatter method, simply returns the underlying stencil which
   * has its own stream operator.
   * @param stencil : the stencil to be formatted.
   * @return Stencil< Container > : just returns the input array.
   */
  const ecl::Stencil< Container >& operator()(const ecl::Stencil< Container > &stencil) {
    return stencil;
  }
  //ecl::Stencil< Container >& operator()(const ecl::Stencil< Container > &stencil) { return stencil;} // this was the old version - couldn't do in place format(xxx.stencil()) commands
};

/*****************************************************************************
 ** Specialisations
 *****************************************************************************/

/**
 * @brief Partial specialisation of the stencil formatter for float containers.
 *
 * Allows precision settings for formatting of float stencils.
 *
 * @tparam N : the size of the fixed size float array to be formatted.
 *
 * @sa @ref ecl::formatters::FloatContainerFormatter "FloatContainerFormatter".
 */
template <typename ValueType, typename Container>
class ECL_PUBLIC StencilFormatter< ValueType, Container, typename ecl::enable_if< ecl::is_float<ValueType> >::type > : public FloatContainerFormatter< ecl::Stencil< Container > >
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
  StencilFormatter(const int p=2, const int w=-1) : FloatContainerFormatter< ecl::Stencil< Container > >(p,w) {};
  virtual ~StencilFormatter()
  {}
};

} // namespace formatters
} // namespace ecl

#endif /* ECL_CONTAINERS_STENCIL_FORMATTERS_HPP_ */
