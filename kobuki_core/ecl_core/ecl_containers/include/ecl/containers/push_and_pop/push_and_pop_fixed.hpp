/**
 * @file /include/ecl/containers/push_and_pop/push_and_pop_fixed.hpp
 *
 * @brief A simple fifo implementation.
 *
 * Internally it uses ecl::Array for various error handling and other
 * routines.
 *
 * @date August 2010
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONTAINERS_PUSH_AND_POP_FIXED_HPP_
#define ECL_CONTAINERS_PUSH_AND_POP_FIXED_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <iostream> //showMe
#include <algorithm> // std::copy
#include <ecl/config/macros.hpp>
#include <ecl/errors/compile_time_assert.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../array.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{


/*****************************************************************************
** Forward Declarations
*****************************************************************************/

namespace formatters {

template <typename Type, size_t N> class PushAndPopFormatter;

} // namespace formatters

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief
 * Surpport push and pack operation
 *
 * ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG!
 *
 * This is very experimental and has a few unfinished, surprising.
 * automatic behaviours.
 *
 * - if full and pushing back, it makes room by popping off the front
 *
 * <b>Usage</b>:
 *
 *
 * @code
 *
 *
 * @code
 *
 *
 * @endcode
 *
 * @sa ecl::Array.
 */

template<typename Type, std::size_t Size=DynamicStorage>
class ECL_PUBLIC PushAndPop
{
public:
  typedef formatters::PushAndPopFormatter<Type,Size> Formatter; /**< @brief Formatter for this class. **/

  PushAndPop()
  :
  size_fifo(Size+1),
  leader(0),
  follower(0)
  {
  }

  PushAndPop( const Type & d ) ecl_assert_throw_decl(StandardException)
  :
  size_fifo(Size+1),
  leader(0),
  follower(0)
  {
    ecl_assert_throw( (size_fifo>0), StandardException(LOC, OutOfRangeError, "SimpleFIFO start with zero size buffer"));
    fill( d );
  }
  virtual ~PushAndPop()
  {}

  Type & operator[] (int idx)
  {
    return data[ ((follower+idx)%size_fifo) ];
  }

  const Type & operator[] (int idx) const
  {
    return data[ ((follower+idx)%size_fifo) ];
  }

  void operator() (const PushAndPop<Type,Size> & otherOne )
  {
    leader = otherOne.leader;
    follower = otherOne.follower;
    for( int i=0; i<size_fifo; i++ )
    {
      data[i] = otherOne.data[i];
    }
  }

  /**
   * @brief Pushes an element onto the back of the container.
   *
   * If there is no empty room, it simply makes room by
   * popping an element of the front.
   */
  void push_back( const Type & datum )
  {
    data[ leader++ ] = datum;
    leader %= size_fifo;
    if( leader == follower )
    {
      follower++;
      follower %= size_fifo;
      //std::cout << "PushAndPop::push_back : exceeds maximum size, follower catch the leader " << std::endl;
    }
  }

  Type pop_front()
  {
    ecl_assert_throw( (follower != leader), StandardException(LOC, OutOfRangeError, "PushAndPop follower==leader"));
    Type value = data[follower++];
    follower %= size_fifo;
    return value;
  }

  void fill( const Type & d )
  {
    for( unsigned int i=0; i<size_fifo; i++ ) data[i] = d;
  }

  /**
   * To get the assigned buffer size while size() return number of data in fifo
   *
   * @return assigned buffer size
   */
  unsigned int asize()
  {
    return size_fifo;
  }

  unsigned int size() const
  {
    if( leader > follower ) return leader - follower;
    else if( leader < follower ) return size_fifo-follower+leader;
    return 0;
  }

  void clear()
  {
    follower = 0;
    leader = 0;
  }

  void showMe()
  {
    std::cout << "[Size|Leader|Follower]: " << size_fifo << "|" << leader << "|" << follower << std::endl;
  }

private:
  ecl::Array <Type,Size+1>data;
  unsigned int size_fifo;
  int leader;
  int follower;
};

} // namespace ecl

#endif /* ECL_CONTAINERS_PUSH_AND_POP_HPP_ */
