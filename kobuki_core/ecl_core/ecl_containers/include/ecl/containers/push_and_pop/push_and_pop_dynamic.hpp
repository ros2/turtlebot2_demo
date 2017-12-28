/**
 * @file /include/ecl/containers/push_and_pop/push_and_pop_dynamic.hpp
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

#ifndef ECL_CONTAINERS_PUSH_AND_POP_DYNAMIC_HPP_
#define ECL_CONTAINERS_PUSH_AND_POP_DYNAMIC_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ecl/config/macros.hpp>
#include "push_and_pop_fixed.hpp"

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
 * - begin() and end() do not point to the beginning/end of the pushpop,
 *   rather they point to the beginning and ending of the underlying array
 * - if full and pushing back, it makes room by popping off the front
 *
 * <b>Usage</b>:
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
template<typename Type>
class ECL_PUBLIC PushAndPop<Type,DynamicStorage>
{
public:
  typedef Type           value_type; /**< Element type. **/
  typedef Type*          iterator;   /**< Iterator type. **/
  typedef const Type*    const_iterator;  /**< Constant iterator type. **/
  typedef Type&          reference;  /**< Element reference type. **/
  typedef const Type&    const_reference; /**< Element const reference type. **/
  typedef std::size_t    size_type;  /**< Type used to denote the length of the array. **/
  typedef std::ptrdiff_t difference_type;
  typedef std::reverse_iterator<iterator> reverse_iterator; /**< Reverse iterator type. **/
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator;  /**< Constant reverse iterator type. **/
  typedef formatters::PushAndPopFormatter<Type,DynamicStorage> Formatter; /**< @brief Formatter for this class. **/

  /**
   * @brief Default constructor.
   *
   * Creates a dynamic push and pop container of zero length.
   */
  PushAndPop() : size_fifo(0), leader(0), follower(0) {}

  PushAndPop( const unsigned int length ) ecl_assert_throw_decl(StandardException)
  : size_fifo(length+1), leader(0), follower(0)
  {
    ecl_assert_throw( (size_fifo>0), StandardException(LOC, OutOfRangeError, "SimpleFIFO start with zero size buffer"));
    data.resize( size_fifo );
  }

  PushAndPop( const unsigned int length, const Type & d ) ecl_assert_throw_decl(StandardException) :
  size_fifo(length+1),
  leader(0),
  follower(0)
  {
    ecl_assert_throw( (size_fifo>0), StandardException(LOC, OutOfRangeError, "SimpleFIFO start with zero size buffer"));
    data.resize( size_fifo );
    fill( d );
  }
  virtual ~PushAndPop()
  {}

  /*********************
  ** Iterators
  **********************/
  /* ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! ACHTUNG! */
  /*
   * These have not been updated to take care with the push/pop, they just point to the
   * start and end of the underlying array!
   */
  /**
   * Generates a pointer (iterator) pointing to the start of the array.
   * @return iterator : points to the beginning of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  iterator begin() ecl_assert_throw_decl(StandardException) {
    return data.begin(); // underlying array will throw if no storage has been allocated
  }
  /**
   * Generates a const pointer (iterator) pointing to the start of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  const_iterator begin() const ecl_assert_throw_decl(StandardException) {
    return data.begin(); // underlying array will throw if no storage has been allocated
  }
  /**
   * Generates an pointer (iterator) pointing to the end of the array.
   * @return iterator : points to the end of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  iterator end() ecl_assert_throw_decl(StandardException) {
    return data.end(); // underlying array will throw if no storage has been allocated
  }
  /**
   * Generates a const pointer (iterator) pointing to the end of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  const_iterator end() const ecl_assert_throw_decl(StandardException) {
    return data.end(); // underlying array will throw if no storage has been allocated
  }
  /**
   * Generates a reverse iterator pointing to the end of the array.
   * @return reverse_iterator : points to the end of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  reverse_iterator rbegin() ecl_assert_throw_decl(StandardException) {
    return reverse_iterator(end());
  }
  /**
   * Generates a constant reverse iterator pointing to the end of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  const_reverse_iterator rbegin() const ecl_assert_throw_decl(StandardException) {
    return const_reverse_iterator(end());
  }
  /**
   * Generates a reverse iterator pointing to the beginning of the array.
   * @return reverse_iterator : points to the beginning of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  reverse_iterator rend() ecl_assert_throw_decl(StandardException) {
    return reverse_iterator(begin());
  }
  /**
   * Generates a constant reverse iterator pointing to the beginning of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
   *
   * @exception StandardException : throws if no storage has been allocated [debug mode only].
   */
  const_reverse_iterator rend() const ecl_assert_throw_decl(StandardException) {
    return const_reverse_iterator(begin());
  }


  /*********************
  ** Accessors
  **********************/
  // Cannot do stencils easily as push and pop roll over - not guaranteed of being contiguous

  Type & operator[] (int idx)
  {
    return data[ ((follower+idx)%size_fifo) ];
  }

  const Type & operator[] (int idx) const
  {
    return data[ ((follower+idx)%size_fifo) ];
  }

  void operator() (const PushAndPop<Type,0> & otherOne )
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
      // it just rolls over
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

  void resize( unsigned int length )
  {
    size_fifo = length+1;
    ecl_assert_throw( (size_fifo>0), StandardException(LOC, OutOfRangeError, "SimpleFIFO start with zero size buffer"));
    data.resize( size_fifo );
  }

  /**
   * @brief The size allocated in memory for the fifo.
   *
   * This is different to the actual used size.
   *
   * @sa size()
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

//protected:
  ecl::Array <Type>data;
  unsigned int size_fifo;
  int leader;
  int follower;
};

} // namespace ecl

#endif /* ECL_CONTAINERS_PUSH_AND_POP_DYNAMIC_HPP_ */
