/**
 * @file /include/ecl/containers/stencil/stencil.hpp
 *
 * @brief Windows over ecl type containers.
 *
 * Provides a windowed snapshot over a portion of an ecl container.
 *
 * @date March 2010
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef ECL_CONTAINERS_STENCIL_STENCIL_HPP_
#define ECL_CONTAINERS_STENCIL_STENCIL_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <cstddef>  // size_t
#include <ecl/config/macros.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/concepts/containers.hpp>
#include <ecl/concepts/streams.hpp>
#include "../initialiser.hpp"
#include "formatters.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace ecl
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief A safe windowing class that opens onto array-like containers.
 *
 * Functions often require access to just a portion of an array. In these cases
 * pointers are typically used, but these end up being dangerous alot of times.
 * The underlying array may disappear, or its size may change and the resulting
 * pointers are left dangling, pointing at rubbish.
 *
 * This class allows such an operation to be performed with two additional
 * benefits:
 *
 * - It provides additional container like functionality.
 * - Users references and range checking to ensure critical problems cannot arise.
 *
 * Currently this works with anything that measures up to the ecl concept of a
 * container.
 *
 * <b>Usage</b>:
 *
 * @code
 * typedef Array<int,5> FixedArray;
 * typedef Stencil< FixedArray > FixedStencil;
 *
 * FixedArray array;
 * array << 1,2,3,4,5;
 * FixedStencil stencil(array,1,3); // windows from index 1 to 3
 * std::cout << stencil[1] << std::endl; // accesses array[2];
 *
 * @endcode
 *
 * @sa ecl::ContainerConcept, ecl::Array.
 */
template <typename Array>
class ECL_PUBLIC Stencil
{
public:
  /*********************
   ** Typedefs
   **********************/
  typedef typename Array::value_type value_type; /**< @brief Uses the array's element type. **/
  //   Looks like I'm now needing to rely on the underlying storage iterator definitions
  //   Naturally, this means I can't do stencils of c arrays, but hey, thats dangerous anyway and we want to avoid that.
  typedef typename Array::iterator iterator; /**< @brief Uses the array's iterator type. **/
  typedef typename Array::const_iterator const_iterator; /**< @brief Uses the array's constant iterator type. **/
  typedef typename Array::reference reference; /**< @brief Uses the array's element reference type. **/
  typedef typename Array::const_reference const_reference; /**< @brief Uses the array's element const reference type. **/
  // typedef typename Array::value_type*          iterator;   /**< @brief Uses the array's iterator type. **/
  // typedef const typename Array::value_type*    const_iterator;  /**< @brief Uses the array's constant iterator type. **/
  // typedef typename Array::value_type&          reference;  /**< @brief Uses the array's element reference type. **/
  // typedef const typename Array::value_type&    const_reference; /**< @brief Uses the array's element const reference type. **/
  typedef std::size_t size_type; /**< @brief Uses the array's type used to denote the length of the array. **/
  typedef std::ptrdiff_t difference_type;
  typedef std::reverse_iterator<iterator> reverse_iterator; /**< @brief Uses the array's reverse iterator type. **/
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator; /**< @brief Uses the array's constant reverse iterator type. **/
  typedef formatters::StencilFormatter<value_type,Array> Formatter;

  /*********************
   ** C&D's
   **********************/
  /**
   * @brief Initialises with a reference to the underlying container and boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * stl containers.
   *
   * Concept Check: makes sure the template parameter provided for the class is
   * a container type with the required functionality.
   *
   * @param underlying_array : reference to the underlying array.
   * @param begin_iter : start of the stencil window.
   * @param end_iter : end of the stencil window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(Array& underlying_array, iterator begin_iter, iterator end_iter) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  b_iter(begin_iter),
  e_iter(end_iter)
  {
    ecl_compile_time_concept_check(ContainerConcept<Array>);
    ecl_assert_throw((b_iter >= array.begin()) && (b_iter <= array.end()), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter >= array.begin()) && (e_iter <= array.end()), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }

  /**
   * @brief Initialises with a reference to the underlying container and boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * eigen block intialisations.
   *
   * Concept Check: makes sure the template parameter provided for the class is
   * a container type with the required functionality.
   *
   * @param underlying_array : reference to the underlying array.
   * @param start_index : start of the stencil window.
   * @param n : number of elements to include in the window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(Array& underlying_array, const unsigned int& start_index = 0, const unsigned int &n = 0) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  b_iter(array.begin()+start_index),
  e_iter(array.begin()+start_index+n)
  {
    ecl_compile_time_concept_check(ContainerConcept<Array>);
    ecl_assert_throw((b_iter >= array.begin()) && (b_iter <= array.end()), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter >= array.begin()) && (e_iter <= array.end()), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }

  virtual ~Stencil()
  {};

  /**
   * @brief Generate a sub-stencil.
   *
   * Opens another window on this stencil.
   * Using a starting index + size is in line with the way eigen blocks and segments work.
   *
   * @param start_index : start of the stencil window.
   * @param n : number of elements to include in the window.
   * @return Stencil<Array> : the generated sub-stencil.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil<Array> stencil(const unsigned int& start_index, const unsigned int& n) const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter+start_index < array.end(), StandardException(LOC, OutOfRangeError, "Start index provided is larger than the underlying stencil size."));
    ecl_assert_throw( b_iter+start_index+n <= array.end(), StandardException(LOC, OutOfRangeError, "Finish index provided is larger than the underlying stencil size."));
    return Stencil<Array>(array,b_iter+start_index,b_iter+start_index+n);
  }

  /**
   * @brief Resettle the stencil on a different range over the same underlying array.
   *
   * Resets the range of the stencil operating on the currently referenced array.
   * Using a starting index + size is in line with the way eigen blocks and segments work.
   *
   * @param start_index : start of the stencil window.
   * @param n : number of elements to include in the window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  void resettle(const unsigned int& start_index, const unsigned int& n) ecl_assert_throw_decl(StandardException)
  {

    b_iter = array.begin()+start_index;
    e_iter = array.begin()+start_index+n;
    ecl_assert_throw((b_iter >= array.begin()) && (b_iter <= array.end()), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter >= array.begin()) && (e_iter <= array.end()), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }

  /*********************
   ** Assignment
   **********************/
  /**
   * Provides a comma initialisation facility. This initiates the comma initialiser
   * with an iterator to the underlying array and then leaves the initialiser to
   * do the rest. The initialiser will do range checking if NDEBUG is not defined.
   *
   * @code
   * Array<int,4> array; // At this point it is uninitialised.
   * array << 1,2,3,4;    // If NDEBUG is not defined, this will throw if you exceed the range.
   * @endcode
   *
   * @param value : the first value to enter , ElementType, ArraySizeinto the array.
   * @return BoundedListInitialiser : the comma initialiser mechanism.
   */
  containers::BoundedListInitialiser<value_type,value_type*> operator<< (const value_type &value)
  {
    return containers::BoundedListInitialiser<value_type,iterator>(value, begin, size());
  }
  /**
   * @brief This either resettles this stencil or copies across to it (depending on the rhs stencil).
   *
   * The assignment operator has one of two effects - it will either
   * reassign this stencil if both stencils work on the same underlying
   * array, OR it copies across if acting on two different arrays.
   *
   * <b>Usage: </b>
   *
   * The first case, resettling:
   *
   * @code
   * Array<char> array(5); array << 1,2,3,4,5;
   * Stencil< Array<char> > stencil = array.stencil(1,2);
   * stencil = array.stencil(0,3); // Here we can repeat the reassignment
   * @endcode
   *
   * The second case, copying:
   * @code
   * Array<char> array1(5); array1 << 1,2,3,4,5;
   * Array<char> array2(6); array2 << 1,2,3,4,5,6;
   * Stencil< Array<char> > stencil = array1.stencil(1,2);
   * stencil = array2.stencil(2,2); // Here we just flood fill the stencil - make sure the capacity is sufficient!
   * @endcode
   *
   * @param s
   * @return Stencil<Array> : a handle to this stencil
   *
   * @exception : StandardException : if copying from another stencil/array, throws if size exceeded [debug mode only].
   */
  Stencil<Array>& operator=(const Stencil<Array> &s) ecl_assert_throw_decl(StandardException)
  {
    if ( &array == &(s.array) )
    { // resettle
      b_iter = s.b_iter;
      e_iter = s.e_iter;
    }
    else
    { // copy
      ecl_assert_throw( s.size() <= size(), StandardException(LOC, OutOfRangeError, "Stencil to be copied is larger than this stencil's size."));
      for ( unsigned int i = 0; i < s.size(); ++i )
      {
        *(b_iter+i) = *(s.b_iter+i);
      }
    }
    return *this;
  }

  /*********************
   ** Iterators
   **********************/
  /**
   * Generates a pointer (iterator) pointing to the start of the stencil.
   * @return iterator : points to the beginning of the stencil.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  iterator begin() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the start of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator begin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates an pointer (iterator) pointing to the end of the array.
   * @return iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  iterator end() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the end of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator end() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a reverse iterator pointing to the end of the array.
   * @return reverse_iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rbegin() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return reverse_iterator(end());
  }
  /**
   * Generates a constant reverse iterator pointing to the end of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rbegin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return const_reverse_iterator(end());
  }
  /**
   * Generates a reverse iterator pointing to the beginning of the array.
   * @return reverse_iterator : points to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rend() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return reverse_iterator(begin());
  }
  /**
   * Generates a constant reverse iterator pointing to the beginning of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rend() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return const_reverse_iterator(begin());
  }

  /*********************
   ** Front/Back
   **********************/
  /**
   * Generates an reference to the first element in the array.
   * @return reference : reference to the first element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reference front() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates a constant reference to the first element in the array (cannot change the value).
   * @return const_reference : const_reference to the first element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference front() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates an reference to the last element in the array.
   * @return reference : reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reference back() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return *(e_iter-1);
  }
  /**
   * Generates a constant reference to the last element in the array (cannot change the value).
   * @return const_reference : const_reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference back() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array.end(), StandardException(LOC,OutOfRangeError));
    return *(e_iter-1);
  }

  /*********************
   ** Accessors
   **********************/
  /**
   * Accesses elements in the stencil, returning references to the requested element. This
   * accessor only does range checks in debug mode (NDEBUG is not defined). Compare this with
   * the at() accessor which always checks if the range is exceeded.
   *
   * @return reference : a reference to the requested element.
   *
   * @exception : StandardException : throws if range is requested element is out of range [debug mode only].
   **/
  reference operator[](size_type i) ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter+i >= array.begin(), StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array.end(), StandardException(LOC,OutOfRangeError));
    return *(b_iter+i);
  }
  /**
   * Accesses elements in the  stencil, returning references to the requested element. This
   * accessor only does range checks in debug mode (NDEBUG is not defined). Compare this with
   * the at() accessor which always checks if the range is exceeded. This also ensures the
   * references are constant, which in turn ensures the contents cannot of the array cannot be
   * modified.
   *
   * @return const_reference : a constant reference to the requested element.
   *
   * @exception : StandardException : throws if range is requested element is out of range [debug mode only].
   **/
  const_reference operator[](size_type i) const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter+i >= array.begin(), StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array.end(), StandardException(LOC,OutOfRangeError));
    return *(b_iter+i);
  }
  /**
   * Accesses elements in the stencil, returning references to the requested element. This
   * accessor always does range checks. Compare this with
   * the [] accessor which only checks if NDEBUG is not defined.
   *
   * @param i : the index of the requested element.
   * @return reference : a reference to the requested element.
   *
   * @exception : StandardException : throws if range is requested element is out of range.
   */
  reference at(size_type i) throw(StandardException)
  {
    if ( b_iter+i <= array.begin() )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array.end() )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    return *(b_iter+i);
  }
  /**
   * Accesses elements in the stencil, returning references to the requested element. This
   * accessor always does range checks. Compare this with
   * the [] accessor which only checks if NDEBUG is not defined. This also ensures the
   * references are constant, which in turn ensures the contents cannot of the array cannot be
   * modified.
   *
   * @param i : the index of the requested element.
   * @return const_reference : a const_reference to the requested element.
   *
   * @exception : StandardException : throws if range is requested element is out of range.
   */
  const_reference at(size_type i) const throw(StandardException)
  {
    if ( b_iter+i <= array.begin() )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array.end() )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    return *(b_iter+i);
  }

  /*********************
   ** Utilities
   **********************/
  /**
   * The size of the stencil.
   *
   * @return size_type : the size of the stencil.
   */
  size_type size() const
  { return e_iter-b_iter;}

  /*********************
   ** Streaming
   **********************/
  /**
   * @brief Insertion operator for stencils.
   *
   * Insertion operator for sending the stencil to an output stream. This
   * is raw, and has no formatting.
   *
   * Concept Check: makes sure the template parameter provided for the stream
   * is a stream type with the required functionality.
   *
   * @param ostream : the output stream.
   * @param stencil : the stencil to be inserted.
   * @tparam OutputStream : the stream being used.
   * @tparam ArrayType : the type of the underlying array.
   * @return OutputStream : continue streaming with the updated output stream.
   */
  template <typename OutputStream, typename ArrayType>
  friend OutputStream& operator<<(OutputStream &ostream , const Stencil<ArrayType> &stencil);

private:
  Array& array;
  iterator b_iter, e_iter;
};

/*****************************************************************************
 ** Implementation [Streaming]
 *****************************************************************************/

template<typename OutputStream, typename ArrayType>
  OutputStream& operator<<(OutputStream &ostream, const Stencil<ArrayType> &stencil)
  {

    ecl_compile_time_concept_check (StreamConcept<OutputStream> );

    ostream << "[ ";
    for (std::size_t i = 0; i < stencil.size(); ++i)
    {
      ostream << stencil[i] << " ";
    }
    ostream << "]";
    ostream.flush();

    return ostream;
  }

} // namespace ecl

#endif /* ECL_CONTAINERS_STENCIL_STENCIL_HPP_ */
