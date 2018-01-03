/**
 * @file /ecl_containers/include/ecl/containers/stencil/specialisations.hpp
 * 
 * @brief Short description of this file.
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_containers_SPECIALISATIONS_HPP_
#define ecl_containers_SPECIALISATIONS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "stencil.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Stencil variant operating on a raw unsigned char array.
 *
 * This of course assumes the user is responsible for range checking on the
 * unsigned char array.
 *
 * TODO : generalise this for T*.
 *
 * @sa ecl::Stencil
 */
template <>
class ECL_PUBLIC Stencil<unsigned char*>
{
public:
  /*********************
   ** Typedefs
   **********************/
  typedef unsigned char value_type; /**< @brief Uses the array's element type. **/
  typedef unsigned char* iterator; /**< @brief Uses the array's iterator type. **/
  typedef const unsigned char* const_iterator; /**< @brief Uses the array's constant iterator type. **/
  typedef unsigned char& reference; /**< @brief Uses the array's element reference type. **/
  typedef const unsigned char& const_reference; /**< @brief Uses the array's element const reference type. **/
  typedef std::size_t size_type; /**< @brief Uses the array's type used to denote the length of the array. **/
  typedef std::ptrdiff_t difference_type;
  typedef std::reverse_iterator<iterator> reverse_iterator; /**< @brief Uses the array's reverse iterator type. **/
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator; /**< @brief Uses the array's constant reverse iterator type. **/
  //typedef formatters::StencilFormatter<value_type,Array> Formatter;

  /*********************
   ** C&D's
   **********************/
  /**
   * @brief Initialises with a pointer to the underlying array with boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * stl containers.
   *
   * @param underlying_array : reference to the underlying array.
   * @param length : of the underlying array
   * @param begin_iter : start of the stencil window.
   * @param end_iter : end of the stencil window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(iterator underlying_array, const unsigned int& length, iterator begin_iter, iterator end_iter) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  length(length),
  b_iter(begin_iter),
  e_iter(end_iter)
  {
    ecl_assert_throw((b_iter <= array+length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array+length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }

  /**
   * @brief Initialises with a reference to the underlying array and boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * eigen block intialisations.
   *
   * @param underlying_array : reference to the underlying array.
   * @param length : of the underlying array
   * @param start_index : start of the stencil window.
   * @param n : number of elements to include in the window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(iterator underlying_array, const unsigned int& length, const unsigned int& start_index = 0, const unsigned int &n = 0) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  length(length),
  b_iter(array+start_index),
  e_iter(array+start_index+n)
  {
    ecl_assert_throw((b_iter <= array+length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array+length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
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
  Stencil<unsigned char*> stencil(const unsigned int& start_index, const unsigned int& n) const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter+start_index <= array+length, StandardException(LOC, OutOfRangeError, "Start index provided is larger than the underlying stencil size."));
    ecl_assert_throw( b_iter+start_index+n <= array+length, StandardException(LOC, OutOfRangeError, "Finish index provided is larger than the underlying stencil size."));
    // cant do out of range errors here (no idea how long the underlying array is!)
    return Stencil<unsigned char*>(array,length,b_iter+start_index,b_iter+start_index+n);
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

    b_iter = array+start_index;
    e_iter = array+start_index+n;
    ecl_assert_throw((b_iter <= array + length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array + length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
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
   * unsigned char buffer[4];          // At this point it is uninitialised.
   * buffer << 0xff, 0x00, 0x01, 0x02; // If NDEBUG is not defined, this will throw if you exceed the range.
   * @endcode
   *
   * @param value : the first value to enter , ElementType, ArraySizeinto the array.
   * @return BoundedListInitialiser : the comma initialiser mechanism.
   */
  containers::BoundedListInitialiser<value_type,iterator> operator<< (const value_type &value)
  {
    return containers::BoundedListInitialiser<value_type,iterator>(value, begin(), size());
  }
  /**
   * @brief This either resettles this stencil or copies across to it (depending on the rhs stencil).
   *
   * The assignment operator has one of two effects - it will either
   * reassign this stencil if both stencils work on the same underlying
   * array, OR it copies across if acting on two different arrays.
   *
   * @param s
   * @return Stencil<unsigned char*> : a handle to this stencil
   *
   * @exception : StandardException : if copying from another stencil/array, throws if size exceeded [debug mode only].
   * @see Stencil
   */
  Stencil<unsigned char*>& operator=(const Stencil<unsigned char*> &s) ecl_assert_throw_decl(StandardException)
  {
    array = s.array;
    length = s.length;
    b_iter = s.b_iter;
    e_iter = s.e_iter;
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
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the start of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator begin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates an pointer (iterator) pointing to the end of the array.
   * @return iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  iterator end() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the end of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator end() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a reverse iterator pointing to the end of the array.
   * @return reverse_iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rbegin() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return reverse_iterator(end());
  }
  /**
   * Generates a constant reverse iterator pointing to the end of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rbegin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return const_reverse_iterator(end());
  }
  /**
   * Generates a reverse iterator pointing to the beginning of the array.
   * @return reverse_iterator : points to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rend() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return reverse_iterator(begin());
  }
  /**
   * Generates a constant reverse iterator pointing to the beginning of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rend() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates a constant reference to the first element in the array (cannot change the value).
   * @return const_reference : const_reference to the first element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference front() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates an reference to the last element in the array.
   * @return reference : reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reference back() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *(e_iter-1);
  }
  /**
   * Generates a constant reference to the last element in the array (cannot change the value).
   * @return const_reference : const_reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference back() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter+i >= array, StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter+i >= array, StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array+length, StandardException(LOC,OutOfRangeError));
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
    if ( b_iter+i <= array )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array+length )
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
    if ( b_iter+i <= array )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array+length )
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
  template <typename OutputStream>
  friend OutputStream& operator<<(OutputStream &ostream , const Stencil<unsigned char*> &stencil);

private:
  iterator array;
  unsigned int length;
  iterator b_iter, e_iter;
};

/*****************************************************************************
 ** Const Specialisations
 *****************************************************************************/

/**
 * @brief Stencil variant operating on a raw const unsigned char array.
 *
 * This of course assumes the user is responsible for range checking on the
 * unsigned char array.
 *
 * TODO : generalise this for const T*.
 *
 * @sa ecl::Stencil
 */
template <>
class ECL_PUBLIC Stencil<const unsigned char*>
{
public:
  /*********************
   ** Typedefs
   **********************/
  typedef unsigned char value_type; /**< @brief Uses the array's element type. **/
  typedef const unsigned char* iterator; /**< @brief Uses the array's iterator type. **/
  typedef const unsigned char* const_iterator; /**< @brief Uses the array's constant iterator type. **/
  typedef const unsigned char& reference; /**< @brief Uses the array's element reference type. **/
  typedef const unsigned char& const_reference; /**< @brief Uses the array's element const reference type. **/
  typedef std::size_t size_type; /**< @brief Uses the array's type used to denote the length of the array. **/
  typedef std::ptrdiff_t difference_type;
  typedef std::reverse_iterator<iterator> reverse_iterator; /**< @brief Uses the array's reverse iterator type. **/
  typedef std::reverse_iterator<const_iterator> const_reverse_iterator; /**< @brief Uses the array's constant reverse iterator type. **/
  //typedef formatters::StencilFormatter<value_type,Array> Formatter;

  /*********************
   ** C&D's
   **********************/
  /**
   * @brief Initialises with a pointer to the underlying array with boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * stl containers.
   *
   * @param underlying_array : reference to the underlying array.
   * @param length : of the underlying array
   * @param begin_iter : start of the stencil window.
   * @param end_iter : end of the stencil window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(const_iterator underlying_array, const unsigned int& length, const_iterator begin_iter, const_iterator end_iter) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  length(length),
  b_iter(begin_iter),
  e_iter(end_iter)
  {
    ecl_assert_throw((b_iter <= array+length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array+length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }

  /**
   * @brief Initialises with a reference to the underlying array and boundary constraints.
   *
   * Sets up the underlying storage container along with boundary constraints in the style of
   * eigen block intialisations.
   *
   * @param underlying_array : reference to the underlying array.
   * @param length : of the underlying array
   * @param start_index : start of the stencil window.
   * @param n : number of elements to include in the window.
   *
   * @exception : StandardException : throws if the indices provided are out of range [debug mode only].
   */
  Stencil(const_iterator underlying_array, const unsigned int& length, const unsigned int& start_index = 0, const unsigned int &n = 0) ecl_assert_throw_decl(StandardException) :
  array(underlying_array),
  length(length),
  b_iter(array+start_index),
  e_iter(array+start_index+n)
  {
    ecl_assert_throw((b_iter <= array+length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array+length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
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
  Stencil<const unsigned char*> stencil(const unsigned int& start_index, const unsigned int& n) const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter+start_index <= array+length, StandardException(LOC, OutOfRangeError, "Start index provided is larger than the underlying stencil size."));
    ecl_assert_throw( b_iter+start_index+n <= array+length, StandardException(LOC, OutOfRangeError, "Finish index provided is larger than the underlying stencil size."));
    // cant do out of range errors here (no idea how long the underlying array is!)
    return Stencil<const unsigned char*>(array,length,b_iter+start_index,b_iter+start_index+n);
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

    b_iter = array+start_index;
    e_iter = array+start_index+n;
    ecl_assert_throw((b_iter <= array + length), StandardException(LOC, OutOfRangeError, "Begin iterator out of range."));
    ecl_assert_throw((e_iter <= array + length), StandardException(LOC, OutOfRangeError, "End iterator out of range."));
  }
  /**
   * @brief This either resettles this stencil or copies across to it (depending on the rhs stencil).
   *
   * The assignment operator has one of two effects - it will either
   * reassign this stencil if both stencils work on the same underlying
   * array, OR it copies across if acting on two different arrays.
   *
   * @param s
   * @return Stencil<unsigned char*> : a handle to this stencil
   *
   * @exception : StandardException : if copying from another stencil/array, throws if size exceeded [debug mode only].
   * @see Stencil
   */
  Stencil<const unsigned char*>& operator=(const Stencil<const unsigned char*> &s) ecl_assert_throw_decl(StandardException)
  {
    array = s.array;
    length = s.length;
    b_iter = s.b_iter;
    e_iter = s.e_iter;
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
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the start of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator begin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return b_iter;
  }
  /**
   * Generates an pointer (iterator) pointing to the end of the array.
   * @return iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  iterator end() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a const pointer (iterator) pointing to the end of the array.
   * @return const_iterator : constant pointer (iterator) pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_iterator end() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return e_iter;
  }
  /**
   * Generates a reverse iterator pointing to the end of the array.
   * @return reverse_iterator : points to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rbegin() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return reverse_iterator(end());
  }
  /**
   * Generates a constant reverse iterator pointing to the end of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the end of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rbegin() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return const_reverse_iterator(end());
  }
  /**
   * Generates a reverse iterator pointing to the beginning of the array.
   * @return reverse_iterator : points to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reverse_iterator rend() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return reverse_iterator(begin());
  }
  /**
   * Generates a constant reverse iterator pointing to the beginning of the array.
   * @return const_reverse_iterator : constant reverse iterator pointing to the beginning of the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reverse_iterator rend() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates a constant reference to the first element in the array (cannot change the value).
   * @return const_reference : const_reference to the first element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference front() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( b_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *b_iter;
  }
  /**
   * Generates an reference to the last element in the array.
   * @return reference : reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  reference back() ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
    return *(e_iter-1);
  }
  /**
   * Generates a constant reference to the last element in the array (cannot change the value).
   * @return const_reference : const_reference to the last element in the array.
   * @exception : StandardException : throws if the underlying array changed size and the stencil is now out of range [debug mode only].
   */
  const_reference back() const ecl_assert_throw_decl(StandardException)
  {
    ecl_assert_throw( e_iter <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter+i >= array, StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array+length, StandardException(LOC,OutOfRangeError));
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
    ecl_assert_throw( b_iter+i >= array, StandardException(LOC,OutOfRangeError));
    ecl_assert_throw( b_iter+i <= array+length, StandardException(LOC,OutOfRangeError));
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
    if ( b_iter+i <= array )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array+length )
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
    if ( b_iter+i <= array )
    {
      throw StandardException(LOC,OutOfRangeError);
    }
    if ( b_iter+i >= array+length )
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
  template <typename OutputStream>
  friend OutputStream& operator<<(OutputStream &ostream , const Stencil<const unsigned char*> &stencil);

private:
  const_iterator array;
  unsigned int length;
  const_iterator b_iter, e_iter;
};

} // namespace ecl

#endif /* ecl_containers_SPECIALISATIONS_HPP_ */
