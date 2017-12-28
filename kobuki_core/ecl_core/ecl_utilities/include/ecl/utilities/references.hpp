/**
 * @file /include/ecl/utilities/references.hpp
 *
 * @brief Defines a reference wrapper for c++ objects.
 *
 * Reference wrappers are soon coming to c++, these definitions will work
 * until that time.
 *
 * @date July 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_UTILITIES_REFERENCE_WRAPPER_HPP_
#define ECL_UTILITIES_REFERENCE_WRAPPER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/mpl/bool.hpp>

/*****************************************************************************
** Wrappers
*****************************************************************************/

namespace ecl {

/**
 * @brief Returns the address of an object (uses correct c++ syntactics).
 *
 * @param v : the object located at this address.
 * @return T* : a pointer to the object.
 *
 * @tparam T : the type of the object.
 */
template <typename T>
T* addressOf( T & v ) {
    return reinterpret_cast<T*>(
        &const_cast<char&>(reinterpret_cast<const volatile char &>(v)));
}
/**
 * @brief Provides a wrapper which allows the original object to be passed like a reference.
 *
 * This class wraps the supplied object, storing it as a pointer and allowing it
 * to be passed like a regular object. By itself, this is not terribly useful, but
 * in combination with the ref/cref functions and also the is_reference_wrapper trait,
 * it allows things like functions and function objects to be easily manipulated.
 *
 * Note: reference wrappers might seem a bit redundant - why do we need them?
 * The following case illustrates a desired usage scenario.
 *
 * @code
   class A {
   public:
       template <typename T>
       void f(T i) { cout << "Not a reference input" << endl; }

       template <typename T>
       void f(T &i) { cout << "A reference input" << endl; }
   }
   @endcode
 * Unfortunately, when you come to call this function, you'll get an ambiguity error.
 *
 * In these situations you can use a simple function call (not ref) and pass in this
 * reference wrapper:
 * @code
   class A {
   public:
       template <typename T>
       void f(const T &t) {
           if ( is_reference_wrapper<T> ) {
               // do something for references
           } else {
               // do something for non-references
           }
       }
   }
   @endcode
 *
 * @tparam T : the type of object this class is wrapping.
 */
template<typename T>
class ReferenceWrapper {

public:
    typedef T type; /**< Convenient handle to the reference wrapper's base type. **/

    /**
     * @brief Constructs the wrapper around the supplied object instance.
     *
     * Constructs the wrapper around the supplied object instance.
     *
     * @param t : the object instance.
     */
    ReferenceWrapper(T& t): obj_ptr(addressOf(t)) {}

    virtual ~ReferenceWrapper() {}

    /**
     * @brief Convenience operator for converting to the underlying reference directly.
     *
     * When used where the underlying reference is expected, this automatically handles
     * the conversion.
     *
     * @return T& : the returned reference to the wrapped object.
     */
    operator T& () const { return *obj_ptr; }

    /**
     * @brief Accessor to the reference of the original instance.
     *
     * Returns a reference to the wrapped object.
     *
     * @return T& : the returned reference to the wrapped object.
     */
    T& reference() const { return *obj_ptr; }

    /**
     * @brief Accessor to a pointer to the original instance.
     *
     * Returns a pointer to the wrapped object.
     *
     * @return T& : the returned pointer to the wrapped object.
     */
    T* pointer() const { return *obj_ptr; }

private:
    T* obj_ptr;
};

/*****************************************************************************
** Utility Functions
*****************************************************************************/

/**
 * This is a simple means of instantiating a reference wrapper.
 *
 * This is a simple means of instantiating a reference wrapper.
 * @param wrapped_object : the object to be referenced.
 * @return ReferenceWrapper<T> : the wrapper.
 */
template <typename T>
ReferenceWrapper<T> ref(T &wrapped_object) {
	return ReferenceWrapper<T>(wrapped_object);
}

/**
 * This is a simple means of instantiating a const reference wrapper.
 *
 * This is a simple means of instantiating a const reference wrapper.
 * @param wrapped_object : the object to be referenced.
 * @return const ReferenceWrapper<T> : the wrapper.
 */
template <typename T>
ReferenceWrapper<T const> cref(T const &wrapped_object) {
	return ReferenceWrapper<T const>(wrapped_object);
}

/**
 * @brief Default action for detection of the reference wrapper type trait (false).
 *
 * Sets the default value (false) for detection of the reference wrapper type trait.
 * Specialisations enable this to true.
 *
 * @tparam T : the test type.
 */
template <typename T>
class is_reference_wrapper : public False {};

/**
 * @brief Configures the reference wrapper type trait for the ReferenceWrapper specialisation to true.
 *
 * Configures the reference wrapper type trait for ReferenceWrapper objects to true.
 *
 * @tparam T : the object wrapped by ReferenceWrapper.
 */
template <typename T>
class is_reference_wrapper< ReferenceWrapper<T> > : public True {};

} // namespace ecl


#endif /* ECL_UTILITIES_REFERENCE_WRAPPER_HPP_ */
