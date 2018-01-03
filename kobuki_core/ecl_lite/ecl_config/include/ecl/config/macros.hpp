/**
 * @file /ecl_config/include/ecl/config/macros.hpp
 *
 * @brief Various macros useful for development.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_UTILITY_MACROS_HPP_
#define ECL_CONFIG_UTILITY_MACROS_HPP_

/*****************************************************************************
** Include
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Macros
*****************************************************************************/
/**
 * @addtogroup Macros
 * @{
**/

/**
 * @brief Prevents a function from being inlined.
 *
 * The following macro prevents functions from being inlined by both
 * gnu or msvc compilers.
 *
 * @code
 * ELC_DONT_INLINE void f() { std::cout << "Dude" << std::endl; }
 * @endcode
**/
#if (defined __GNUC__)
  #define ECL_DONT_INLINE __attribute__((noinline))
#elif (defined _MSC_VER)
  #define ECL_DONT_INLINE __declspec(noinline)
#else
  #define ECL_DONT_INLINE
#endif

/**
 * @brief C++11 support is available.
 *
 * This is a very dodgy way of checking - haven't tested across compilers
 * very well, and it doesn't take into account the partial completeness
 * of the implementations.
 *
 * It does however, provide a way for headers to use c++11 symbols
 * in a non-destructive way for users (we often define typedefs for
 * shared ptr variations).
 */
#if defined(__cplusplus) && (__cplusplus >= 201103L)
    #define ECL_CXX11_FOUND
#else
    #define ECL_CXX11_NOT_FOUND
#endif

/*****************************************************************************
** Depracated
*****************************************************************************/

/**
 * @brief Deprecated compiler warnings.
 *
 * The following macro emits deprecated compiler warnings by both
 * gnu or msvc compilers.
 *
 * @code
 * ELC_DEPRECATED void f() { std::cout << "Dude" << std::endl; }
 * @endcode
**/
#if (defined __GNUC__)
  #define ECL_DEPRECATED __attribute__((deprecated))
#elif (defined _MSC_VER)
  #define ECL_DEPRECATED __declspec(deprecated)
#elif defined(__clang__)
  #define ECL_DEPRECATED  __attribute__((deprecated("Use of this method is deprecated")))
#else
  #define ECL_DEPRECATED
#endif

/**
 * @}
 **/

/**
 * @def ECL_HELPER_LOCAL
 * @sa ECL_HELPER_EXPORT, ECL_HELPER_IMPORT
**/
/**
 * @def ECL_HELPER_IMPORT
 * @sa ECL_HELPER_EXPORT, ECL_HELPER_LOCAL
**/
/**
 * @def ECL_HELPER_EXPORT
 *
 * @brief Declare public visibility for libraries.
 *
 * This emits (in a cross platform way) the required symbols
 * for setting whether a function/class should be visible or
 * hidden. Note only does this protect the private parts
 * of your library, but it can also greatly speed up the
 * linking process.
 *
 * - http://gcc.gnu.org/wiki/Visibility
 *
 * To check what symbols are being exported for a gnu library,
 *
 * @code
 * nm -C -D <library>.so
 * @endcode
 *
 * Usage:
 *
 * Each package needs to create it's own macro to make use of
 * the macro that cmake defines for library targets:
 *
 * @code
 * #ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
 *   #ifdef ecl_common_EXPORTS // we are building a shared lib/dll
 *     #define ecl_common_PUBLIC ROS_HELPER_EXPORT
 *   #else // we are using shared lib/dll
 *     #define ecl_common_PUBLIC ROS_HELPER_IMPORT
 *   #endif
 * #else // ros is being built around static libraries
 *   #define ecl_common_DECL
 * #endif
 * @endcode
 *
 * And use it alongside class or function definitions:
 *
 * @code
 * extern "C" ecl_common_PUBLIC void function(int a);
 * class ecl_common_PUBLIC SomeClass {
 *     int c;
 *     ecl_common_LOCAL void privateMethod();  // Only for use within this DSO
 * public:
 *     Person(int _c) : c(_c) { }
 *     static void foo(int a);
 * };
 * @endcode
 *
 * @sa ECL_HELPER_IMPORT, ECL_HELPER_LOCAL
**/

#if defined(ECL_IS_WIN32) || defined(ECL_IS_CYGWIN)
  #define ECL_HELPER_IMPORT __declspec(dllimport)
  #define ECL_HELPER_EXPORT __declspec(dllexport)
  #define ECL_HELPER_LOCAL
#else
  #if defined(ECL_IS_POSIX) && __GNUC__ >= 4
    #define ECL_HELPER_IMPORT __attribute__ ((visibility("default")))
    #define ECL_HELPER_EXPORT __attribute__ ((visibility("default")))
    #define ECL_HELPER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ECL_HELPER_IMPORT
    #define ECL_HELPER_EXPORT
    #define ECL_HELPER_LOCAL
  #endif
#endif

// Until we depracate this entirely, need this to avoid being interpreted as a variable
#define ECL_PUBLIC
#define ECL_LOCAL


#endif /* ECL_UTILITY_CONFIG_MACROS_HPP_ */
