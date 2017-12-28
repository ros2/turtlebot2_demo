/**
 * @file /include/ecl/exceptions/macros.hpp
 *
 * @brief Macros for ecl exception handling.
 *
 * @date March, 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_MACROS_HPP_
#define ECL_EXCEPTIONS_MACROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Declspecs
*****************************************************************************/

/*
 * Import/exports symbols for the library
 */
#ifdef ECL_HAS_SHARED_LIBS // ecl is being built around shared libraries
  #ifdef ecl_exceptions_EXPORTS // we are building a shared lib/dll
    #define ecl_exceptions_PUBLIC ECL_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define ecl_exceptions_PUBLIC ECL_HELPER_IMPORT
  #endif
  #define ecl_exceptions_LOCAL ECL_HELPERS_LOCAL
#else // ecl is being built around static libraries
  #define ecl_exceptions_PUBLIC
  #define ecl_exceptions_LOCAL
#endif

/*****************************************************************************
** Macros [ecl_throw]
*****************************************************************************/

#if defined(ECL_DISABLE_EXCEPTIONS)
  #define ecl_throw_decl(exception)
  #define ecl_throw(exception) ((void)0)
  #define ecl_try if(true)
  #define ecl_catch(exception) else
#else
/**
 * @addtogroup Macros
 * @{
**/
/**
 * @brief Standard ecl throw exception declaration.
 *
 * Use when declaring a function that throws a normal exception. If
 * ECL_DISABLE_EXCEPTIONS is not defined, this simply inserts the required
 * exception statement, otherwise it inserts nothing.
 *
 * @sa @ref ecl_debug_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_throw_decl(exception) throw(exception)
/**
 * @brief Standard ecl throw exception throw.
 *
 * Use when throwing a function that throws a normal exception. If
 * ECL_DISABLE_EXCEPTIONS is not defined, this simply inserts the required
 * exception constructor otherwise it inserts nothing.
 *
 * @sa @ref ecl_throw_decl, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_throw(exception) throw exception;
/**
 * @brief The try part of a try-catch macro matching ecl_throw calls.
 *
 * @sa @ref ecl_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_try try
/**
 * @brief The catch part of a try-catch macro matching ecl_throw calls.
 *
 * @sa @ref ecl_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_catch(exception) catch(exception)
/**
 * @}
 **/
#endif

/*****************************************************************************
** Macros [ecl_assert_throw, ecl_debug_throw]
*****************************************************************************/
/* Some bad logic here - release can still mean we use some exceptions */
#if defined(NDEBUG) || defined(ECL_NDEBUG) || defined(ECL_DISABLE_EXCEPTIONS)
  #define ecl_assert_throw_decl(exception)
  #define ecl_assert_throw(expression,exception) ((void)0)
  #define ecl_debug_throw_decl(exception)
  #define ecl_debug_throw(exception) ((void)0)
  #define ecl_debug_try if(true)
  #define ecl_debug_catch(exception) else // TODO - this is useles, you can't do ecl_debug_catch(Exception &e) { std::cout << e.what() << std::endl; } in release mode
#else
  #define ECL_HAS_EXCEPTIONS
/**
 * @addtogroup Macros
 * @{
**/
/**
 * @brief Assure throw exception declaration.
 *
 * Use when declaring a function that throws the assert mode exceptions. If NDEBUG
 * is not defined, this simply inserts the required exception statement, otherwise it
 * inserts nothing. This is exactly the same as the @ref ecl_debug_throw_decl, but added
 * here for convenience as its confusing to have to use the ecl_debug_throw_decl
 * when also using ecl_assert_throw.
 *
 * @sa @ref ecl_assert_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_assert_throw_decl(exception) throw(exception)
/**
 * @brief Debug mode throw with a logical condition check.
 *
 * Does a simple logical check before throwing, only works if NDEBUG is not
 * defined.
 *
 * @sa @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_assert_throw(expression,exception) if ( !(expression) ) { throw exception; }
/**
 * @brief Debug mode throw exception declaration.
 *
 * Use when declaring a function that throws the debug mode exceptions. If NDEBUG
 * is not defined, this simply inserts the required exception statement, otherwise it
 * inserts nothing.
 *
 * @sa @ref ecl_debug_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_debug_throw_decl(exception) throw(exception)
/**
 * @brief Debug mode exception throw.
 *
 * Use when throwing a debug mode only (NDEBUG is not defined) exception. This
 * simply inserts the required exception constructor, otherwise it
 * inserts nothing.
 *
 * @sa @ref ecl_debug_throw_decl, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_debug_throw(exception) throw exception;
/**
 * @brief The try part of a try-catch macro matching ecl_debug_throw/ecl_assert_throw calls.
 *
 * @sa @ref ecl_assert_throw, ecl_debug_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_debug_try try
/**
 * @brief The catch part of a try-catch macro matching ecl_debug_throw/ecl_assert_throw calls.
 *
 * @sa @ref ecl_assert_throw, ecl_debug_throw, @ref errorsExceptions "Exceptions Guide".
 */
  #define ecl_debug_catch(exception) catch(exception)
#endif
/**
 * @}
 **/

#endif /* ECL_EXCEPTIONS_MACROS_HPP_ */
