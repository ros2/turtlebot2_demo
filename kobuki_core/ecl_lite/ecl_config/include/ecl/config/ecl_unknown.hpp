/**
 * @file /ecl_config/include/ecl/config/ecl_unknown.hpp
 *
 * @brief Guesswork to determine parameters from macros only.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONFIG_UNKNOWN_HPP_
#define ECL_CONFIG_UNKNOWN_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <limits>
#include <climits>

/*****************************************************************************
** Integers
*****************************************************************************/

/*
 * The c99 standard will only define certain macros for c++
 * if explicitly requested.
 */
#define __STDC_LIMIT_MACROS

#if defined(__GNUC__)
	/*
	 * Note that cstdint is not enabled by default by gcc yet (v4.4). It will
	 * give a compile time error unless you explicitly pass -std=c++0x to gcc.
	 */
	#if __GXX_EXPERIMENTAL_CXX0X__ > 0
		#include <cstdint>
	#else
		#include <stdint.h>
	#endif
#elif (defined(__STDC__) && __STDC__)
	#include <stdint.h>
#else
	// nothing here yet - look in graveyard/include/portable_integers for some ideas.
#endif

/*
 * These are very naive - update these as we come across different platforms.
 */
#if CHAR_MAX == INT8_MAX
	#define ECL_SIZE_OF_CHAR 1
#else
	#error "ECL_SIZE_OF_CHAR could not be determined (probably faulty ecl logic)."
#endif
#if SHRT_MAX == INT16_MAX
	#define ECL_SIZE_OF_SHORT 2
#else
	#error "ECL_SIZE_OF_SHORT could not be determined (probably faulty ecl logic)."
#endif
#if INT_MAX == INT32_MAX
	#define ECL_SIZE_OF_INT 4
#else
	#error "ECL_SIZE_OF_INT could not be determined (probably faulty ecl logic)."
#endif
#if LONG_MAX == INT32_MAX
	#define ECL_SIZE_OF_LONG 4
#elif LONG_MAX == INT64_MAX
	#define ECL_SIZE_OF_LONG 8
#else
	#error "ECL_SIZE_OF_LONG could not be determined (probably faulty ecl logic)."
#endif

#if LONG_LONG_MAX == INT64_MAX
	#define ECL_SIZE_OF_LONG_LONG 8
#else
	#error "ECL_SIZE_OF_LONG_LONG could not be determined (probably faulty ecl logic)."
#endif

/*****************************************************************************
** Floats
*****************************************************************************/
/*
 * These may be somewhat naively assumed. I should  hand craftfor
 * particular platforms in the future if it ever really
 * differs.
 */
#ifdef __GNUC__
	#ifdef __SIZEOF_FLOAT__
		#define ECL_SIZE_OF_FLOAT __SIZEOF_FLOAT__
	#else
		#define ECL_SIZE_OF_FLOAT 4
	#endif
	#ifdef __SIZEOF_DOUBLE__
		#define ECL_SIZE_OF_DOUBLE __SIZEOF_DOUBLE__
	#else
		#define ECL_SIZE_OF_DOUBLE 8
	#endif
	#ifdef __SIZEOF_LONG_DOUBLE__
		#define ECL_SIZE_OF_LONG_DOUBLE __SIZEOF_LONG_DOUBLE__
	#else
		#define ECL_SIZE_OF_LONG_DOUBLE 16
	#endif
#else // Non Gnu platforms
	#define ECL_SIZE_OF_FLOAT 4
	#define ECL_SIZE_OF_DOUBLE 8
	#define ECL_SIZE_OF_LONG_DOUBLE 16 // This is often 12!
#endif

#endif /* ECL_CONFIG_UNKNOWN_HPP_ */
