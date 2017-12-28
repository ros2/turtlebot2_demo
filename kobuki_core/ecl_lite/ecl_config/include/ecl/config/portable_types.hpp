/**
 * @file /ecl_config/include/ecl/config/portable_types.hpp
 *
 * @brief Type definitions for integer types.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_PORTABLE_TYPES_HPP_
#define ECL_PORTABLE_TYPES_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/ecl.hpp>
#include <limits>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Integers
*****************************************************************************/
// If errors show up, need to check the ecl macro logic.
#if ECL_SIZE_OF_CHAR == 1
	typedef char int8;		/**< @brief An alias for the platform's 8 bit integer type. **/
	typedef unsigned char uint8;		/**< @brief An alias for the platform's unsigned 8 bit integer type. **/
#else
	#error "ecl::int8 could not be typedef'd (probably faulty ecl logic)."
#endif

#if ECL_SIZE_OF_SHORT == 2
	typedef short int16;		/**< @brief An alias for the platform's 16 bit integer type. **/
	typedef unsigned short uint16;		/**< @brief An alias for the platform's unsigned 16 bit integer type. **/
#else
	#error "ecl::int16 could not be typedef'd (probably faulty ecl logic)."
#endif

#if ECL_SIZE_OF_INT == 4
	typedef int int32;		/**< @brief An alias for the platform's 32 bit integer type. **/
	typedef unsigned int uint32;		/**< @brief An alias for the platform's unsigned 32 bit integer type. **/
#else
	#error "ecl::int32 could not be typedef'd (probably faulty ecl logic)."
#endif

#if ECL_SIZE_OF_LONG == 4
	typedef long long int64;		/**< @brief An alias for the platform's 64 bit integer type. **/
	typedef unsigned long long uint64;		/**< @brief An alias for the platform's unsigned 64 bit integer type. **/
#elif ECL_SIZE_OF_LONG == 8
	typedef long int64;		/**< @brief An alias for the platform's 64 bit integer type. **/
	typedef unsigned long uint64;		/**< @brief An alias for the platform's unsigned 64 bit integer type. **/
#else
	#error "ecl::int64 could not be typedef'd (probably faulty ecl logic)."
#endif

/*****************************************************************************
** Floats
*****************************************************************************/

#if ECL_SIZE_OF_FLOAT == 4
	typedef float float32; /**< @brief An alias for the platform's 32 bit float type. **/
#else
	#error "ecl::float32 could not be typedef'd (probably faulty ecl logic)."
#endif
#if ECL_SIZE_OF_DOUBLE == 8
	typedef double float64; /**< @brief An alias for the platform's 64 bit float type. **/
#elif ECL_SIZE_OF_LONG_DOUBLE == 8
	typedef long double float64; /**< @brief An alias for the platform's 64 bit float type. **/
#else
	#error "ecl::float64 could not be typedef'd (probably faulty ecl logic)."
#endif
#if ECL_SIZE_OF_LONG_DOUBLE == 12
	typedef long double float96; /**< @brief An alias for the platform's 96 bit float type. **/
#elif ECL_SIZE_OF_LONG_DOUBLE == 16
	typedef long double float128; /**< @brief An alias for the platform's 128 bit float type. **/
#else
	#define ECL_LONG_DOUBLE_UNDEFINED
	#define ECL_LONG_LONG_DOUBLE_UNDEFINED
	//#error "ecl::float96 could not be typedef'd (probably faulty ecl logic)."
#endif

} // namespace ecl

#endif /* ECL_PORTABLE_TYPES_HPP_ */
