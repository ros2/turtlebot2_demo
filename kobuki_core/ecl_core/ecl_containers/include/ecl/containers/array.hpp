/**
 * @file /include/ecl/containers/array.hpp
 *
 * @brief Fixed size containers with a few bells and whistles.
 *
 * @date September 2009
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_CONTAINERS_ARRAY_HPP_
#define ECL_CONTAINERS_ARRAY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifdef ECL_MEM_CHECK_ARRAYS
  #include "array/array_mem_check.hpp"
  #include "array/array_dynamic_mem_check.hpp"
#else
  #include "array/array_no_mem_check.hpp"
  #include "array/array_dynamic_no_mem_check.hpp"
#endif
// make sure the utility classes get included.
#include "array/converters.hpp"
#include "array/formatters.hpp"

#endif /* ECL_CONTAINERS_ARRAY_HPP_ */
