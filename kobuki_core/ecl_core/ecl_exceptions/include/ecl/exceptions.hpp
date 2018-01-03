/**
 * @file /include/ecl/exceptions.hpp
 *
 * @brief Collects headers for ecl_exceptions.
 *
 * @date Feb 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_EXCEPTIONS_HPP_
#define ECL_EXCEPTIONS_HPP_

/*****************************************************************************
** Platform Detection
*****************************************************************************/

#include <ecl/config/ecl.hpp>

/*****************************************************************************
** Includes
*****************************************************************************/

#include "exceptions/macros.hpp"
#include "exceptions/exception.hpp"
#include "exceptions/standard_exception.hpp"
#include "exceptions/data_exception.hpp"
#if defined(ECL_IS_POSIX)
  #include "exceptions/posix_error_handler.hpp"
#endif

#endif /* ECL_EXCEPTIONS_HPP_ */
