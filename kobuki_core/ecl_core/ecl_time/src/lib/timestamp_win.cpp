/**
 * @file /ecl_time/src/lib/timestamp_win.cpp
 *
 * @brief Windows implementation of the timestamp class.
 *
 * @date May 22, 2010
 **/
/*****************************************************************************
** Platform Check
*****************************************************************************/

#include <ecl/config.hpp>
#if defined(ECL_IS_WIN32)

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/windows.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include "../../include/ecl/time/timestamp_win.hpp"
//#include "../../include/ecl/time/detail/time_functions_win.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [Constructors]
*****************************************************************************/

TimeStamp::TimeStamp() ecl_debug_throw_decl(StandardException) {
	stamp();
}

TimeStamp::TimeStamp (const double& decimal_time_value) ecl_assert_throw_decl(StandardException) :
	TimeStampBase(decimal_time_value)
{
}

TimeStamp::TimeStamp (const time_t& seconds, const long& nanoseconds) ecl_assert_throw_decl(StandardException) :
	TimeStampBase(seconds, nanoseconds)
{
}

TimeStamp::TimeStamp(const TimeStampBase& base) : TimeStampBase(base) {}

/*****************************************************************************
** Implementation [Stamps]
*****************************************************************************/

const TimeStamp& TimeStamp::stamp() ecl_debug_throw_decl(StandardException) {
    return (*this);
}

}; // namespace ecl

#endif /* ECL_IS_WIN32 */
