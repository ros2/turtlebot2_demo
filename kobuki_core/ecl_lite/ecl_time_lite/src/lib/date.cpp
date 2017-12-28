/**
 * @file /src/lib/date.cpp
 *
 * @brief Implementation of date manipulators.
 **/


/*****************************************************************************
** Cross Platform Configuration
*****************************************************************************/

#include <ecl/time_lite/config.hpp>

#if defined(ECL_HAS_POSIX_TIMERS) || defined(ECL_HAS_RT_TIMERS) || defined(ECL_HAS_MACH_TIMERS)

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/time_lite/date.hpp"
#include <ctime>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {


std::string get_date_string() {
  time_t now;
  char the_date[12];

  the_date[0] = '\0';
  now = time(NULL);
  if (now != -1)
  {
    // this makes use of the variable LC_TIME
    // it also gives me incorrect time on my machine, wrong time zone?
    strftime(the_date, 17, "%Y%m%d-%H%M%S", gmtime(&now));
  }
  return std::string(the_date);
}

} // namespace ecl

#endif // POSIX_TIMERS || RT_TIMERS
