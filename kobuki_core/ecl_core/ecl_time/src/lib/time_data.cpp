/**
 * @file /ecl_time/src/lib/time_data.cpp
 *
 * @brief Implementation of the time data interface.
 *
 * @date September 2010
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include "../../include/ecl/time/time_data.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation [TimeData]
*****************************************************************************/

ecl::Duration TimeData::average() const {
	double sum = 0.0;
	for ( unsigned int i = 0; i < durations.size(); ++i ) {
		sum += durations[i];
	}
	return ecl::Duration(sum/static_cast<double>(durations.size()));
};

ecl::Duration TimeData::stdDev() const {
	ecl::Duration avg = average();
	double sum = 0.0;
	for ( unsigned int i = 0; i < durations.size(); ++i ) {
		sum += (static_cast<double>(durations[i]) - static_cast<double>(avg))*
				(static_cast<double>(durations[i]) - static_cast<double>(avg));
	}
	return ecl::Duration(sqrt(sum/static_cast<double>(durations.size())));
};

ecl::Duration TimeData::variance() const {
	ecl::Duration avg = average();
	double sum = 0.0;
	for ( unsigned int i = 0; i < durations.size(); ++i ) {
		sum += (static_cast<double>(durations[i]) - static_cast<double>(avg))*
				(static_cast<double>(durations[i]) - static_cast<double>(avg));
	}
	return ecl::Duration(sum/static_cast<double>(durations.size()));
};

} // namespace ecl

