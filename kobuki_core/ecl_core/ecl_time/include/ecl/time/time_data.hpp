/**
 * @file /ecl_time/include/ecl/time/time_data.hpp
 *
 * @brief Device for conveniently storing and analysing benchmarking times.
 *
 * @date September 2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ELC_TIME_TIME_DATA_HPP_
#define ELC_TIME_TIME_DATA_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <vector>
#include <ecl/config/macros.hpp>
#include <ecl/time/duration.hpp>
#include "macros.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [TimeData]
*****************************************************************************/
/**
 * @brief Device for conveniently storing and analysing benchmarking times.
 *
 * This serves as both a container and interface for generating statistics on
 * a sequence of timings.
 */
class ecl_time_PUBLIC TimeData {
public:
	/*********************
	** Collection
	**********************/
	/**
	 * @brief Append a new timing measurement to the sequence.
	 *
	 * @param duration : measurement to be added.
	 */
	void push_back(const ecl::Duration& duration ) { durations.push_back(duration); }
	/**
	 * @brief Reset the container (i.e. clear it).
	 */
	void clear() { durations.clear(); }

	/*********************
	** Statistics
	**********************/
	/**
	 * @brief Return the average of the elements currently stored.
	 * @return Duration : average.
	 */
	ecl::Duration average() const;
	/**
	 * @brief Return the standard deviation of the elements currently stored.
	 * @return Duration : standard deviation.
	 */
	ecl::Duration stdDev() const;
	/**
	 * @brief Return the variance of the elements currently stored.
	 * @return Duration : variance.
	 */
	ecl::Duration variance() const;

private:
	std::vector<ecl::Duration> durations;
};

} // namespace ecl

#endif /* ELC_TIME_TIME_DATA_HPP_ */
