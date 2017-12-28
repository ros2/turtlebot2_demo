/**
 * @file /ecl_time/include/ecl/time/frequency_diagnostics.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_time_TIME_FREQUENCY_DIAGNOSTICS_HPP_
#define ecl_time_TIME_FREQUENCY_DIAGNOSTICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "timestamp.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/
/**
 * @brief Data relevant to frequencies of an incoming stream.
 *
 * Typical use case is for characterising an incoming stream of sensor data.
 */
struct FrequencyDiagnostics {

  FrequencyDiagnostics()
  : has_connection(false)
  , hz(-1.0)
  , std_dev(-1.0)
  , minimum_interval(-1.0)
  , maximum_interval(-1.0)
  , last_incoming()
  {}

  bool has_connection;
  float hz;
  float std_dev;
  float minimum_interval;
  float maximum_interval;
  float last_incoming;

};

/**
 * @brief Lightweight and fast frequency monitor.
 *
 * This class doesn't dynamically update over a window, nor does it collect
 * and store data to compute characteristics like standard deviations. It is
 * however, suitable for most use case scenarios where you are simply
 * interested in the hz/minimum_interval/maximum_interval of data over
 * a fixed period.
 *
 * <b>Usage:</b>
 *
 *
 * @code
 * float window_period = 2.0;
 * FrequencyMonitor monitor(window_period, false);
 * FrequencyMonitor ros_monitor(window_period, true);  // use the realtime clock to match ros timestamps
 * FrequencyDiagnostics diagnostics;
 *
 * void incomingDataCallback() {
 *   monitor.update();  // register that new data has arrived
 * }
 *
 * void process() {
 *   diagnostics = monitor.analyse();
 * }
 *
 * void process2() {
 *   monitor.analyse();
 *   diagnostics = monitor.getDiagnostics();
 * }
 * @endcode
 */
class FrequencyMonitor {
public:
  /**
   * @brief Setup the frequency monitor.
   *
   * @param window_period : time interval over which updates are analysed.
   * @param use_realtime_clock : better to use the monotonic clock, but sometimes (e.g. ros), realtime is required.
   */
  FrequencyMonitor(
      const float& window_period = 2.0,
      const bool& use_realtime_clock=false
      );
  /**
   * @brief Let the diagnostics know that new data has arrived.
   *
   * This updates the internal counter, with a timestamp.
   */
  void update();
  /**
   * @brief Analyse the recent updates to generate monitoring statistics.
   *
   * Right now this just uses a fixed window (not moving) that only updates the information if
   * the period since the last analyse() request was made has been exceeded.
   */
  const FrequencyDiagnostics& analyse();

  /**
   * @brief Diagnostics getter function.
   * @return FrequencyDiagnostics
   */
  const FrequencyDiagnostics& diagnostics() const { return current_diagnostics; };

protected:
  FrequencyDiagnostics current_diagnostics;
  unsigned int incoming_counter;
  ecl::TimeStamp last_anyalsed;
  ecl::TimeStamp period;
  ecl::TimeStamp minimum_interval;
  ecl::TimeStamp maximum_interval;
  bool use_realtime_clock;
  ecl::TimeStamp last_incoming;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace ecl

#endif /* ecl_time_TIME_FREQUENCY_DIAGNOSTICS_HPP_ */
