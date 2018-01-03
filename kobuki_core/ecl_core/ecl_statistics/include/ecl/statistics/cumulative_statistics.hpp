/**
 * @file /ecl_statistics/include/ecl/statistics/cumulative_statistics.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_statistics_CUMULATIVE_STATISTICS_HPP_
#define ecl_statistics_CUMULATIVE_STATISTICS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/mpl.hpp>
#include <ecl/type_traits.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief Dummy parent class for Homogenous points.
 *
 * SFINAE trick to ensure that only floats are used for the template parameter.
 * The real class is in the specialisation.
 */
template <typename T, typename Enable = void>
class CumulativeStatistics : public ecl::FailedObject {};

/**
 * Calculate cumulative mean and variance (i.e. without storage).
 */
template <typename T>
class CumulativeStatistics<T, typename ecl::enable_if< ecl::is_float<T> >::type> {
public:
  CumulativeStatistics(): number_of_data(0.0) {}

  void clear() { number_of_data = 0.0; }

  /**
   * Catch the new data and update the cumulative calculations.
   *
   * TODO it uses a float as a counter to increase the longevity of the cumulative calculation,
   * however it really ought to handle the number_of_data rolling over problem.
   *
   * @param x : new data
   */
  void push_back(const T & x)
  {
    number_of_data++;
    if(number_of_data == 1.0)
    {
      old_mean = new_mean = x;
      old_variance = 0.0;
    }
    else
    {
      new_mean = old_mean + static_cast<T>(x - old_mean) / number_of_data;
      new_variance = old_variance + static_cast<T>(x - old_mean) * static_cast<T>(x - new_mean);

      old_mean = new_mean;
      old_variance = new_variance;
    }
  }

  /**
   * Number of data used for statistics
   * @return T
   */
  T size() const { return number_of_data; }

  /**
   * @brief Current cumulative calculation of mean.
   * @return T or 0.0 if there is not yet data.
   */
  T mean() const { return (number_of_data > 0) ? new_mean : 0.0; }

  /**
   * @brief Current cumulative calculation of variance.
   * @return T or 0.0 if there is not yet data.
   */
  T variance() const { return ((number_of_data > 1) ? new_variance / (number_of_data - 1) : 0.0); }

private:
  T number_of_data;
  T old_mean, old_variance;
  T new_mean, new_variance;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace ecl

#endif /* ecl_statistics_CUMULATIVE_STATISTICS_HPP_ */
