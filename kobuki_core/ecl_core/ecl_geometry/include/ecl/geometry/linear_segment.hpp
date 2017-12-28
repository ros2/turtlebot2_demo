/**
 * @file /ecl_geometry/include/ecl/geometry/linear_segment.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ecl_geometry_GEOMETRY_LINEAR_SEGMENT_HPP_
#define ecl_geometry_GEOMETRY_LINEAR_SEGMENT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/utilities/parameter.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class LinearSegment {
public:
  LinearSegment(const double& x_1,
                const double& y_1,
                const double& x_2,
                const double& y_2);

  /**
   * @brief Distance of a point from the segment.
   *
   * Returns the distance from one of the end points if it is closer than
   * the segment itself.
   *
   * TODO shift this to a standard geometry function
   * computing the squared distance between two geometric objects.
   *
   * @param x : x coordinate of the point
   * @param y : y-coordinate of the point
   * @return double
   */
  double squaredDistanceFromPoint(const double& x, const double& y) const;

  ecl::Parameter<double> A, B, C;

private:
  double x_1, y_1;
  double x_2, y_2;
};

/*****************************************************************************
** Trailers
*****************************************************************************/

} // namespace ecl

#endif /* ecl_geometry_GEOMETRY_LINEAR_SEGMENT_HPP_ */
