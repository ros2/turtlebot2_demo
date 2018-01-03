/**
 * @file /ecl_geometry/src/lib/linear_segment.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/geometry/linear_segment.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

LinearSegment::LinearSegment(const double& x_1,
                               const double& y_1,
                               const double& x_2,
                               const double& y_2)
: x_1(x_1), y_1(y_1)
, x_2(x_2), y_2(y_2)
{
  if ( x_2 == x_1 ) {
    B = 0; A = 1; C = x_1;
  } else {
    B = 1;
    A = -1*(y_2-y_1)/(x_2-x_1);
    C = -y_1 - x_1*A;
  }
}

double LinearSegment::squaredDistanceFromPoint(const double& x, const double& y) const
{
  // pp1 = [ x - x1 ]     p2p1 = [ x2 - x1 ]
  //       [ y - y1 ]            [ y2 - y1 ]
  // The dot product of the above two vectors will give a hint as to where
  // the point lies in relation to the segment
  double dot = (x-x_1)*(x_2-x_1) + (y-y_1)*(y_2-y_1);
  double squared_length_p2p1 = (x_2-x_1)*(x_2-x_1) + (y_2-y_1)*(y_2-y_1);
  double t = -1;
  if ( squared_length_p2p1 != 0 ) { // special case handling
    t = dot / squared_length_p2p1;
  }
  double xx, yy;
  if ( t < 0 ) {
    xx = x_1; yy = y_1;
  } else if ( t > 1 ) {
    xx = x_2; yy = y_2;
  } else {
    xx = x_1 + t*(x_2-x_1);
    yy = y_1 + t*(y_2-y_1);
  }
  return (x - xx)*(x - xx) + (y - yy)*(y - yy);
}

/*****************************************************************************
 ** Trailers
 *****************************************************************************/

} // namespace ecl
