/**
 * @file /yocs_math_toolkit/include/yocs_math_toolkit/sophus/interpolators.hpp
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yocs_math_toolkit_SOPHUS_INTERPOLATERS_HPP_
#define yocs_math_toolkit_SOPHUS_INTERPOLATERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/linear_algebra.hpp>
#include <iomanip>
#include <iostream>
#include <sophus/se3.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>
#include <string>

#include "formatters.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace Sophus {

/*****************************************************************************
** Interfaces
*****************************************************************************/


/**
 * @brief Interpolate between two lie group objects.
 *
 * Working with Sophus::SE3f classes, but haven't tested with the other
 * group types yet.
 *
 * Refer to Section 7 in http://ethaneade.com/lie.pdf.
 *
 * An alternative (slow/fast?) way of returning the result is:
 *
 * @code
 * Group::exp(t*tangent_a + (1.0-t)*tangent_b);
 * @endcode
 */
template <typename Group>
class Interpolator {
public:
  Interpolator(const Group& T_a, const Group& T_b)
  : T_a(T_a)
  {
    Group T_b_rel_a = T_b*T_a.inverse();
    tangent = Group::log(T_b_rel_a);
  }
  Group operator()(const double& t) {
    return Group::exp(t*tangent)*T_a;
  }
private:
  Group T_a;
  typename Group::Tangent tangent;
};

/**
 * A special case of the interpolator that takes two coplanar frames and
 * interpolates between them.
 *
 * The plane is defined by the first frame and the T_b_rel_a must be able
 * to be represented by a SE2 transformation (no z-translation and yaw rotation only).
 *
 * This is oft used when dealing with SE3 frames fixed on the z plane.
 */
class PlanarInterpolator {
public:
  PlanarInterpolator(const Sophus::SE3f& T_a, const Sophus::SE3f& T_b) throw(ecl::StandardException);
  Sophus::SE3f operator()(const double& t);

private:
  Sophus::SE3f T_a;
  Sophus::SE2f t_a;
  Sophus::SE2f::Tangent tangent;
};

/**
 * A special case of the interpolator that slides along the straight line
 * connecting two frames whilst interpolating the orientation between the two.
 *
 * Normal se3 interpolation will not guarantee a straight line connection
 * if there is a rotation involved. i.e. it will swing its hips like it is
 * dancing!
 */
class SlidingInterpolator {
public:
  SlidingInterpolator(const Sophus::SE3f& T_a, const Sophus::SE3f& T_b);
  Sophus::SE3f operator()(const double& t);

private:
  Interpolator<Sophus::SE3f> interpolator;
  Sophus::SE3f T_a, T_b;
  Sophus::SE3f::Tangent tangent;
};


} // namespace Sophus

#endif /* yocs_math_toolkit_SOPHUS_INTERPOLATERS_HPP_ */
