/**
 * @file /yocs_math_toolkit/include/yocs_math_toolkit/sophus_helpers.hpp
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef yocs_math_toolkit_SOPHUS_HELPERS_HPP_
#define yocs_math_toolkit_SOPHUS_HELPERS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/config/macros.hpp>
#include <ecl/converters.hpp>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/linear_algebra.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#if defined(ECL_CXX11_FOUND)
    #include <memory>
#endif
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
 ** C++11 Api Only
 *****************************************************************************/

#if defined(ECL_CXX11_FOUND)
    typedef std::shared_ptr<SE3f> SE3fPtr;

    /// Converts a line drawn between two points on the z-plane into the transform of a sophus frame relative to the origin.
    Sophus::SE3fPtr points2DToSophusTransform(float from_x, float from_y, float to_x, float to_y);
#endif

/*****************************************************************************
** Interfaces
*****************************************************************************/

template<typename T>
std::ostream & operator << ( std::ostream & out, const SO3<T> & se3 )
{
//  typename SE3Group<T>::Tangent tanget_vector = SE3Group<T>::log( se3 );
//  out << tanget_vector.transpose();
  const Eigen::Matrix<T,3,1> & t = se3.translation();
  const Eigen::Quaternion<T> & q = se3.unit_quaternion();
  out << t.transpose() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
  return out;
}

template<typename T>
std::ostream & operator << ( std::ostream & out, const SO2<T> & se2 )
{
  typename SO2<T>::Tangent tanget_vector = SO2<T>::log( se2 );
  out << tanget_vector.transpose();
  return out;
}

/**
 * @brief Convert a full Sophus pose into a 2 dimensional pose.
 *
 * The 2d pose is a typical mobile robot 2d pose with (x, y, heading) with
 * heading measured in radians.
 **/
Eigen::Vector3f toPose2D(const Sophus::SE3f& pose);
/**
 * @brief Convert a 2 dimensional pose to a full Sophus pose in 3d.
 *
 * The 2d pose is a typical mobile robot 2d pose with (x, y, heading) with
 * heading measured in radians.
 **/
Sophus::SE3f toPose3D(const Eigen::Vector3f& pose);

class PlanarRotation2Quaternion
{
public:
  PlanarRotation2Quaternion(){}
  Eigen::Quaternionf operator() ( float theta )
  {
    Eigen::Matrix3f R = ( Eigen::AngleAxis<float> (static_cast<float> ( theta ), Eigen::Vector3f::UnitZ ()) ).matrix();
    Eigen::Quaternionf q(R);
    q.normalize();
    return q;
//    Eigen::Quaternionf q;
//    // in this case x and y part is zero since we assumbed that rotationa round z axis on the xy-plane
//    q.vec() << 0, 0, sin(theta*0.5f);
//    q.w() = cos(theta*0.5f);
//    q.normalize();
//    return q;
  }

  void convert( float theta, Eigen::Quaternionf & q )
  {
    q.vec() << 0, 0, sin(theta*0.5f);
    q.w() = cos(theta*0.5f);
  }
};


} // namespace Sophus

/*****************************************************************************
** Converters
*****************************************************************************/

namespace ecl {

/**
 * @brief Converter class representing the Sophus::toPose3D method.
 */
template<>
class Converter<Sophus::SE3f, Eigen::Vector3f> {
public:
  Sophus::SE3f operator()(const Eigen::Vector3f& pose);
};

/**
 * @brief Converter class representing the Sophus::toPose2D method.
 */
template<>
class Converter<Eigen::Vector3f, Sophus::SE3f> {
public:
  Eigen::Vector3f operator()(const Sophus::SE3f& pose);
};

} // namespace ecl


#endif /* yocs_math_toolkit_SOPHUS_HELPERS_HPP_ */
