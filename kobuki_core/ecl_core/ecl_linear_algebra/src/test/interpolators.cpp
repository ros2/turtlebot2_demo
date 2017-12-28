/**
 * @file src/test/interpolators.cpp
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <ecl/math.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include "../../include/ecl/linear_algebra/sophus.hpp"

/*****************************************************************************
 ** Methods
 *****************************************************************************/

double getAngle(const Sophus::SE3f& T) {
  Eigen::Matrix<float,3,1> axis_angle = T.inverse().so3().log();
  // our tests are doing yaws only
  return axis_angle(2);
}

/*****************************************************************************
** Tests
*****************************************************************************/

TEST(Interpolation, Interpolation) {
  Sophus::SE3fFormatter formatter(6,2);
  /********************
   ** Frame Poses
   ********************/
  Eigen::Matrix3f rotation_matrix_start = Eigen::AngleAxis<float> (1*ecl::pi_2, Eigen::Vector3f::UnitZ ()).matrix();
  Sophus::SE3f start = Sophus::SE3f(rotation_matrix_start, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f rotation_matrix_finish = Eigen::AngleAxis<float> (1*ecl::pi, Eigen::Vector3f::UnitZ ()).matrix();
  Sophus::SE3f finish = Sophus::SE3f(rotation_matrix_finish, Eigen::Vector3f::UnitX());
  std::cout << "Start  : " << formatter(start) << std::endl;
  std::cout << "Finish : " << formatter(finish) << std::endl;
  /********************
   ** Transforms
   ********************/
  Sophus::SE3f T_start_rel_world = start.inverse();
  Sophus::SE3f T_finish_rel_world = finish.inverse();
  /********************
   ** Interpolation
   ********************/
  //Sophus::Interpolator<Sophus::SE3f> interpolator(T_start_rel_world, T_finish_rel_world);
  Sophus::SlidingInterpolator interpolator(T_start_rel_world, T_finish_rel_world);
  for ( unsigned int i = 0; i <= 10; ++i ) {
    double t = i*0.1;
    Sophus::SE3f T_t_rel_world = interpolator(t);
    //EXPECT_NEAR(getAngle(T_t_rel_world), i*0.15708, 0.001);
    std::cout << formatter(T_t_rel_world.inverse()) << " [t:" << t << "][axis_angle:" << getAngle(T_t_rel_world) << "]" <<  std::endl;
  }
}

TEST(Interpolation, TwoDimensionalInterpolation) {
  Sophus::SE3fFormatter formatter(6,2);
  /********************
   ** Frame Poses
   ********************/
  Sophus::SE3f start = Sophus::SE3f(Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero());
  Eigen::Matrix3f rotation_matrix = Eigen::AngleAxis<float> (-1*ecl::pi_2, Eigen::Vector3f::UnitZ ()).matrix();
  Sophus::SE3f finish = Sophus::SE3f(rotation_matrix, 10*Eigen::Vector3f::UnitX());
  std::cout << "Start  : " << formatter(start) << std::endl;
  std::cout << "Finish : " << formatter(finish) << " [axis_angle:" << getAngle(finish.inverse()) << "]" << std::endl;
  /********************
   ** Transforms
   ********************/
  Sophus::SE3f T_start_rel_world = start.inverse();
  Sophus::SE3f T_finish_rel_world = finish.inverse();
  /********************
   ** Interpolation
   ********************/
  Sophus::PlanarInterpolator interpolator(T_start_rel_world, T_finish_rel_world);
  for ( unsigned int i = 0; i <= 10; ++i ) {
    double t = i*0.1;
    Sophus::SE3f T_t_rel_world = interpolator(t);
    EXPECT_NEAR(getAngle(T_t_rel_world), -1.0*i*0.15708, 0.001);
    std::cout << formatter(T_t_rel_world.inverse()) << " [t:" << t << "][axis_angle:" << getAngle(T_t_rel_world) << "]" <<  std::endl;
  }
}

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}



