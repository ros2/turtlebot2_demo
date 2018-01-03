/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /kobuki_dock_drive/include/kobuki_dock_drive/dock_drive.hpp
 *
 * @brief Simple module for the docking drive algorithm
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_DOCK_DRIVE_HPP_
#define KOBUKI_DOCK_DRIVE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/linear_algebra.hpp>

#include "kobuki_dock_drive/state.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki {

/*****************************************************************************
** Interfaces
*****************************************************************************/

class DockDrive {
public:
  DockDrive();
  ~DockDrive();

  bool init(){ return true; }
  bool isEnabled() const { return is_enabled; }
  bool canRun() const { return can_run; }

  void enable() { modeShift("enable"); }
  void disable() { modeShift("disable"); }
  void modeShift(const std::string& mode);

  void update(const std::vector<unsigned char> &signal /* dock_ir signal*/
                , const unsigned char &bumper
                , const unsigned char &charger
                , const ecl::LegacyPose2D<double> &pose);

  void velocityCommands(const double &vx, const double &wz);

  /*********************
  ** Command Accessors
  **********************/
  double getVX() const { return vx; }
  double getWZ() const { return wz; }

  /*********************
  ** Mode Accessors
  **********************/
  RobotDockingState::State getState() const { return state; }
  std::string getStateStr() const { return state_str; }
  std::string getDebugStr() const { return debug_str; }

  /*********************
  ** Parameters Mutators
  **********************/
  void setMinAbsV(double mav) { min_abs_v = mav; }
  void setMinAbsW(double maw) { min_abs_w = maw; }

  //debugging
  std::string getDebugStream() { return debug_output; } //stream.str(); }
  //std::string getDebugStream() { return debug_stream.str(); }
  //std::ostringstream debug_stream;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void processBumpChargeEvent(const unsigned char& bumper, const unsigned char& charger);
  void computePoseUpdate(ecl::LegacyPose2D<double>& pose_update, const ecl::LegacyPose2D<double>& pose);
  void filterIRSensor(std::vector<unsigned char>& signal_filt,const std::vector<unsigned char> &signal );
  void generateDebugMessage(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::LegacyPose2D<double>& pose_update, const std::string& debug_str);
  void updateVelocity(const std::vector<unsigned char>& signal_filt, const ecl::LegacyPose2D<double>& pose_update, std::string& debug_str);
  RobotDockingState::State determineRobotLocation(const std::vector<unsigned char>& signal_filt,const unsigned char& charger);
  bool validateSignal(const std::vector<unsigned char>& signal_filt, const unsigned int state);


  // States
  void idle(RobotDockingState::State& state,double& vx, double& wz);
  void scan(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt, const ecl::LegacyPose2D<double>& pose_update, std::string& debug_str);
  void find_stream(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt);
  void get_stream(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt);
  void aligned(RobotDockingState::State& state,double& vx, double& wz, const std::vector<unsigned char>& signal_filt, std::string& debug_str);
  void bumped(RobotDockingState::State& nstate,double& nvx, double& nwz, int& bump_count);


private:
  bool is_enabled, can_run;

  RobotDockingState::State state;
  std::string state_str, debug_str;
  double vx, wz;
  std::vector<std::vector<unsigned char> > past_signals;
  unsigned int signal_window;
  int bump_remainder;
  int dock_stabilizer;
  int dock_detector;
  double rotated;
  double min_abs_v;
  double min_abs_w;
  ecl::LegacyPose2D<double> pose_priv;

  void setVel(double v, double w);

  std::string binary(unsigned char number) const;

  std::string debug_output;
  std::vector<std::string> ROBOT_STATE_STR;
};

} // namespace kobuki

#endif /* KOBUKI_DOCK_DRIVE_HPP_ */
