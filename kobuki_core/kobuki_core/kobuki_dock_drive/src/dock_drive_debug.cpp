/*
 * copyright (c) 2013, Yujin Robot.
 * all rights reserved.
 *
 * redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * neither the name of yujin robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * this software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. in no event shall the copyright owner or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */
/**
 * @file /kobuki_driver/src/driver/dock_drive_debug.cpp
 *
 **/

/*****************************************************************************
** includes
*****************************************************************************/

#include "kobuki_dock_drive/dock_drive.hpp"

namespace kobuki {
/*************************
 * debug prints
 *************************/

/***********************************************************
  @breif generates debug string to tell the current status of robot. Signal info + bumper + charger + current velocity + dock detector
 ***********************************************************/
void DockDrive::generateDebugMessage(const std::vector<unsigned char>& signal_filt, const unsigned char &bumper, const unsigned char &charger, const ecl::LegacyPose2D<double>& pose_update, const std::string& debug_str)
{

  std::ostringstream debug_stream;
  // pose_update and pose_update_rates for debugging
  std::string far_signal  = "[F: "; //far field
  std::string near_signal = "[N: "; //near field
  for (unsigned int i=0; i<3; i++) {
    if (signal_filt[2-i]&DockStationIRState::FAR_LEFT   ) far_signal  += "L"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_CENTER ) far_signal  += "C"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::FAR_RIGHT  ) far_signal  += "R"; else far_signal  += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_LEFT  ) near_signal += "L"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_CENTER) near_signal += "C"; else near_signal += "-";
    if (signal_filt[2-i]&DockStationIRState::NEAR_RIGHT ) near_signal += "R"; else near_signal += "-";
    far_signal  += " ";
    near_signal += " ";
  }
  far_signal  += "]";
  near_signal += "]";
  debug_stream << far_signal << near_signal;

  //bumper
  {
  std::string out = "[B: ";
  if (bumper&4) out += "L"; else out += "-";
  if (bumper&2) out += "C"; else out += "-";
  if (bumper&1) out += "R"; else out += "-";
  out += "]";
  debug_stream << out;
  }

  //charger
  {
  std::ostringstream oss;
  oss << "[C:" << std::setw(2) << (unsigned int)charger;
  oss << "(";
  if (charger) oss << "ON"; else oss << "  ";
  oss << ")]";
  debug_stream << oss.str();
  }

  //debug_stream << std::fixed << std::setprecision(4)
  debug_stream << "[vx: " << std::setw(7) << vx << ", wz: " << std::setw(7) << wz << "]";
  debug_stream << "[S: " << state_str << "]";
  debug_stream << "[dock_detecotr: : " << dock_detector << " ]";
  debug_stream << "[" << debug_str << "]";
  //debug_stream << std::endl;
  debug_output = debug_stream.str();

  //std::cout << debug_output << std::endl;;
}

std::string DockDrive::binary(unsigned char number) const {
  std::string ret;
  for( unsigned int i=0;i<6; i++){
    if (number&1) ret = "1" + ret;
    else          ret = "0" + ret;
    number = number >> 1;
  }
  return ret;
}

}
