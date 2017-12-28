/*
 * Copyright (c) 2013, Yujin Robot.
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
 * @file /kobuki_dock_drive/include/kobuki_dock_drive/state.hpp
 *
 * @brief States 
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef KOBUKI_DOCK_DRIVE_STATE_HPP_
#define KOBUKI_DOCK_DRIVE_STATE_HPP_

/*****************************************************************************
** States
*****************************************************************************/
#include <iostream>

namespace kobuki {

  // indicates the ir sensor from docking station
  struct DockStationIRState {
    enum State {
      INVISIBLE=0,
      NEAR_LEFT=1,
      NEAR_CENTER=2,
      NEAR_RIGHT=4,
      FAR_CENTER=8,
      FAR_LEFT=16,
      FAR_RIGHT=32,
      NEAR = 7, // NEAR_LEFT + NEAR_CENTER + NEAR_RIGHT
      FAR = 56, // FAR_CENTER + FAR_LEFT + FAR_RIGHT
    };
  };

  // the current robot states
  struct RobotDockingState {
    enum State {
      IDLE,
      DONE,
      DOCKED_IN,
      BUMPED_DOCK,
      BUMPED,
      SCAN,
      FIND_STREAM,
      GET_STREAM,
      ALIGNED,
      ALIGNED_FAR,
      ALIGNED_NEAR,
      UNKNOWN,
      LOST
    };

  };

  /*
  struct RobotDockingState {
      enum State {
        IDLE,
        NEAR_LEFT,
        NEAR_CENTER,
        NEAR_RIGHT,
        FAR_CENTER,
        FAR_LEFT,
        FAR_RIGHT,
        IN_DOCK,
        DONE,
        ERROR,
      };
  };*/
}

#endif // KOBUKI_DOCK_DRIVE_STATE_HPP_ 
