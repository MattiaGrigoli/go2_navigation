// BSD 3-Clause License

// Copyright (c) 2024, Intelligent Robotics Lab
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GO2_DRIVER__GO2_API_ID_HPP
#define GO2_DRIVER__GO2_API_ID_HPP

namespace go2_driver
{

enum class Mode
{
  Damp = 1001,
  BalanceStand = 1002,
  StopMove = 1003,
  StandUp = 1004,
  StandDown = 1005,
  RecoveryStand = 1006,
  Euler = 1007,
  Move = 1008,
  Sit = 1009,
  RiseSit = 1010,
  SwitchGait = 1011,
  Trigger = 1012,
  BodyHeight = 1013,
  FootRaiseHeight = 1014,
  SpeedLevel = 1015,
  Hello = 1016,
  Stretch = 1017,
  TrajectoryFollow = 1018,
  ContinuousGait = 1019,
  Content = 1020,
  Wallow = 1021,
  Dance1 = 1022,
  Dance2 = 1023,
  GetBodyHeight = 1024,
  GetFootRaiseHeight = 1025,
  GetSpeedLevel = 1026,
  SwitchJoystick = 1027,
  Pose = 1028,
  Scrape = 1029,
  FrontFlip = 1030,
  FrontJump = 1031,
  FrontPounce = 1032,
  WiggleHips = 1033,
  GetState = 1034,
  EconomicGait = 1035,
  FingerHeart = 1036,
};

}  // namespace go2_driver

#endif // GO2_DRIVER__GO2_API_ID_HPP
