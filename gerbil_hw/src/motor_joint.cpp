// Copyright (c) 2023 Eric Cox
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roboclaw_hardware_interface/motor_joint.hpp"

#include <cmath>
#include <limits>

namespace roboclaw_hardware_interface
{

MotorJoint::MotorJoint(const std::string joint_name, const int32_t qppr)
: ticks_per_radian_(static_cast<double>(qppr) * 0.5 * M_1_PI), name(joint_name)
{
}

int32_t MotorJoint::getTickRateCommand() const
{
  // Guard against NaN or infinity which would cause undefined behavior on cast
  if (!std::isfinite(velocity_command_)) {
    return 0;
  }
  
  double tick_rate = velocity_command_ * ticks_per_radian_;
  
  // Clamp to int32_t range to prevent overflow
  constexpr double max_tick_rate = static_cast<double>(std::numeric_limits<int32_t>::max());
  constexpr double min_tick_rate = static_cast<double>(std::numeric_limits<int32_t>::min());
  
  if (tick_rate > max_tick_rate) {
    return std::numeric_limits<int32_t>::max();
  }
  if (tick_rate < min_tick_rate) {
    return std::numeric_limits<int32_t>::min();
  }
  
  return static_cast<int32_t>(tick_rate);
}

// Set the position given the current wheel encoder count
void MotorJoint::setPositionState(const int32_t & encoder_count)
{
  if (initialized_encoder_count_) {
    // Update the joint angle
    position_state_ +=
      static_cast<double>(encoder_count - prior_encoder_count_) / ticks_per_radian_;
  } else {
    initialized_encoder_count_ = true;
  }
  // Store the prior encoder count for next time
  prior_encoder_count_ = encoder_count;
}

void MotorJoint::setVelocityState(const int32_t & encoder_speed)
{
  // Convert EncCounts/Sec directly to Radians/Sec
    velocity_state_ = static_cast<double>(encoder_speed) / ticks_per_radian_;
}
}  // namespace roboclaw_hardware_interface
