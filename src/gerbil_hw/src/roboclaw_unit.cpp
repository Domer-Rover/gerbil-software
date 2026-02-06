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

#include "roboclaw_hardware_interface/roboclaw_unit.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <roboclaw_serial/command.hpp>
#include <roboclaw_serial/interface.hpp>

#include "roboclaw_hardware_interface/motor_joint.hpp"

namespace roboclaw_hardware_interface
{

RoboClawUnit::RoboClawUnit(
  roboclaw_serial::Interface::SharedPtr interface, uint8_t address, MotorJoint::SharedPtr m1,
  MotorJoint::SharedPtr m2, bool use_duty_cycle)
: address_(address), use_duty_cycle_(use_duty_cycle)
{
  // Copy the pointer to the roboclaw interface
  interface_ = interface;

  // Set motor joint configurations
  joints[0] = m1;
  joints[1] = m2;

  if (use_duty_cycle_) {
    std::cerr << "[RoboClawUnit] Using duty cycle mode (no encoders required)" << std::endl;
  }
}

// Read the encoder counts from the roboclaw and update position state
void RoboClawUnit::read()
{
  try {
    // Read both encoders in single transaction
    interface_->read(encoder_state_, address_);
    const auto & [m1_ticks, m2_ticks] = encoder_state_.fields;

    // Convert tick counts to position states
    if (joints[0]) {
      joints[0]->setPositionState(m1_ticks);
    }
    if (joints[1]) {
      joints[1]->setPositionState(m2_ticks);
    }
  } catch (const std::exception & e) {
    static auto last_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
      std::cerr << "[RoboClawUnit] Read error from RoboClaw 0x" << std::hex
                << static_cast<int>(address_) << std::dec << ": " << e.what() << std::endl;
      last_log = now;
    }
    return;
  }
}

// Write the motor command to the roboclaw
void RoboClawUnit::write()
{
  try {
    if (use_duty_cycle_) {
      // Duty cycle mode: no encoders needed
      auto & [m1_duty, m2_duty] = duty_command_.fields;
      if (joints[0]) {
        m1_duty = joints[0]->getDutyCycleCommand();
      }
      if (joints[1]) {
        m2_duty = joints[1]->getDutyCycleCommand();
      }
      interface_->write(duty_command_, address_);
    } else {
      // Velocity PID mode: requires encoders
      auto & [m1_speed, m2_speed] = tick_rate_command_.fields;
      if (joints[0]) {
        m1_speed = joints[0]->getTickRateCommand();
      }
      if (joints[1]) {
        m2_speed = joints[1]->getTickRateCommand();
      }
      interface_->write(tick_rate_command_, address_);
    }
  } catch (const std::exception & e) {
    static auto last_log = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log).count() >= 5) {
      std::cerr << "[RoboClawUnit] Write error to RoboClaw 0x" << std::hex
                << static_cast<int>(address_) << std::dec << ": " << e.what() << std::endl;
      last_log = now;
    }
    return;
  }
}
}  // namespace roboclaw_hardware_interface
