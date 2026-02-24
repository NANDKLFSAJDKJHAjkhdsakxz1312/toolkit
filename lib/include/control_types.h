#pragma once

#include <array>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace controller {

struct ControlTypeBase
{
  bool motion_finished = false;
};

struct ControlCommand{
  std::array<double, 7> left_arm{0.0};
  std::array<double, 7> right_arm{0.0};
  std::array<double, 4> waist{0.0};
};

class Torques : public ControlTypeBase
{
 public:
  Torques(const std::array<double, 14>& torques) noexcept: tau_J(torques) {}

  Torques(std::initializer_list<double> torques) {
  if (torques.size() != tau_J.size()) {
    throw std::invalid_argument("Invalid number of elements in tau_J.");
  }
  std::copy(torques.begin(), torques.end(), tau_J.begin());
}

  std::array<double, 14> tau_J{};
};

class JointPositions : public ControlTypeBase
{
 public:
  JointPositions(const ControlCommand& joint_positions) noexcept: q(joint_positions) {}

  ControlCommand q{};
};

inline Torques MotionFinished(Torques command) noexcept
{
  command.motion_finished = true;
  return command;
}

inline JointPositions MotionFinished(JointPositions command) noexcept
{
  command.motion_finished = true;
  return command;
}
}  // namespace controller