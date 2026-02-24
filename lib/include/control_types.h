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
  JointPositions(const std::array<double, 14>& joint_positions) noexcept: q(joint_positions) {}

  JointPositions(std::initializer_list<double> joint_positions){
  if (joint_positions.size() != q.size()) {
    throw std::invalid_argument("Invalid number of elements in joint_positions.");
  }
  std::copy(joint_positions.begin(), joint_positions.end(), q.begin());
}

  std::array<double, 14> q{};
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