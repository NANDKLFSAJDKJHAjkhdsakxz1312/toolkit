#pragma once

#include <duration.h>

#include <array>
#include <ostream>

namespace controller {

enum class RequestMode { kCSP = 0, kCST = 1 };

enum class CurrentMode { kNotStart, kNotEnable, kCSP, kCST, kOther };

enum class CurrentMission {
  kUnavailable,
  kIdle,
  kHoming,
  kJog,
  kReturnToZero,
  kDrag,
  kJointImpedance,
  kCartImpedance,
  kUserCSP,
  kUserCST,
  kFoldedWaistControl
};

enum class JointSelect {
  // 左臂关节 1-7
  kLeftJoint1,
  kLeftJoint2,
  kLeftJoint3,
  kLeftJoint4,
  kLeftJoint5,
  kLeftJoint6,
  kLeftJoint7,

  // 右臂关节 1-7
  kRightJoint1,
  kRightJoint2,
  kRightJoint3,
  kRightJoint4,
  kRightJoint5,
  kRightJoint6,
  kRightJoint7,

  // 腰部关节 1-4
  kWaistJoint1,
  kWaistJoint2,
  kWaistJoint3,
  kWaistJoint4,

  // 双轮
  kLeftWheel,
  kRightWheel
};

enum class HandSelect
{
  kLeftHand,
  kRightHand
};

enum class CurrentArmMode { kDualArm, kLeftArmOnly, kRightArmOnly, kFoldedWaistOnly, kLeftArmWithHand };

struct StartConfig
{
  CurrentArmMode arm_mode{CurrentArmMode::kDualArm};  // 1. 工况
  double distance_between_arm{0.5};                   // 2. 臂间距

  struct EndEffector
  {
    double m{0.0};                             // 1. 质量 (单位: kg)
    std::array<double, 3> com{0.0, 0.0, 0.0};  // 2. 质心 (单位: m)

    struct Inertia
    {
      double ixx{0.0}, iyy{0.0}, izz{0.0};
      double ixy{0.0}, ixz{0.0}, iyz{0.0};
    } inertia;  // 3. 惯量张量 (Inertia Tensor)（单位: kg·m^2）
  };

  EndEffector left_end_effector;
  EndEffector right_end_effector;
};

enum class Direction { kForward, kReverse, kStop };

struct JointImpedanceParameter
{
  std::array<double, 7> left_arm_stiffness{};   // 左臂刚度
  std::array<double, 7> right_arm_stiffness{};  // 右臂刚度
  std::array<double, 7> left_arm_damping{};     // 左臂阻尼
  std::array<double, 7> right_arm_damping{};    // 右臂阻尼
};

struct CartesianImpedanceParameter
{
  std::array<double, 6> left_arm_stiffness{};   // 左臂刚度
  std::array<double, 6> right_arm_stiffness{};  // 右臂刚度
  std::array<double, 6> left_arm_damping{};     // 左臂阻尼
  std::array<double, 6> right_arm_damping{};    // 右臂阻尼
};

// 错误码
enum class RobotError {
  OK = 0,
  kEmergencyStop,      // 急停触发
  kPoseOverLimit,      // 位置超限
  kJointOverTemp,      // 关节过热
  kEncoderFault,       // 编码器故障
  kCollisionDetected,  // 碰撞检测
  kCommunicationLost   // 通信丢失
};

enum class RobotArm { kLeftArm, kRightArm };

struct RobotState
{
  template <size_t N>
  struct JointState
  {
    std::array<uint16_t, N> status_word{};      // 状态字
    std::array<uint16_t, N> error_code{};       // 错误字
    std::array<int8_t, N> mode_of_operation{};  // 运行模式
    std::array<double, N> q{};                  // 关节位置（反馈值）
    std::array<double, N> q_d{};                // 关节位置（期望值）
    std::array<double, N> dq{};                 // 关节速度（反馈值）
    std::array<double, N> tau_J{};              // 关节力矩（电机反馈值）
  };

  struct ArmState : public JointState<7>
  {
    std::array<double, 7> dq_d{};     // 关节速度（期望值）
    std::array<double, 7> tau_Js{};   // 关节力矩（传感器反馈值）
    std::array<double, 7> tau_J_d{};  // 关节力矩（期望值）

    // 末端执行器位姿
    std::array<double, 16> O_T_EE{};    // 末端位姿（反馈值）
    std::array<double, 16> O_T_EE_d{};  // 末端位姿（期望值）

    // 末端执行器参数
    double m_total{};                    // 质量（末端执行器+负载）
    std::array<double, 9> I_total{};     // 惯性张量（末端执行器+负载）
    std::array<double, 3> F_x_Ctotal{};  // 质心（末端执行器+负载）

    // 关节阻抗参数
    std::array<double, 7> joint_k_gains{0.0};
    std::array<double, 7> joint_d_gains{0.0};

    // 笛卡尔阻抗参数
    std::array<double, 6> stiffness{0.0};
    std::array<double, 6> damping{0.0};
  };

  struct FoldedWaist : public JointState<6>
  {
    std::array<double, 6> q_e{};
  };

  struct HandState : public JointState<7>
  {
  };

  struct HeadState : public JointState<2>
  {
  };

  /*  ********************************************************** */
  ArmState left_arm; // 左臂状态

  ArmState right_arm; // 右臂状态

  HandState left_hand; // 左手状态

  HandState right_hand; // 右手状态

  FoldedWaist folded_waist; // 腰部状态

  HeadState head; // 头状态

  RobotError current_errors{RobotError::OK};

  // RobotError last_motion_errors{};

  CurrentMode current_mode;  // 当前模式

  CurrentMission current_mission;  // 当前状态

  CurrentArmMode current_arm_mode;  // 当前构型

  Duration time{};  // 当前时间
};

}  // namespace controller