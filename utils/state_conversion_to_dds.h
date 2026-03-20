#pragma once

#include <type_traits>
#include <cstring>
#include <array>

#include "ZdlController/controller_api.h"
#include "../idl/robot_state.h"

// ================= 工具函数 =================
template<typename T, size_t N>
inline void copyArray(const std::array<T, N>& src, T* dst)
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "copyArray requires trivially copyable type");

    std::memcpy(dst, src.data(), sizeof(T) * N);
}

template<typename SrcT, typename DstT, size_t N>
inline void copyArrayCast(const std::array<SrcT, N>& src, DstT* dst)
{
    static_assert(std::is_trivially_copyable<SrcT>::value, "Src must be trivially copyable");
    static_assert(std::is_trivially_copyable<DstT>::value, "Dst must be trivially copyable");

    for (size_t i = 0; i < N; ++i)
    {
        dst[i] = static_cast<DstT>(src[i]);
    }
}


// ================= Joint =================
inline void ToDdsJointState7(
    const controller::RobotState::JointState<7>& src,
    zdl_msg_dds__state_JointState7& dst)
{
    copyArray(src.status_word, dst.status_word);
    copyArray(src.error_code, dst.error_code);
    copyArrayCast(src.mode_of_operation, dst.mode_of_operation);
    copyArray(src.q, dst.q);
    copyArray(src.q_d, dst.q_d);
    copyArray(src.dq, dst.dq);
    copyArray(src.tau_J, dst.tau_J);
}

inline void ToDdsJointState2(
    const controller::RobotState::JointState<2>& src,
    zdl_msg_dds__state_JointState2& dst)
{
    copyArray(src.status_word, dst.status_word);
    copyArray(src.error_code, dst.error_code);
    copyArrayCast(src.mode_of_operation, dst.mode_of_operation);
    copyArray(src.q, dst.q);
    copyArray(src.q_d, dst.q_d);
    copyArray(src.dq, dst.dq);
    copyArray(src.tau_J, dst.tau_J);
}

inline void ToDdsJointState6(
    const controller::RobotState::JointState<6>& src,
    zdl_msg_dds__state_JointState6& dst)
{
    copyArray(src.status_word, dst.status_word);
    copyArray(src.error_code, dst.error_code);
    copyArrayCast(src.mode_of_operation, dst.mode_of_operation);
    copyArray(src.q, dst.q);
    copyArray(src.q_d, dst.q_d);
    copyArray(src.dq, dst.dq);
    copyArray(src.tau_J, dst.tau_J);
}

// ================= Arm =================
inline void ToDdsArmState(
    const controller::RobotState::ArmState& src,
    zdl_msg_dds__state_ArmState& dst)
{
    ToDdsJointState7(src, dst.joint_state);

    copyArray(src.dq_d, dst.dq_d);
    copyArray(src.tau_Js, dst.tau_Js);
    copyArray(src.tau_J_d, dst.tau_J_d);

    copyArray(src.O_T_EE, dst.O_T_EE);
    copyArray(src.O_T_EE_d, dst.O_T_EE_d);

    dst.m_total = src.m_total;
    copyArray(src.I_total, dst.I_total);
    copyArray(src.F_x_Ctotal, dst.F_x_Ctotal);

    copyArray(src.joint_k_gains, dst.joint_k_gains);
    copyArray(src.joint_d_gains, dst.joint_d_gains);

    copyArray(src.stiffness, dst.stiffness);
    copyArray(src.damping, dst.damping);
}

// ================= 其他部件 =================
inline void ToDdsFoldedWaist(
    const controller::RobotState::FoldedWaist& src,
    zdl_msg_dds__state_FoldedWaist& dst)
{
    ToDdsJointState6(src, dst.joint_state);
    copyArray(src.q_e, dst.q_e);
}

inline void ToDdsHandState(
    const controller::RobotState::HandState& src,
    zdl_msg_dds__state_HandState& dst)
{
    ToDdsJointState7(src, dst.joint_state);
}

inline void ToDdsHeadState(
    const controller::RobotState::HeadState& src,
    zdl_msg_dds__state_HeadState& dst)
{
    ToDdsJointState2(src, dst.joint_state);
}

// ================= Robot =================
inline void ToDdsRobotState(
    const controller::RobotState& src,
    zdl_msg_dds__state_RobotState& dst)
{
    ToDdsArmState(src.left_arm, dst.left_arm);
    ToDdsArmState(src.right_arm, dst.right_arm);

    ToDdsHandState(src.left_hand, dst.left_hand);
    ToDdsHandState(src.right_hand, dst.right_hand);

    ToDdsFoldedWaist(src.folded_waist, dst.folded_waist);

    ToDdsHeadState(src.head, dst.head);

    // enum（C enum）
    dst.current_errors =
        (zdl_msg_dds__state_RobotError)src.current_errors;

    dst.current_mode =
        (zdl_msg_dds__state_CurrentMode)src.current_mode;

    dst.current_mission =
        (zdl_msg_dds__state_CurrentMission)src.current_mission;

    dst.current_arm_mode =
        (zdl_msg_dds__state_CurrentArmMode)src.current_arm_mode;

    // time
    dst.time = src.time.toSec();
}