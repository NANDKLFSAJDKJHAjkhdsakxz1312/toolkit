#pragma once

#include <cstdint>

#include "../idl/start_task.h"
#include "../idl/enable.h"
#include "ZdlController/robot_state.h"

namespace zdl {
namespace msg {
namespace dds_ {

using DdsCurrentArmMode = zdl_msg_dds__CurrentArmMode;
using DdsInertia = zdl_msg_dds__Inertia;
using DdsEndEffector = zdl_msg_dds__EndEffector;
using DdsStartConfig = zdl_msg_dds__StartConfig;
using DdsStartRequest = zdl_msg_dds__StartRequest;


// 默认规则与 controller::StartConfig 对齐:
// arm_mode = kDualArm, distance_between_arm = 0.5, 其余为 0.
DdsStartConfig MakeDefaultStartConfig(
    DdsCurrentArmMode arm_mode = zdl_msg_dds__kDualArm,
    double distance_between_arm = 0.5);
DdsStartRequest MakeDefaultStartRequest(
    uint32_t request_id,
    DdsCurrentArmMode arm_mode = zdl_msg_dds__kDualArm,
    double distance_between_arm = 0.5);

// RPC(IDL) -> 控制器
controller::StartConfig ToControllerStartConfig(const DdsStartConfig& in);
controller::StartConfig ToControllerStartConfig(const DdsStartRequest& in);
controller::RequestMode ToControllerRequestMode(zdl_msg_dds__EnableMode mode);
// 控制器 -> RPC(IDL)
DdsStartConfig ToDdsStartConfig(const controller::StartConfig& in);
DdsStartRequest ToDdsStartRequest(uint32_t request_id, const controller::StartConfig& config);

}  // namespace dds_
}  // namespace msg
}  // namespace zdl
