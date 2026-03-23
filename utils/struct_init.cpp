#include "struct_init.h"

namespace zdl {
namespace msg {
namespace dds_ {

namespace {

controller::RobotParts ToControllerRobotPart(DdsRobotParts part)
{
    switch (part) {
        case zdl_msg_dds__kLeftArm:
            return controller::RobotParts::kLeftArm;
        case zdl_msg_dds__kRightArm:
            return controller::RobotParts::kRightArm;
        case zdl_msg_dds__kLeftHand:
            return controller::RobotParts::kLeftHand;
        case zdl_msg_dds__kRightHand:
            return controller::RobotParts::kRightHand;
        case zdl_msg_dds__kHead:
            return controller::RobotParts::kHead;
        case zdl_msg_dds__kFoldedWaist:
            return controller::RobotParts::kFoldedWaist;
        case zdl_msg_dds__kDoubleWheel:
            return controller::RobotParts::kDoubleWheel;
        default:
            throw std::runtime_error("unknown DdsRobotParts");
    }
}


controller::CurrentArmMode ToControllerCurrentArmMode(DdsCurrentArmMode mode)
{
  switch (mode)
  {
    case zdl_msg_dds__kDualArm:
      return controller::CurrentArmMode::kDualArm;
    case zdl_msg_dds__kLeftArmOnly:
      return controller::CurrentArmMode::kLeftArmOnly;
    case zdl_msg_dds__kRightArmOnly:
      return controller::CurrentArmMode::kRightArmOnly;
    case zdl_msg_dds__kFoldedWaistOnly:
      return controller::CurrentArmMode::kFoldedWaistOnly;
    default:
      return controller::CurrentArmMode::kDualArm;
  }
}

DdsCurrentArmMode ToDdsCurrentArmMode(controller::CurrentArmMode mode)
{
  switch (mode)
  {
    case controller::CurrentArmMode::kDualArm:
      return zdl_msg_dds__kDualArm;
    case controller::CurrentArmMode::kLeftArmOnly:
      return zdl_msg_dds__kLeftArmOnly;
    case controller::CurrentArmMode::kRightArmOnly:
      return zdl_msg_dds__kRightArmOnly;
    case controller::CurrentArmMode::kFoldedWaistOnly:
      return zdl_msg_dds__kFoldedWaistOnly;
    default:
      return zdl_msg_dds__kDualArm;
  }
}

controller::StartConfig::EndEffector::Inertia ToControllerInertia(const DdsInertia& in)
{
  controller::StartConfig::EndEffector::Inertia out{};
  out.ixx = in.ixx;
  out.iyy = in.iyy;
  out.izz = in.izz;
  out.ixy = in.ixy;
  out.ixz = in.ixz;
  out.iyz = in.iyz;
  return out;
}

controller::StartConfig::EndEffector ToControllerEndEffector(const DdsEndEffector& in)
{
  controller::StartConfig::EndEffector out{};
  out.m = in.m;
  out.com[0] = in.com[0];
  out.com[1] = in.com[1];
  out.com[2] = in.com[2];
  out.inertia = ToControllerInertia(in.inertia);
  return out;
}

DdsInertia ToDdsInertia(const controller::StartConfig::EndEffector::Inertia& in)
{
  DdsInertia out{};
  out.ixx = in.ixx;
  out.iyy = in.iyy;
  out.izz = in.izz;
  out.ixy = in.ixy;
  out.ixz = in.ixz;
  out.iyz = in.iyz;
  return out;
}

DdsEndEffector ToDdsEndEffector(const controller::StartConfig::EndEffector& in)
{
  DdsEndEffector out{};
  out.m = in.m;
  out.com[0] = in.com[0];
  out.com[1] = in.com[1];
  out.com[2] = in.com[2];
  out.inertia = ToDdsInertia(in.inertia);
  return out;
}

DdsRobotParts ToDdsRobotPart(controller::RobotParts part)
{
    switch (part) {
        case controller::RobotParts::kLeftArm:       return zdl_msg_dds__kLeftArm;
        case controller::RobotParts::kRightArm:      return zdl_msg_dds__kRightArm;
        case controller::RobotParts::kLeftHand:      return zdl_msg_dds__kLeftHand;
        case controller::RobotParts::kRightHand:     return zdl_msg_dds__kRightHand;
        case controller::RobotParts::kHead:          return zdl_msg_dds__kHead;
        case controller::RobotParts::kFoldedWaist:   return zdl_msg_dds__kFoldedWaist;
        case controller::RobotParts::kDoubleWheel:   return zdl_msg_dds__kDoubleWheel;
        default: throw std::runtime_error("unknown RobotParts");
    }
}



}  // namespace

// 默认初始化规则与 controller::StartConfig 保持一致:
// arm_mode = kDualArm, distance_between_arm = 0.5, 其余字段为 0.
DdsStartConfig MakeDefaultStartConfig(DdsCurrentArmMode arm_mode, double distance_between_arm)
{
  DdsStartConfig config{};
  config.arm_mode = arm_mode;
  config.distance_between_arm = distance_between_arm;
  return config;
}

DdsStartRequest MakeDefaultStartRequest(
    uint32_t request_id,
    DdsCurrentArmMode arm_mode,
    double distance_between_arm)
{
  DdsStartRequest req{};
  req.request_id = request_id;
  req.config = MakeDefaultStartConfig(arm_mode, distance_between_arm);
  return req;
}

controller::StartConfig ToControllerStartConfig(const DdsStartConfig& in)
{
    controller::StartConfig out{};
    out.arm_mode = ToControllerCurrentArmMode(in.arm_mode);
    out.distance_between_arm = in.distance_between_arm;
    out.left_end_effector = ToControllerEndEffector(in.left_end_effector);
    out.right_end_effector = ToControllerEndEffector(in.right_end_effector);

    // ✅ 拷贝 part_config 序列到 std::vector
    out.part_config.clear();
    for (uint32_t i = 0; i < in.part_config._length; ++i)
    {
        out.part_config.push_back(ToControllerRobotPart(in.part_config._buffer[i]));
    }

    return out;
}

controller::StartConfig ToControllerStartConfig(const DdsStartRequest& in)
{
  return ToControllerStartConfig(in.config);
}

DdsStartConfig ToDdsStartConfig(const controller::StartConfig& in)
{
  DdsStartConfig out = MakeDefaultStartConfig();
  out.arm_mode = ToDdsCurrentArmMode(in.arm_mode);
  out.distance_between_arm = in.distance_between_arm;
  out.left_end_effector = ToDdsEndEffector(in.left_end_effector);
  out.right_end_effector = ToDdsEndEffector(in.right_end_effector);
  // 3️⃣ 处理 part_config
    size_t n = in.part_config.size();
    if (n > 0) {
        out.part_config._buffer = dds_sequence_zdl_msg_dds__RobotParts_allocbuf(n);
        out.part_config._length = n;
        out.part_config._maximum = n;
        out.part_config._release = true;

        for (size_t i = 0; i < n; ++i) {
            out.part_config._buffer[i] = ToDdsRobotPart(in.part_config[i]);
        }
    } else {
        // 空序列
        out.part_config._buffer = nullptr;
        out.part_config._length = 0;
        out.part_config._maximum = 0;
        out.part_config._release = true;
    }
  return out;
}

DdsStartRequest ToDdsStartRequest(uint32_t request_id, const controller::StartConfig& config)
{
  DdsStartRequest out{};
  out.request_id = request_id;
  out.config = ToDdsStartConfig(config);
  return out;
}

controller::RequestMode ToControllerRequestMode(zdl_msg_dds__EnableMode mode)
{
    switch (mode)
    {
        case zdl_msg_dds__kCSP:
            return controller::RequestMode::kCSP;

        case zdl_msg_dds__kCST:
            return controller::RequestMode::kCST;

        default:
            throw std::runtime_error("invalid request mode");
    }
}

}  // namespace dds_
}  // namespace msg
}  // namespace zdl
