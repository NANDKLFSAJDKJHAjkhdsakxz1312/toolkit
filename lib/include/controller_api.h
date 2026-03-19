#pragma once

#include <array>
#include <functional>
#include <memory>

#include "control_types.h"
#include "robot_state.h"

// 只兼容 Linux/GCC/Clang
#define CONTROLLER_API __attribute__((visibility("default")))

namespace controller {

using JointPositionsFunction  = std::function<JointPositions(const RobotState&, Duration)>;
using TorquesFunction         = std::function<Torques(const RobotState&, Duration)>;

/**
 * @class ControllerInterface
 * @brief 机器人控制器接口类
 *
 * 定义机器人控制的各种运动和控制模式的纯虚函数接口。
 * 所有具体控制器实现应继承此接口。
 */

class CONTROLLER_API ControllerInterface {
 public:
  virtual ~ControllerInterface() = default;


  /************************************控制函数*********************************/
  /**
   * @brief 启动任务
   */
  virtual void start(StartConfig config) = 0;

  /**
   * @brief 停止任务
   */
  virtual void stop() = 0;

  /**
   * @brief 操作模式上使能
   */
  virtual bool enable(const RequestMode mode) = 0;

  /**
   * @brief 操作模式下使能
   */
  virtual void disable() = 0;

  /**
   * @brief 急停
   */
  virtual void emergencyStop() = 0;

  /**
   * @brief 结束当前任务
   */
  virtual void stopCurrentMisiion() = 0;

  /************************************功能函数*********************************/

  /**
   * @brief F1：关节周期位置运动 (CSP-J)
   * @param motion_generator_callback 位置生成回调函数
   * @note 底层集成控制，周期位置控制模式
   */
  virtual const bool runCycleJointMotion(JointPositionsFunction motion_generator_callback) = 0;

  /**
   * @brief F2：返回零位
   * @note 运动到预设的零位位置
   */
  virtual const bool returnToZero() = 0;

  /**
   * @brief F3：点动控制
   * @param joint 目标关节枚举值
   * @param dir 点动方向（正向/反向）
   * @note  仅在 CSP 模式下可用
   */
  virtual void jogControl(JointSelect joint, Direction dir) = 0;

  /**
   * @brief F4：关节周期力控制 (CST-J)
   * @param control_callback 力矩控制回调函数
   * @note 底层集成控制，周期力矩控制模式
   */
  virtual const bool runCycleTorque(TorquesFunction control_callback) = 0;

  /**
   * @brief F5：拖动示教
   * @note 零力拖动模式，用于手动示教
   */
  virtual const void setZeroForceDrag(bool status) = 0;

  /**
   * @brief F6：关节阻抗控制
   * @note 启用关节空间阻抗控制
   */
  virtual const bool runJointImpedance(const JointImpedanceParameter& p) = 0;

  /**
   * @brief F7：笛卡尔阻抗控制
   * @note 启用笛卡尔空间阻抗控制
   */
  virtual const bool runCartesianImpedance(const CartesianImpedanceParameter& p) = 0;


  virtual void setFoldedWaistTarget(double pos) = 0;

  virtual void setWheelTarget(double left_vel, double right_vel) = 0;

  virtual void setHandTarget(HandSelect hand, HandCommand cmd) = 0;

  /************************************设置函数*********************************/

  /**
   * @brief 设置零位
   */
  virtual void setZeroPosition() = 0;

  /**
   * @brief 获取机器人状态
   */
  virtual const RobotState& getRobotState() = 0;
};


[[nodiscard]] CONTROLLER_API std::unique_ptr<ControllerInterface> createController();

}  // namespace controller