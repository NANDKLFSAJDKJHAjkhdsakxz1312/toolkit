#include "linear_interpolator.h"

void LinearInterpolator::receive(double timestamp, const std::array<double, DOF>& q)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // 👉 第一次数据
    if (!has_first_point_)
    {
        t0_ = timestamp;
        q0_ = q;
        has_first_point_ = true;
        return;
    }

    // 👉 第二个点开始，构造插值段
    t1_ = timestamp;
    q1_ = q;

    double dt = t1_ - t0_;

    // =========================
    // ❗异常保护
    // =========================
    if (dt <= 0.0 || dt > 0.1)
    {
        // 数据异常，丢弃
        return;
    }

    // =========================
    // ⭐ 计算插值步数
    // =========================
    int steps = static_cast<int>(std::round(dt / 0.001));  // 1ms

    // 限幅（防抖动）
    total_steps_ = std::clamp(steps, 5, 30);

    step_ = 0;
    has_segment_ = true;

    // 👉 更新上一点
    t0_ = t1_;
    q0_ = q1_;
}

bool LinearInterpolator::get(std::array<double, DOF>& q_out)
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (!has_segment_)
        return false;

    // =========================
    // ⭐ 计算 alpha
    // =========================
    double alpha = static_cast<double>(step_) / total_steps_;

    if (alpha > 1.0)
        alpha = 1.0;

    // =========================
    // ⭐ 线性插值
    // =========================
    for (int i = 0; i < DOF; ++i)
    {
        q_out[i] = q0_[i] + alpha * (q1_[i] - q0_[i]);
    }

    step_++;

    // =========================
    // ⭐ 防止跑过头
    // =========================
    if (step_ > total_steps_)
    {
        // 保持最后一个点
        q_out = q1_;
    }

    return true;
}