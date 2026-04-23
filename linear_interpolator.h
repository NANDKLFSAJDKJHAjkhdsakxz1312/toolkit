#pragma once
#include <array>
#include <mutex>
#include <algorithm>
#include <cmath>

class LinearInterpolator
{
public:
    static constexpr int DOF = 36;

    // 👉 DDS接收接口（60Hz）
    void receive(double timestamp, const std::array<double, DOF>& q);

    // 👉 控制循环调用（1000Hz）
    bool get(std::array<double, DOF>& q_out);

private:
    std::mutex mtx_;

    std::array<double, DOF> q0_{};
    std::array<double, DOF> q1_{};
    std::array<double, DOF> last_q_{};

    double t0_ = 0.0;
    double t1_ = 0.0;
    double last_t_ = 0.0;

    int step_ = 0;
    int total_steps_ = 1;

    bool has_first_point_ = false;
    bool has_segment_ = false;
};
