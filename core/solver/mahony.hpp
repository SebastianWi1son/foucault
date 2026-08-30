#pragma once

#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {


struct MahonyConfig {
    float kp_ = 5.0f;
    float ki_ = 0.3f;
    float integral_limit_ = 10.0f;
};

class Mahony {
public:
    constexpr Mahony() = default;
    explicit Mahony(const MahonyConfig& cfg) : cfg_(cfg) {}

    void reset();           // 姿态复位为单位元
    void reset(const math::Quatf& q);   // 指定初始姿态(初始对齐F5？)

    // 异步多速率
    // predict 同imu同速率，observe事件驱动
    void predict(const math::Vec3f& gyro, float dt);
    void observe(const math::Vec3f& acc, float dt);
    void update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt);

    const math::Quatf& quaternion() const { return q_; }
    math::Vec3f euler() const;

private:
    MahonyConfig cfg_;
    math::Quatf q_;         // 姿态状态(估算器的输出)
    math::Vec3f e_int_;     // integral (零偏估计)
    math::Vec3f e_last_;    // 最近一次残差
    bool is_meas_ = false;// 是否已收到量测
};




}