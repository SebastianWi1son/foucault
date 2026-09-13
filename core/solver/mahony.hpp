#pragma once

#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {


struct MahonyConfig {
    float kp_ = 5.0f;
    float ki_ = 0.3f;
    float integral_limit_ = 10.0f;
    float acc_timeout_ = 0.1f;
    // external yaw entry
    float kp_heading_ = 5.0f;
    float heading_timeout_ = 0.3f;      // 航向参考有效期 10hz容忍三个丢包
};

class Mahony {
public:
    constexpr Mahony() = default;
    explicit Mahony(const MahonyConfig& cfg);

    void reset();                       // 姿态复位为单位元
    void reset(const math::Quatf& q);   // 指定初始姿态(初始对齐F5？)

    // 异步多速率
    // predict 同imu同速率，observe事件驱动
    void predict(const math::Vec3f& gyro, float dt);
    void observe(const math::Vec3f& acc);        // 只存量测（残差留给 predict 用当前 q̂ 现算）
    void observe_heading(float heading_ref, float trust = 1.0f);
    void update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt);

    // Quality Check
    bool is_acc_valid() const;
    bool is_heading_valid() const;

    const math::Quatf& quaternion() const;
    math::Vec3f euler() const;

private:
    static constexpr float k_never_measured_ = 1.0e6f;

    void reset_heading_channel();

    MahonyConfig cfg_;
    math::Quatf q_;                             // 姿态状态(估算器的输出)
    math::Vec3f e_int_;                         // integral (零偏估计)
    math::Vec3f acc_;                           // 最近一次有效 acc 量测（已归一化，只存方向）
    float acc_age_ = k_never_measured_;         // 距离上一次有效acc量测时间间隔

    float heading_ref_ =0.0f;                       // 最近的一次航向参考
    float heading_offset_ = 0.0f;                   // 首次观测的自动对齐量
    float heading_age_ = k_never_measured_;         // 距上次航向参考的秒数
    float heading_trust_ = 1.0;                     // 最近一次参考的可信度
    bool is_heading_aligned_ = false;               // 是否已完成首次对齐
};




}