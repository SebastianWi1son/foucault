#pragma once

#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {


/// Tuning parameters for Mahony.
struct MahonyConfig {
    float kp_ = 5.0f;               ///< Accelerometer proportional gain.
    float ki_ = 0.3f;               ///< Accelerometer integral gain (gyro bias estimate).
    float integral_limit_ = 10.0f;  ///< Integral term clamp [rad/s].
    float acc_timeout_ = 0.1f;      ///< Accelerometer residual validity [s].

    float acc_min_ = 0.5f;          ///< Accepted |acc| lower bound [g]; rejects dead sensor / free-fall.
    float acc_max_ = 2.0f;          ///< Accepted |acc| upper bound [g]; rejects saturation.

    float kp_heading_ = 5.0f;       ///< Heading proportional gain (P only, not integrated).
    float heading_timeout_ = 0.3f;  ///< Heading reference validity [s].
};

class Mahony {
public:
    constexpr Mahony() = default;
    explicit Mahony(const MahonyConfig& cfg);

    // ----- State -----
    /// @brief Reset attitude to identity.
    void reset();

    /// @brief Reset attitude to the given orientation.
    /// @param q Initial attitude (normalized internally).
    void reset(const math::Quatf& q);

    // ----- Input (multi-rate: drive each source at its own rate) -----
    /// @brief Advance by dt: gyro integration plus all channel corrections.
    /// @param gyro Body-frame angular rate [rad/s]
    /// @param dt   Time step [s]
    void predict(const math::Vec3f& gyro, float dt);

    /// @brief Store an accelerometer sample (direction only, normalized internally).
    /// @note Samples failing the input guards are dropped and counted, not stored.
    /// @param acc Raw reading in [g]; |acc| must lie in [acc_min_, acc_max_]
    void observe(const math::Vec3f& acc);

    /// @brief Store an external heading reference.
    /// @param heading_ref Absolute heading [rad]
    /// @param trust       Confidence in [0,1]; 0 ignores the reference
    /// @note First call auto-aligns (see align_heading).
    void observe_heading(float heading_ref, float trust = 1.0f);

    /// @brief Re-bind the heading zero to heading_ref. Attitude does not jump; roll/pitch untouched.
    /// @param heading_ref Reference heading [rad] matching the current yaw
    /// @note Use after repositioning or an odometry restart.
    void align_heading(float heading_ref);

    /// @brief Single-step convenience: observe() then predict().
    void update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt);

    // ----- Output -----
    /// @brief Current attitude (unit quaternion).
    const math::Quatf& quaternion() const;

    /// @brief Current attitude as ZYX intrinsic Euler angles [rad]: (roll, pitch, yaw).
    math::Vec3f euler() const;

    // ----- Status -----
    /// @brief True while the accelerometer residual is fresh; false = roll/pitch on gyro only.
    bool is_acc_valid() const;

    /// @brief True while the heading reference is fresh; false = yaw on gyro only.
    bool is_heading_valid() const;

    /// @brief Input frames rejected by the guards (NaN/Inf, |acc| out of range, dt <= 0).
    /// @note Counted since construction or the last reset(); a rising count means a dying sensor.
    unsigned rejected_count() const;

private:
    static constexpr float k_never_measured_ = 1.0e6f;

    void reset_heading_channel();

    math::Vec3f correction_acc(const math::Vec3f& v, float dt);     // ⚠ 副作用：更新e_int_（积分累加）
    math::Vec3f correction_heading(const math::Vec3f& v);           //   无副作用

    MahonyConfig cfg_;
    math::Quatf q_;                             // 姿态状态(估算器的输出)
    math::Vec3f e_int_;                         // integral (零偏估计)
    math::Vec3f acc_;                           // 最近一次有效 acc 量测（已归一化，只存方向）
    float acc_age_ = k_never_measured_;         // 距离上一次有效acc量测时间间隔
    unsigned rejected_count_ = 0;

    float heading_ref_ =0.0f;                       // 最近的一次航向参考
    float heading_offset_ = 0.0f;                   // 首次观测的自动对齐量
    float heading_age_ = k_never_measured_;         // 距上次航向参考的秒数
    float heading_trust_ = 1.0;                     // 最近一次参考的可信度
    bool is_heading_aligned_ = false;               // 是否已完成首次对齐
};




}