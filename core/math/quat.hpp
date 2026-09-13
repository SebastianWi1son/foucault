#pragma once

#include <cmath>
#include "scalar_ops.hpp"
#include "vec3.hpp"

namespace foucault::math {

template <typename T>
struct Quat {
    T q0_, q1_, q2_, q3_;

    constexpr Quat() : q0_(1), q1_(0), q2_(0), q3_(0) {}        // 单位元-零旋转
    constexpr Quat(T w, T x, T y, T z) : q0_(w), q1_(x), q2_(y), q3_(z) {}

    static Quat identity() { return Quat(1,0,0,0); }

    // --- self-normalization ---
    // 积分后必做，模长漂离1会让旋转变形。
    // 用位魔法 inv_sqrt（相对误差 ~4e-6）而非 1/std::sqrt：本函数在 1kHz 级热路径上
    // 每帧都跑，4e-6 的模长误差对应 ~2e-4° 的姿态误差，可忽略；换精确 sqrt 则慢一个量级（P1-2）。
    T norm_squared() const { return q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_; }
    T norm() const { return std::sqrt(norm_squared()); }
    Quat& normalize() {
        T n2 = norm_squared();
        if (n2 > T(0)) {
            T inv = inv_sqrt(n2);
            q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
        }
        else if (n2 == T(0)) { *this = identity(); }    // 零四元数无意义，复位为单位元
        // n2 = NaN 时不处理：脏值必须传到调用方，不得静默洗成单位元（P0-5）
        // （n2 = +Inf 走上面分支 → 结果是非有限值，明显异常、可被检出）
        return *this;
    }

    // 共轭 = 单位四元数的逆
    Quat conjugated() const { return Quat(q0_, -q1_, -q2_, -q3_); }
    
    // Hamilton Product: q * r
    Quat operator*(const Quat& r) const {
        return Quat(
           q0_ * r.q0_ - q1_ * r.q1_ - q2_ * r.q2_ - q3_ * r.q3_,
            q0_ * r.q1_ + q1_ * r.q0_ + q2_ * r.q3_ - q3_ * r.q2_,
            q0_ * r.q2_ - q1_ * r.q3_ + q2_ * r.q0_ + q3_ * r.q1_,
            q0_ * r.q3_ + q1_ * r.q2_ - q2_ * r.q1_ + q3_ * r.q0_);
    }

    // 旋转向量：v' = q ⊗ (0,v) ⊗ q*
    // ⚠ 下面的快速展开式【仅当 |q| = 1】时等于 q⊗(0,v)⊗q*；非单位四元数下两者相差可观
    //   （实测 |q|²=1.09 时输出模长 1.0161，而精确乘积应为 1.09）→ rotate 前必须归一化
    Vec3<T> rotate(const Vec3<T>& v) const {
        Vec3<T> qv(q1_, q2_, q3_);
        Vec3<T> t = qv.cross(v) * T(2);
        return v + t * q0_ + qv.cross(t);
    }

    // 陀螺积分（名义一阶欧拉，实测量化后为全局二阶）：q̇ = ½·q⊗(0,ω) → q += ½·q⊗(0,ω)·dt → 归一化
    // 二阶来源：归一化恰好消掉二阶项（纯四元数 x 有 x² = -|x|²），与 exp(½ω̂dt) 差 O(|x|³)
    // 实测观测阶 p = 2.04（常角速度）/ 1.98（时变 ω）；见 tests/unit/test_math_audit.cpp F3
    // ω 是机体系角速度（rad/s），dt 单位 s
    Quat& integrate(const Vec3<T>& omega, T dt) {
        T half = T(0.5) * dt;
        T wx = omega.x_, wy = omega.y_, wz = omega.z_;
        // q_dot =  1/2 * q ⊗ (0, w1, w2, w3)
        // q += q_dot
        T q0 = q0_ + (-q1_ * wx - q2_ * wy - q3_ * wz) * half;
        T q1 = q1_ + ( q0_ * wx + q2_ * wz - q3_ * wy) * half;
        T q2 = q2_ + ( q0_ * wy - q1_ * wz + q3_ * wx) * half;
        T q3 = q3_ + ( q0_ * wz + q1_ * wy - q2_ * wx) * half;

        q0_ = q0; q1_ = q1; q2_ = q2; q3_ = q3;
        return normalize();
    }

    // 欧拉角 → 四元数（内旋 ZYX：先 yaw、再 pitch、最后 roll），弧度制
    static Quat from_euler(T roll, T pitch, T yaw) {
        T cr = std::cos(roll  * T(0.5)), sr = std::sin(roll  * T(0.5));
        T cp = std::cos(pitch * T(0.5)), sp = std::sin(pitch * T(0.5));
        T cy = std::cos(yaw   * T(0.5)), sy = std::sin(yaw   * T(0.5));
        return Quat(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy);
    }

    // 四元数 → 欧拉角（atan2/asin 直接公式），输出弧度
    void to_euler(T& roll, T& pitch, T& yaw) const {
        roll  = std::atan2(T(2) * (q0_ * q1_ + q2_ * q3_), T(1) - T(2) * (q1_ * q1_ + q2_ * q2_));
        pitch = std::asin(T(-2) * (q1_ * q3_ - q0_ * q2_));
        yaw   = std::atan2(T(2) * (q1_ * q2_ + q0_ * q3_), T(1) - T(2) * (q2_ * q2_ + q3_ * q3_));
    }
};

using Quatf = Quat<float>;

}  // namespace foucault::math
