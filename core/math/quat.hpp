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
    // 积分后必做，模长漂离1会让旋转变形
    T norm_squared() const { return q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_; }
    T norm() const { return std::sqrt(norm_squared()); }
    Quat& normalize() {
        T n2 = norm_squared();
        if (n2 > T(0)) {
            T inv = inv_sqrt(n2);
            q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
        }
        else { *this = identity(); }        // 零四元数无意义，复位为单位元
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

    // 旋转向量: v' = q

    Vec3<T> rotate(const Vec3<T>& v) const {
        Vec3<T> qv(q1_, q2_, q3_);
        Vec3<T> t = qv.cross(v) * T(2);
        return v + t * q0_ + qv.cross(t);
    }

    // 陀螺积分（一阶欧拉）：q̇ = ½·q⊗(0,ω) → q += ½·q⊗(0,ω)·dt → 归一化
    // ω 是机体系角速度（rad/s），dt 单位 s
    Quat& integrate(const Vec3<T>& omega, T dt) {
        T half = T(0.5) * dt;
        T wx = omega.x_, wy = omega.y_, wz = omega.z_;
        T n0 = q0_ + (-q1_ * wx - q2_ * wy - q3_ * wz) * half;
        T n1 = q1_ + ( q0_ * wx + q2_ * wz - q3_ * wy) * half;
        T n2 = q2_ + ( q0_ * wy - q1_ * wz + q3_ * wx) * half;
        T n3 = q3_ + ( q0_ * wz + q1_ * wy - q2_ * wx) * half;
        q0_ = n0; q1_ = n1; q2_ = n2; q3_ = n3;
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








}






