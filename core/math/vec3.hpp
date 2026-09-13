#pragma once

#include <cmath>

namespace foucault::math {

template <typename T>
struct Vec3 {
    // --- internal member ---
    T x_, y_, z_;
    // --- constructor ---
    constexpr Vec3() : x_(0), y_(0), z_(0) {}       // non‑parametric constructor
    constexpr Vec3(T x, T y, T z) : x_(x), y_(y), z_(z) {}    // 三参数构造
    explicit constexpr Vec3(T x) : x_(x), y_(x), z_(x) {}   // 单值广播；explicit 防隐式转换（P1-3）

    // ----- operator reload -----
    // ⚠ 下标越界【不做检查】：0/1 之外（含负数）一律返回 z_（P1-4）。
    //   嵌入式取舍：调用方保证 0..2。要检查请在调用点做，别在这条热路径上加分支。
    T& operator[](int i) {
        switch (i) {
            case 0: return x_;
            case 1: return y_;
            default: return z_;
        }
    }

    // 与上一个同构：const 版本返回 const 引用，供只读对象使用（不是重复代码）
    const T& operator[](int i) const {
        switch (i) {
            case 0: return x_;
            case 1: return y_;
            default: return z_;
        }
    }

    // ----- four arithmetic operations -----
    Vec3 operator+(const Vec3& other) const { return Vec3(x_ + other.x_, y_ + other.y_, z_ + other.z_); }
    Vec3 operator-(const Vec3& other) const { return Vec3(x_ - other.x_, y_ - other.y_, z_ - other.z_); }
    Vec3 operator-() const { return Vec3(-x_, -y_, -z_); }
    Vec3 operator*(T scalar) const { return Vec3(x_ * scalar, y_ * scalar, z_ * scalar); }
    Vec3 operator/(T scalar) const { return Vec3(x_ / scalar, y_ / scalar, z_ / scalar); }
    // --- add on itself ---
    Vec3& operator+=(const Vec3& other) { x_ += other.x_; y_ += other.y_; z_ += other.z_; return *this; }
    Vec3& operator-=(const Vec3& other) { x_ -= other.x_; y_ -= other.y_; z_ -= other.z_; return *this; }
    Vec3& operator*=(T scalar) { x_ *= scalar; y_ *= scalar; z_ *= scalar; return *this; }


    // ----- vector operations -----
    // dot product
    T dot(const Vec3& other) const { return x_ * other.x_ + y_ * other.y_ + z_ * other.z_; }
    // cross product
    Vec3 cross(const Vec3& other) const {
        return Vec3(y_ * other.z_ - z_ * other.y_,
                    z_ * other.x_ - x_ * other.z_,
                    x_ * other.y_ - y_ * other.x_);
    }
    // self-normalization（模长归一到 1；零向量保持零）
    T norm_squared() const { return x_ * x_ + y_ * y_ + z_ * z_; }
    T norm() const { return std::sqrt(norm_squared()); }
    Vec3& normalize() {
        T n = norm();
        if (n > T(0)) {
            T inv = T(1) / n;
            x_ *= inv; y_ *= inv; z_ *= inv;
        }
        return *this;   // 零向量保持零向量（防御）
    }

};

// 标量在左的乘法（T * Vec3），与成员 operator*(T) 对称
template <typename T>
inline Vec3<T> operator*(T scalar, const Vec3<T>& v) { return v * scalar; }

using Vec3f = Vec3<float>;

}  // namespace foucault::math
