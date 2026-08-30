#include "mahony.hpp"

#include "../math/scalar_ops.hpp"

namespace foucault::solver {


void Mahony::reset() {
    q_ = math::Quatf::identity();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    is_meas_ = false;
}

void Mahony::reset(const math::Quatf& q) {
    q_ = q;
    q_.normalize();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    is_meas_ = false;
}

void Mahony::predict(const math::Vec3f& gyro, float dt) {
    math::Vec3f omega = gyro;
    if (is_meas_) {
        // 修正折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
        omega.x_ += cfg_.kp_ * e_last_.x_ + cfg_.ki_ * e_int_.x_;
        omega.y_ += cfg_.kp_ * e_last_.y_ + cfg_.ki_ * e_int_.y_;
        omega.z_ += cfg_.kp_ * e_last_.z_ + cfg_.ki_ * e_int_.z_;
    }
    q_.integrate(omega, dt);
}

void Mahony::observe(const math::Vec3f& acc, float dt) {
    math::Vec3f a = acc;
    a.normalize();

    math::Vec3f g_world(0, 0, 1);
    math::Vec3f v = q_.conjugated().rotate(g_world);

    math::Vec3f e = a.cross(v);

    e_int_.x_ += e.x_ * dt;
    e_int_.y_ += e.y_ * dt;
    e_int_.z_ += e.z_ * dt;
    e_int_.x_ = math::clamp(e_int_.x_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.y_ = math::clamp(e_int_.y_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.z_ = math::clamp(e_int_.z_, -cfg_.integral_limit_, cfg_.integral_limit_);

    e_last_ = e;
    is_meas_ = true;
}

void Mahony::update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt) {
    observe(acc, dt);
    predict(gyro, dt);
}

math::Vec3f Mahony::euler() const {
    float roll, pitch, yaw;
    q_.to_euler(roll, pitch, yaw);
    return math::Vec3f(roll, pitch, yaw);
}




}