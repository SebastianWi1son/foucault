#include "mahony.hpp"

#include "../math/scalar_ops.hpp"

namespace foucault::solver {


namespace {
    constexpr math::Vec3f k_gravity_world(0.0f, 0.0f, 1.0f);      // 世界系重力方向
    constexpr float k_half_pi = 1.5707963267948966f;                    // 航向残差限幅
}

// ----- constructor -----
Mahony::Mahony(const MahonyConfig& cfg) : cfg_(cfg) {}

// ----- API -----
void Mahony::reset() {
    q_ = math::Quatf::identity();
    e_int_ = math::Vec3f(0, 0, 0);
    acc_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;
    reset_heading_channel();
}

void Mahony::reset(const math::Quatf& q) {
    q_ = q;
    q_.normalize();
    e_int_ = math::Vec3f(0, 0, 0);
    acc_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;
    reset_heading_channel();
}

void Mahony::predict(const math::Vec3f& gyro, float dt) {
    math::Vec3f omega = gyro;

    // 【统一消费口】v̂ = q̂*·(0,0,1)：重力方向在机体系里的表达，本周期只算一次，两个通道共用
    //   它是【纯状态投影】——只由 q̂ 决定，不需要任何量测，所以谁先谁后都拿得到同一个值
    const math::Vec3f v = q_.conjugated().rotate(k_gravity_world);

    if (is_acc_valid()) {
        // 算error
        const math::Vec3f e_acc = acc_.cross(v);
        // error integral (also zero offset)
        e_int_.x_ += e_acc.x_ * dt;
        e_int_.y_ += e_acc.y_ * dt;
        e_int_.z_ += e_acc.z_ * dt;
        // integral limit
        e_int_.x_ = math::clamp(e_int_.x_, -cfg_.integral_limit_, cfg_.integral_limit_);
        e_int_.y_ = math::clamp(e_int_.y_, -cfg_.integral_limit_, cfg_.integral_limit_);
        e_int_.z_ = math::clamp(e_int_.z_, -cfg_.integral_limit_, cfg_.integral_limit_);
        // 修正折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
        omega.x_ += cfg_.kp_ * e_acc.x_ + cfg_.ki_ * e_int_.x_;
        omega.y_ += cfg_.kp_ * e_acc.y_ + cfg_.ki_ * e_int_.y_;
        omega.z_ += cfg_.kp_ * e_acc.z_ + cfg_.ki_ * e_int_.z_;
    }
    if (is_heading_valid()) {
        // 航向残差 = 参考 − 估计；必须 wrap 到 [−π,π] 再限幅（每周期用当前 q̂ 现算，不得冻结）
        float roll, pitch, psi;
        q_.to_euler(roll, pitch, psi);
        float e_heading = math::wrap_pi(heading_ref_ + heading_offset_ - psi);
        e_heading = math::clamp(e_heading, -k_half_pi, k_half_pi);
        // 绕世界z轴转 == body系中沿重力方向 v 加角速度（v 由上面的统一消费口给出，本周期只算一次）
        const float k = cfg_.kp_heading_ * heading_trust_ * e_heading;      // 增益
        // 修正折进v
        omega.x_ += k * v.x_;
        omega.y_ += k * v.y_;
        omega.z_ += k * v.z_;
    }
    acc_age_ += dt;
    heading_age_ += dt;

    q_.integrate(omega, dt);
}

void Mahony::observe(const math::Vec3f& acc) {
    acc_ = acc;
    acc_.normalize();                                         // only fetch direction
    acc_age_ = 0.0f;                                       // acc_age_唯一清零口
}

void Mahony::observe_heading(float heading_ref, float trust) {
    if (!is_heading_aligned_) {
        float roll, pitch, psi;
        q_.to_euler(roll, pitch, psi);
        heading_offset_ = math::wrap_pi(psi - heading_ref);
        is_heading_aligned_ = true;
    }
    heading_ref_ = heading_ref;
    heading_trust_ = math::clamp(trust, 0.0f, 1.0f);
    heading_age_ = 0.0f;            // heading_age_ 唯一清零口
}

void Mahony::update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt) {
    observe(acc);
    predict(gyro, dt);
}

// ----- ref valid check -----
bool Mahony::is_acc_valid() const { return acc_age_ <= cfg_.acc_timeout_; }
bool Mahony::is_heading_valid() const { return heading_age_ <= cfg_.heading_timeout_; }

// ----- typical output -----
const math::Quatf& Mahony::quaternion() const { return q_; }

math::Vec3f Mahony::euler() const {
    float roll, pitch, yaw;
    q_.to_euler(roll, pitch, yaw);
    return math::Vec3f(roll, pitch, yaw);
}

// ----- private -----
void Mahony::reset_heading_channel() {
    heading_ref_ = 0.0f;
    heading_offset_ = 0.0f;
    heading_age_ = k_never_measured_;
    heading_trust_ = 1.0f;
    is_heading_aligned_ = false;
}



}