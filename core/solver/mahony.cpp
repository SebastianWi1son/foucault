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
    // 【统一消费口】v̂ = q̂*·(0,0,1)：重力方向在机体系里的表达，本周期只算一次，两个通道共用
    //   它是【纯状态投影】——只由 q̂ 决定，不需要任何量测，所以谁先谁后都拿得到同一个值
    const math::Vec3f v = q_.conjugated().rotate(k_gravity_world);
    // 【统一出口】每通道产出一个角速度修正项，通道不在线则为 0（加法幺元）
    //   Mahony 的架构特性：所有修正都折进角速度 → 没有独立的 correct 步骤
    const math::Vec3f omega = gyro
                          + correction_acc(v, dt)
                          + correction_heading(v);

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
    // 首次自动兜底：忘了手动对齐时航向通道也能工作（否则会静默失效）
    if (!is_heading_aligned_) { align_heading(heading_ref); }
    heading_ref_ = heading_ref;     // ★ 必须：让本周期目标自洽（否则拿旧 ref 算残差 → 对齐瞬间跳变）
    heading_trust_ = math::clamp(trust, 0.0f, 1.0f);
    heading_age_ = 0.0f;            // heading_age_ 唯一清零口
}

void Mahony::align_heading(float heading_ref) {
    float roll, pitch, psi;
    q_.to_euler(roll, pitch, psi);
    heading_offset_ = math::wrap_pi(psi - heading_ref);
    heading_ref_ = heading_ref;
    is_heading_aligned_ = true;
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

math::Vec3f Mahony::correction_acc(const math::Vec3f& v, float dt) {
    // Gating
    if (!is_acc_valid()) { return math::Vec3f(0.0f, 0.0f, 0.0f); }
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
    return math::Vec3f(cfg_.kp_ * e_acc.x_ + cfg_.ki_ * e_int_.x_,
                       cfg_.kp_ * e_acc.y_ + cfg_.ki_ * e_int_.y_,
                       cfg_.kp_ * e_acc.z_ + cfg_.ki_ * e_int_.z_);
}

math::Vec3f Mahony::correction_heading(const math::Vec3f& v) {
    // Gating
    if (!is_heading_valid()) { return math::Vec3f(0.0f, 0.0f, 0.0f); }
    // 航向残差 = 参考 − 估计；必须 wrap 到 [−π,π] 再限幅（每周期用当前 q̂ 现算，不得冻结）
    float roll, pitch, psi;
    q_.to_euler(roll, pitch, psi);
    float e_heading = math::wrap_pi(heading_ref_ + heading_offset_ - psi);
    e_heading = math::clamp(e_heading, -k_half_pi, k_half_pi);
    // 绕世界z轴转 == body系中沿重力方向 v 加角速度（v 由上面的统一消费口给出，本周期只算一次）
    const float k = cfg_.kp_heading_ * heading_trust_ * e_heading;      // 增益
    // 该通道的角速度修正项 = 大小 k 沿重力轴 v̂（纯 P）
    return math::Vec3f(k * v.x_, k * v.y_, k * v.z_);
}

}