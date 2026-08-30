# CODE.md — 代码施工文档（边抄边学）

> **工作流**：本文档 = 代码的唯一施工来源。用户照着敲进实际源文件，边敲边理解边提问；AI 负责讲解、复查、出下一批。
> 批次内容可为**真实代码**或**伪代码**（由用户按学习阶段选择）。
> 抄完必须跑测试：**改完必回归**，ALL PASS 才算完成本批。

## 当前文件结构（2026-08-30，批次 1+2+3 ✅ 已验收）

```
foucault/
├── AGENT.md                    # 代理规则（含本文档索引）
├── CMakeLists.txt              # ✅ AI 编写；test_math/mahony/estimator + replay_nav2 四 target
├── core/
│   ├── config.hpp              # ✅ 批次 3 已抄录（Scene + make_mahony_config）
│   ├── estimator.hpp           # ✅ 批次 3 已抄录（门面，模板插拔；抄时拼成 estimater 已改名）
│   ├── math/
│   │   ├── scalar_ops.hpp      # ✅ AI 修复完成（2026-08-30）：删残留行 + snake_case
│   │   ├── vec3.hpp            # ✅ AI 修复完成：namespace/三参数构造/normalize return/cross 公式
│   │   └── quat.hpp            # ✅ AI 补齐：namespace/成员统一尾下划线/rotate/integrate/euler/Quatf
│   ├── measure/
│   │   └── measure.hpp         # ✅ 批次 3 已抄录（IMUSample，acc_/gyro_）
│   └── solver/
│       ├── mahony.hpp          # ✅ 已抄录（is_meas_/constexpr 用户改进，已同步文档）
│       └── mahony.cpp          # ✅ 已抄录（update 签名 2 处笔误已修；kp_/ki_ 成员名已对齐）
├── host/
│   └── replay/
│       └── replay_nav2.cpp     # ✅ AI 已写（NAV2 回放，真实数据验收 PASS）
├── tests/
│   └── unit/
│       ├── test_math.cpp       # ✅ AI 已写（21 项锚点）；批次 1 已验收全绿（2026-08-30）
│       ├── test_mahony.cpp     # ✅ AI 已写（4 项锚点）；批次 2 已验收全绿（2026-08-30）
│       └── test_estimator.cpp  # ✅ AI 已写（4 项锚点）；批次 3 已验收全绿（2026-08-30）
├── docs/                       # 文档体系（RESEARCH/DESIGN/DEV/复盘/待办/本文件）
├── reference/                  # 参考库（只读）：CtrBoard / gaochq / MahonyAHRS / 课程工程
├── Mahony-ICM42688-MMC5983/    # 课程工程（建议挪入 reference/）
└── .idea/                      # CLion 工程文件

约定：core 头文件统一用 .hpp（用户选择，CLion 惯例）；代码风格 snake_case + 成员尾下划线（用户定案 2026-08-30，F12 更新）；
更新规则：每批次新增/完成文件时，同步更新本结构树。
```

## 使用说明

1. 按批次顺序抄录；每个文件开头有 `文件：<路径>`，建好目录再敲
2. 抄完一批 → 编译跑测试 → 把输出贴给 AI 确认 → 再进下一批
3. 任何一行看不懂，直接对话问，不要憋着
4. 声明即承诺：文档里给出的 API 都会在后续批次用到；不用的不写

---

## 批次 1：core/math 数学内核（真实代码 ✅ 已在本机编译验证）

**目标**：`vec3.hpp` + `quat.hpp` + `scalar_ops.hpp` + 行为测试，共 4 个文件。

**抄前必懂（3 条，对应已学内容）**：
1. **四元数 = 轴角编码**：`q = (cos(θ/2), sin(θ/2)·n̂)` —— 所以 `fromEuler` 里全是 `θ/2`，`rotate` 两侧各乘一次（`v' = q⊗v⊗q*`）；
2. **integrate = 每拍注入增量角**：`q += ½·q⊗(0,ω)·dt` 展开成 4 行标量式，就是参考库 Mahony 的同款式子；ω 是**机体系**角速度（rad/s）；
3. **单位模长是硬约束**：积分后必须归一化，否则旋转会"变形"（测试 5 就是验这个）。

**约定**：命名空间 `foucault::math`（子命名空间分层，F10）；标量类型模板化（`Vec3<float>`，可用别名 `Vec3f`/`Quatf`）；header-only（数学内核允许的例外）；零 STL 依赖（F9）。
**代码风格（用户定案 2026-08-30）**：函数/变量 snake_case（`inv_sqrt`/`deg_to_rad`/`from_euler`）；成员统一尾下划线（`x_`/`q0_`，F12 更新）。
**测试归属**：测试文件一律由 AI 编写（用户指示），用户无需抄录 test 文件。
**决策编号**：F1~F12 见 DESIGN.md §5；代码注释中 `F#` 即对应决策，改代码前先看对应决策（F11）。

### 文件 1：core/math/scalar_ops.hpp

```cpp
#pragma once
// foucault core/math: scalar_ops.hpp —— 标量运算与平台钩子
// 零依赖，仅标准库头文件（F9）；float 快路径面向 H7 目标平台（F1）
#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace foucault::math {

// 快速倒数开方 1/sqrt(x_)
// float：经典位魔法（0x5f3759df）+ 两轮牛顿迭代，MCU 上比除法快
// double：直接走标准库（double 用得少，不值得优化）
template <typename T>
inline T inv_sqrt(T x_)
{
    if constexpr (std::is_same_v<T, float>)
    {
        float y_ = x_;
        std::int32_t i;
        std::memcpy(&i, &y_, sizeof(i));        // 把 float 的位模式当整数读
        i = 0x5f3759df - (i >> 1);             // 魔法常数：一次好的初始猜测
        std::memcpy(&y_, &i, sizeof(y_));
        y_ = y_ * (1.5f - 0.5f * x_ * y_ * y_);     // 牛顿迭代第 1 轮
        y_ = y_ * (1.5f - 0.5f * x_ * y_ * y_);     // 牛顿迭代第 2 轮
        return y_;
    }
    else
    {
        return T(1) / std::sqrt(x_);
    }
}

template <typename T>
inline T clamp(T v, T lo, T hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

template <typename T>
inline T deg_to_rad(T deg) { return deg * T(0.017453292519943295); }  // π/180

template <typename T>
inline T rad_to_deg(T rad) { return rad * T(57.29577951308232); }     // 180/π

} // namespace foucault::math
```

### 文件 2：core/math/vec3.hpp

```cpp
#pragma once
// foucault core/math: vec3.hpp —— 三维向量（POD 值类型，成员无尾下划线，F12）
#include <cmath>

namespace foucault::math {

template <typename T>
struct Vec3
{
    T x_, y_, z_;

    constexpr Vec3() : x_(0), y_(0), z_(0) {}
    constexpr Vec3(T x_, T y_, T z_) : x_(x_), y_(y_), z_(z_) {}
    constexpr Vec3(T fill) : x_(fill), y_(fill), z_(fill) {}  // 标量填充

    T& operator[](int i)
    {
        switch (i)
        {
        case 0:  return x_;
        case 1:  return y_;
        default: return z_;
        }
    }
    const T& operator[](int i) const
    {
        switch (i)
        {
        case 0:  return x_;
        case 1:  return y_;
        default: return z_;
        }
    }

    Vec3 operator+(const Vec3& o) const { return Vec3(x_ + o.x_, y_ + o.y_, z_ + o.z_); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x_ - o.x_, y_ - o.y_, z_ - o.z_); }
    Vec3 operator-() const { return Vec3(-x_, -y_, -z_); }
    Vec3 operator*(T s) const { return Vec3(x_ * s, y_ * s, z_ * s); }
    Vec3 operator/(T s) const { return Vec3(x_ / s, y_ / s, z_ / s); }

    Vec3& operator+=(const Vec3& o) { x_ += o.x_; y_ += o.y_; z_ += o.z_; return *this; }
    Vec3& operator-=(const Vec3& o) { x_ -= o.x_; y_ -= o.y_; z_ -= o.z_; return *this; }
    Vec3& operator*=(T s) { x_ *= s; y_ *= s; z_ *= s; return *this; }

    T dot(const Vec3& o) const { return x_ * o.x_ + y_ * o.y_ + z_ * o.z_; }

    // 叉积：结果 = 垂直两向量的第三个方向，模长 = 两向量夹角正弦 × 模长之积
    Vec3 cross(const Vec3& o) const
    {
        return Vec3(y_ * o.z_ - z_ * o.y_,
                    z_ * o.x_ - x_ * o.z_,
                    x_ * o.y_ - y_ * o.x_);
    }

    T norm_squared() const { return x_ * x_ + y_ * y_ + z_ * z_; }
    T norm() const { return std::sqrt(norm_squared()); }

    // 原地归一化；零向量保持零向量（防御：避免除以 0）
    Vec3& normalize()
    {
        T n = norm();
        if (n > T(0))
        {
            T inv = T(1) / n;
            x_ *= inv; y_ *= inv; z_ *= inv;
        }
        return *this;
    }
};

template <typename T>
inline Vec3<T> operator*(T s, const Vec3<T>& v) { return v * s; }

using Vec3f = Vec3<float>;

} // namespace foucault::math
```

### 文件 3：core/math/quat.hpp

```cpp
#pragma once
// foucault core/math: quat.hpp —— 四元数（姿态表示 + 运动学积分）
//
// 约定：
//   · q = (q0_, q1_, q2_, q3_) = (w, x, y, z)
//   · 轴角编码：q = (cos(θ/2), sin(θ/2)·n̂)，绕轴 n̂ 转 θ
//   · 单位模长约束：积分后必须 normalize()
//   · 欧拉角采用内旋 ZYX（yaw → pitch → roll），弧度制
//   · 陀螺角速度一律视为机体系（body frame）读数
//   · 单位 rad / rad/s，命名中性不绑单位（F7）
#include <cmath>
#include "scalar_ops.hpp"
#include "vec3.hpp"

namespace foucault::math {

template <typename T>
struct Quat
{
    T q0_, q1_, q2_, q3_;

    constexpr Quat() : q0_(1), q1_(0), q2_(0), q3_(0) {}   // 单位元：零旋转
    constexpr Quat(T w, T x, T y, T z) : q0_(w), q1_(x), q2_(y), q3_(z) {}

    static Quat identity() { return Quat(1, 0, 0, 0); }

    T norm_squared() const { return q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_; }
    T norm() const { return std::sqrt(norm_squared()); }

    // 归一化：积分后必做（模长漂离 1 会让旋转"变形"）
    Quat& normalize()
    {
        T n2 = norm_squared();
        if (n2 > T(0))
        {
            T inv = inv_sqrt(n2);
            q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
        }
        else
        {
            *this = identity();   // 防御：零四元数无意义，复位为单位元
        }
        return *this;
    }

    // 共轭 = 单位四元数的逆（反向旋转）
    Quat conjugated() const { return Quat(q0_, -q1_, -q2_, -q3_); }

    // Hamilton 积：q * r 表示"先 r 后 q"的合成旋转
    Quat operator*(const Quat& r) const
    {
        return Quat(
            q0_ * r.q0_ - q1_ * r.q1_ - q2_ * r.q2_ - q3_ * r.q3_,
            q0_ * r.q1_ + q1_ * r.q0_ + q2_ * r.q3_ - q3_ * r.q2_,
            q0_ * r.q2_ - q1_ * r.q3_ + q2_ * r.q0_ + q3_ * r.q1_,
            q0_ * r.q3_ + q1_ * r.q2_ - q2_ * r.q1_ + q3_ * r.q0_);
    }

    // 旋转向量：v' = q ⊗ (0,v) ⊗ q*
    // （两侧各乘一次 → 半角编码的原因；下面是快速展开，不做完整两次乘法）
    Vec3<T> rotate(const Vec3<T>& v) const
    {
        // 快速公式：v' = v + 2·q0_·(qv×v) + 2·qv×(qv×v)，其中 qv = (q1_,q2_,q3_)
        Vec3<T> qv(q1_, q2_, q3_);
        Vec3<T> t = qv.cross(v) * T(2);
        return v + t * q0_ + qv.cross(t);
    }

    // 陀螺积分（一阶欧拉，= 一阶毕卡 = RK1）：
    //   q̇ = ½·q⊗(0,ω)  →  q += ½·q⊗(0,ω)·dt  →  归一化
    // ω 必须是机体系角速度，单位 rad/s；dt 单位 s
    Quat& integrate(const Vec3<T>& omega, T dt)
    {
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
    static Quat from_euler(T roll, T pitch, T yaw)
    {
        T cr = std::cos(roll  * T(0.5)), sr = std::sin(roll  * T(0.5));
        T cp = std::cos(pitch * T(0.5)), sp = std::sin(pitch * T(0.5));
        T cy = std::cos(yaw   * T(0.5)), sy = std::sin(yaw   * T(0.5));
        return Quat(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy);
    }

    // 四元数 → 欧拉角（atan2/asin 直接公式，无需旋转矩阵），输出弧度
    void to_euler(T& roll, T& pitch, T& yaw) const
    {
        roll  = std::atan2(T(2) * (q0_ * q1_ + q2_ * q3_), T(1) - T(2) * (q1_ * q1_ + q2_ * q2_));
        pitch = std::asin(T(-2) * (q1_ * q3_ - q0_ * q2_));
        yaw   = std::atan2(T(2) * (q1_ * q2_ + q0_ * q3_), T(1) - T(2) * (q2_ * q2_ + q3_ * q3_));
    }
};

using Quatf = Quat<float>;

} // namespace foucault::math
```

### 文件 4：tests/unit/test_math.cpp（AI 已写，无需抄录）

测试由 AI 直接编写在 `tests/unit/test_math.cpp`（21 项行为锚点：标量运算 7 项、Vec3 7 项、Quat 7 项，含零向量/零四元数防御路径）。
你不抄这个文件，但**验收标准不变**：修完文件 1~3 后跑测试，输出 21 PASS + ALL PASS 才算批次 1 完成。

### 编译与验收

CMake 已配好（`CMakeLists.txt`，AI 编写），构建测试一条龙：

```bash
cd /home/wilson/Dev/Workspace/foucault
mkdir -p core/math tests/unit        # 按上面 4 个文件路径敲入代码
cmake -B build -S .                 # 首次配置（之后只需 --build）
cmake --build build
ctest --test-dir build --output-on-failure
```

或单文件快速验证（不想用 CMake 时）：

```bash
g++ -std=c++17 -Wall -Wextra -Werror -I core tests/unit/test_math.cpp -o /tmp/test_math
/tmp/test_math
```

**验收标准（2026-08-30 更新）**：输出 21 行 `[PASS]` + `ALL PASS`，退出码 0。
已在 /tmp 验证通过（2026-08-30）：g++ 13 / C++17 / -Werror 零告警，21 项 PASS（含位魔法精度实测 ~4.4e-6 相对误差，容差 1e-5）。

**看测试输出时对照理解**：
- rotate：绕 z 转 90°，x 轴 → y 轴（二重覆盖：v' = q⊗v⊗q*）
- integrate：就是"每拍注入增量角" —— 1 rad/s 转 1 秒 ≈ yaw 1 rad，且模长始终为 1
- 欧拉往返：pitch 远离 ±90° 时精确；靠近万向锁会失真 —— 这正是四元数的意义

---

## 批次 2：core/solver/mahony（真实代码 ✅ 已在 /tmp 编译验证，2026-08-30）

**目标**：`mahony.hpp` + `mahony.cpp` 两个文件（用户抄录）；`test_mahony.cpp` 已由 AI 写好，CMake target 已就位。

**抄前必懂（对应刚学的课程）**：
1. **本质公式实例**：`q̂̇ = ½·q̂⊗(ω−b̂) + K·(z−h(q̂))`。Mahony 把修正项**折进角速度**：
   `ω_corrected = ω + Kp·e + Ki·∫e`，然后走批次 1 的 `integrate()`；
2. **残差 e = 实测 × 预测**：加速度计实测方向 × 预测重力方向（叉积，小角度下 ∝ 姿态误差角）；
3. **Ki·∫e = 零偏估计**（Mahony 版 b̂）：积分项限幅防饱和；
4. **异步多速率（F4）**：`predict` 跟 IMU 速率（用最近一次残差），`observe` 事件驱动（量测到了才算残差）；
5. 类内成员用尾下划线（`q_`/`e_int_`），Config 聚合（foc 范式）。

**先决条件**：批次 1 必须全绿（21 PASS）——本批用到 `rotate`（共轭旋转）、`integrate`、`normalize`、`cross`、`clamp`、`to_euler`。

### 文件 5：core/solver/mahony.hpp

```cpp
#pragma once
// foucault core/solver: mahony.hpp —— Mahony 互补滤波器（增益求解器之一，float）
//
// 本质公式实例：q̂̇ = ½·q̂⊗(ω−b̂) + K·(z−h(q̂))
//   Mahony 把修正项折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
//   e = 实测加速度方向 × 预测重力方向（叉积残差，小角度下 ∝ 姿态误差角）
//   Ki·∫e ≈ 陀螺零偏估计（Mahony 版 b̂）
// 参考：MahonyAHRS.c（PaulStoffregen）/ Mahony-ICM42688-MMC5983 课程工程
#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {

// 配置聚合（foc 范式）：参数集中，默认值 = 常用起步值
struct MahonyConfig
{
    float kp_ = 5.0f;               // 比例增益：残差 → 角速度修正的力度
    float ki_ = 0.3f;               // 积分增益：累积残差（估零偏）
    float integral_limit_ = 10.0f;  // 积分项限幅（防饱和，rad/s）
};

class Mahony
{
public:
    constexpr Mahony() = default;
    explicit Mahony(const MahonyConfig& cfg) : cfg_(cfg) {}

    void reset();                      // 姿态复位为单位元，积分项清零
    void reset(const math::Quatf& q);  // 指定初始姿态（初始对准，F5）

    // F4 异步多速率：predict 跟 IMU 速率（用最近一次残差），observe 事件驱动
    void predict(const math::Vec3f& gyro, float dt);
    void observe(const math::Vec3f& acc, float dt);
    void update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt);  // 经典单次调用

    const math::Quatf& quaternion() const { return q_; }
    math::Vec3f euler() const;

private:
    MahonyConfig cfg_;
    math::Quatf q_;         // 姿态状态（估算器的输出）
    math::Vec3f e_int_;     // 积分项（零偏估计）
    math::Vec3f e_last_;    // 最近一次残差（predict 时复用）
    bool is_meas_ = false;  // 是否已收到量测（首次 predict 前无修正）
};

} // namespace foucault::solver

```

### 文件 6：core/solver/mahony.cpp

```cpp
#include "mahony.hpp"
#include "../math/scalar_ops.hpp"

namespace foucault::solver {

void Mahony::reset()
{
    q_ = math::Quatf::identity();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    is_meas_ = false;
}

void Mahony::reset(const math::Quatf& q)
{
    q_ = q;
    q_.normalize();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    is_meas_ = false;
}

void Mahony::predict(const math::Vec3f& gyro, float dt)
{
    math::Vec3f omega = gyro;
    if (is_meas_)
    {
        // 修正折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
        omega.x_ += cfg_.kp_ * e_last_.x_ + cfg_.ki_ * e_int_.x_;
        omega.y_ += cfg_.kp_ * e_last_.y_ + cfg_.ki_ * e_int_.y_;
        omega.z_ += cfg_.kp_ * e_last_.z_ + cfg_.ki_ * e_int_.z_;
    }
    q_.integrate(omega, dt);
}

void Mahony::observe(const math::Vec3f& acc, float dt)
{
    // 1) 量测归一化：只留方向（加速度计大小含运动噪声，方向才是信息）
    math::Vec3f a = acc;
    a.normalize();

    // 2) 预测：当前姿态下，重力在机体系应指向哪 —— h(q̂) = R(q̂)ᵀ·g
    const math::Vec3f g_world(0, 0, 1);            // 世界系重力（归一化后）
    math::Vec3f v = q_.conjugated().rotate(g_world);

    // 3) 残差：实测 × 预测（叉积，小角度下模长 ∝ 姿态误差角）
    math::Vec3f e = a.cross(v);

    // 4) 积分项累加（零偏估计）+ 限幅（防饱和）
    e_int_.x_ += e.x_ * dt;
    e_int_.y_ += e.y_ * dt;
    e_int_.z_ += e.z_ * dt;
    e_int_.x_ = math::clamp(e_int_.x_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.y_ = math::clamp(e_int_.y_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.z_ = math::clamp(e_int_.z_, -cfg_.integral_limit_, cfg_.integral_limit_);

    e_last_ = e;
    is_meas_ = true;
}

void Mahony::update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt)
{
    observe(acc, dt);
    predict(gyro, dt);
}

math::Vec3f Mahony::euler() const
{
    float roll, pitch, yaw;
    q_.to_euler(roll, pitch, yaw);
    return math::Vec3f(roll, pitch, yaw);
}

} // namespace foucault::solver

```

### 文件 7：tests/unit/test_mahony.cpp（AI 已写，无需抄录）

AI 已写好 4 项行为锚点：静止保持零姿态、横滚 30° 加速度计收敛、纯陀螺积分 2s yaw≈2rad、reset 回零。
验收：`cmake --build build && ctest --test-dir build --output-on-failure` → 两个测试全过。

**批次预告**：批次 3 = measure + estimator 门面 + config（把 Mahony 装进统一接口，数据集回放验证开始）；批次 4 = EKF（先补 Level 3 理论：状态空间/雅可比/可观测性/Q-R/卡方）。
---

## 批次 3：measure + config + estimator 门面（真实代码 ✅ 已在 /tmp 验证，2026-08-30）

**目标**：三个文件（用户敲）：`core/measure/measure.hpp`、`core/config.hpp`、`core/estimator.hpp`。
测试 `test_estimator.cpp` 与回放工具 `host/replay/replay_nav2.cpp` 由 AI 直接写（你无需抄）。

**抄前必懂（门面 = 统一入口）**：
1. **门面（Facade）**：给上层一个不变接口（predict/observe/reset/euler），算法怎么实现被藏起来。
   换算法只换模板参数：`Estimator<solver::Mahony>` → `Estimator<solver::EKF>`（批次 4），调用方代码一行不改；
2. **模板默认参数**：`template <typename Solver = solver::Mahony>` —— 不写参数就是 Mahony；
3. **Scene 档位**：云台/小车/步兵三场景起步参数相同，真正分档在 EKF 的 Q/R（批次 4）；
4. **IMUSample**：把"一次传感器采样"打包成一个结构（acc 单位 g，gyro 单位 rad/s）；
5. **observe_yaw 占位**：外部 yaw 注入接口（F4 设计），Mahony 不支持 → 空函数忽略，EKF 批实现。

**先决条件**：批次 2 全绿（test_mahony 4 PASS）。

### 文件 7：core/measure/measure.hpp

```cpp
#pragma once
// foucault core/measure: measure.hpp —— 量测输入结构（本批最小化，仲裁留 EKF 批）
#include "../math/vec3.hpp"

namespace foucault::measure {

// 一次 IMU 采样（机体系；acc 单位 g，gyro 单位 rad/s）
struct IMUSample
{
    math::Vec3f acc_;     // 加速度计（g）：方向指向"上"的反向 = 重力
    math::Vec3f gyro_;    // 陀螺（rad/s）：机体系角速度
};

} // namespace foucault::measure
```

### 文件 8：core/config.hpp

```cpp
#pragma once
// foucault core/config.hpp —— 配置聚合 + 场景档位（F 级起步值，回放校准后更新）
#include "solver/mahony.hpp"

namespace foucault {

// 场景档位：云台（低动态高精度）/ 地面小车（平面为主）/ 立体步兵（高动态）
enum class Scene
{
    gimbal,     // 云台
    car,        // 地面小车
    infantry,   // 立体步兵
};

// 场景 → Mahony 配置（当前三档同起步值；真正分档在 EKF 的 Q/R，批次 4）
inline solver::MahonyConfig make_mahony_config(Scene scene)
{
    (void)scene;   // 本批三档同参数，EKF 批启用分档
    solver::MahonyConfig cfg;
    cfg.kp_ = 5.0f;               // 比例增益（残差 → 角速度修正力度）
    cfg.ki_ = 0.3f;               // 积分增益（零偏估计）
    cfg.integral_limit_ = 10.0f;  // 积分限幅（防饱和，rad/s）
    return cfg;
}

} // namespace foucault
```

### 文件 9：core/estimator.hpp

```cpp
#pragma once
// foucault core/estimator.hpp —— 估算器门面（统一接口，可插拔增益求解器）
//
// 设计（F10/F4）：模板参数 = 增益求解器策略（Mahony 现在，EKF 批次 4）；
// 统一入口 predict/observe/observe_yaw；求解器不支持的量测自动忽略。
#include "config.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "measure/measure.hpp"
#include "solver/mahony.hpp"

namespace foucault {

template <typename Solver = solver::Mahony>
class Estimator
{
public:
    // 用场景档位构造（内部转换为对应求解器配置）
    explicit Estimator(Scene scene = Scene::gimbal)
        : solver_(make_mahony_config(scene))
    {
    }

    // F4 异步多速率：predict 跟 IMU 速率，observe 事件驱动
    void predict(const measure::IMUSample& s, float dt) { solver_.predict(s.gyro_, dt); }
    void observe(const measure::IMUSample& s, float dt) { solver_.observe(s.acc_, dt); }

    // 外部 yaw 注入（F4 设计）：Mahony 不支持，忽略；EKF 批实现
    void observe_yaw(float /*yaw_rad*/, float /*dt*/) {}

    void reset() { solver_.reset(); }
    void reset(const math::Quatf& q) { solver_.reset(q); }

    const math::Quatf& quaternion() const { return solver_.quaternion(); }
    math::Vec3f euler() const { return solver_.euler(); }

private:
    Solver solver_;
};

} // namespace foucault
```

### 文件 10/11：测试与回放工具（AI 已写，无需抄录）

- `tests/unit/test_estimator.cpp`：4 项锚点（门面与直接调用 Mahony 等价、observe_yaw 无害、reset 指定姿态、Scene 构造）；
- `host/replay/replay_nav2.cpp`：读参考库 NAV2 数据集（文本 12 列：acc3+gyro3+mag3+真值 euler3，弧度，50Hz）→ 跑 `Estimator<solver::Mahony>` → 输出 roll/pitch 的 RMSE/MAX（度）与 yaw 漂移量，可选导出 CSV；
- **验收标准**：test_estimator 4 PASS + replay 报告 roll/pitch RMSE 有界（预期 < 5°）；**yaw 漂移是 6 轴预期行为**（无磁力计，yaw 不可观），报告里记录漂移量即可，不算失败。

**批次预告**：批次 4 = EKF（先补 Level 3 理论：状态空间/雅可比/可观测性/Q-R/卡方，再动手）；届时 `Estimator<solver::EKF>` 一行切换。
