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
│   ├── config.hpp              # ✅ 批次 3 已抄录（Dimension + make_mahony_config，2026-08-30 用户定案改）
│   ├── estimator.hpp           # ✅ 批次 3 已抄录（门面，模板插拔；抄时拼成 estimater 已改名）
│   ├── math/
│   │   ├── scalar_ops.hpp      # ✅ AI 修复完成（2026-08-30）：删残留行 + snake_case
│   │   ├── vec3.hpp            # ✅ AI 修复完成：namespace/三参数构造/normalize return/cross 公式
│   │   └── quat.hpp            # ✅ AI 补齐：namespace/成员统一尾下划线/rotate/integrate/euler/Quatf
│   ├── measure/
│   │   └── measure.hpp         # ✅ 批次 3 已抄录（IMUSample，acc_/gyro_）
│   └── solver/
│       ├── mahony.hpp          # ✅ 已抄录；【批次 4a-1 待重抄】is_meas_ → acc_age_ 年龄计数器
│       └── mahony.cpp          # ✅ 已抄录；【批次 4a-1 待重抄】同上（3 处改动）
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

// 快速倒数开方 1/sqrt(x)
// float：经典位魔法（0x5f3759df）+ 两轮牛顿迭代，MCU 上比除法快
// double：直接走标准库（double 用得少，不值得优化）
template <typename T>
inline T inv_sqrt(T x)
{
    if constexpr (std::is_same_v<T, float>)
    {
        float y = x;
        std::int32_t i;
        std::memcpy(&i, &y, sizeof(i));        // 把 float 的位模式当整数读
        i = 0x5f3759df - (i >> 1);             // 魔法常数：一次好的初始猜测
        std::memcpy(&y, &i, sizeof(y));
        y = y * (1.5f - 0.5f * x * y * y);     // 牛顿迭代第 1 轮
        y = y * (1.5f - 0.5f * x * y * y);     // 牛顿迭代第 2 轮
        return y;
    }
    else
    {
        return T(1) / std::sqrt(x);
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
// foucault core/math: vec3.hpp —— 三维向量（值类型，成员统一尾下划线，F12）
#include <cmath>

namespace foucault::math {

template <typename T>
struct Vec3
{
    T x_, y_, z_;

    constexpr Vec3() : x_(0), y_(0), z_(0) {}
    constexpr Vec3(T x, T y, T z) : x_(x), y_(y), z_(z_) {}   // 参数不带 _，只有成员带 _
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
    // 异常输入：NaN 时【不做处理】，脏值原样传播（防御只应对付它认识的那一种输入）
    // 旧版 `else { *this = identity(); }` 会把 NaN 静默洗成单位元 → 单帧脏量测永久废掉姿态
    //（+Inf 走 if 分支 → 结果非有限值，明显异常可检出）
    Quat& normalize()
    {
        T n2 = norm_squared();
        if (n2 > T(0))
        {
            T inv = inv_sqrt(n2);
            q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
        }
        else if (n2 == T(0))
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
    // ⚠ 快速展开式【仅当 |q| = 1】时等于 q⊗(0,v)⊗q*；非单位四元数下两者相差可观
    //   （实测 |q|²=1.09 时输出模长 1.0161，而精确乘积应为 1.09）→ rotate 前必须归一化
    Vec3<T> rotate(const Vec3<T>& v) const
    {
        // 快速公式：v' = v + 2·q0_·(qv×v) + 2·qv×(qv×v)，其中 qv = (q1_,q2_,q3_)
        Vec3<T> qv(q1_, q2_, q3_);
        Vec3<T> t = qv.cross(v) * T(2);
        return v + t * q0_ + qv.cross(t);
    }

    // 陀螺积分（名义一阶欧拉 = 一阶毕卡 = RK1，实测量化后为全局二阶）：
    //   q̇ = ½·q⊗(0,ω)  →  q += ½·q⊗(0,ω)·dt  →  归一化
    // 二阶来源：归一化恰好消掉二阶项（纯四元数 x 有 x² = -|x|²），与 exp(½ω̂dt) 差 O(|x|³)
    // 实测观测阶 p = 2.04（常角速度）/ 1.98（时变 ω）；见 tests/unit/test_math_audit.cpp F3
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
5. 成员统一尾下划线（`q_`/`e_int_`），**struct / class 不作区分**（F12）；Config 聚合（foc 范式）。

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
3. **Dimension 档位**：d2/d25/d3 是数学抽象（状态空间大小），产品映射靠注释（地面小车→d2，云台/步兵→d3）；
   当前三档起步参数相同，真正分档在 EKF 的 Q/R（批次 4）与维度模式（批次 5）；
   （2026-08-30 用户定案：核心层只懂数学，产品语义属应用层，弃用产品枚举 Scene）
4. **IMUSample**：把"一次传感器采样"打包成一个结构（acc 单位 g，gyro 单位 rad/s）；
5. **observe_heading 占位**：外部 yaw 注入接口（F4 设计），Mahony 不支持 → 空函数忽略，EKF 批实现。

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
// foucault core/config.hpp —— 配置聚合 + 维度档位（F 级起步值，回放校准后更新）
#include "solver/mahony.hpp"

namespace foucault {

// 维度模式（数学抽象；核心层只懂数学，产品语义属应用层）
// 产品映射（注释即文档）：地面小车 → d2；云台/立体步兵 → d3
enum class Dimension
{
    d2,   // 2D：只估 yaw，平面运动（地面小车）—— 批次 5 落地
    d25,  // 2.5D：平面运动 + 倾角 —— 批次 5 落地
    d3    // 3D：全姿态（云台/立体步兵）—— 当前唯一实现
};

// 维度 → Mahony 配置（当前各档同起步值；分档在 EKF 的 Q/R 批次 4 + 维度模式批次 5）
inline solver::MahonyConfig make_mahony_config(Dimension dim)
{
    (void)dim;   // 本批各档同参数，EKF 批启用分档
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
// 统一入口 predict/observe/observe_heading；求解器不支持的量测自动忽略。
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
    explicit Estimator(Dimension dim = Dimension::d3)
        : solver_(make_mahony_config(dim))
    {
    }

    // F4 异步多速率：predict 跟 IMU 速率，observe 事件驱动
    void predict(const measure::IMUSample& s, float dt) { solver_.predict(s.gyro_, dt); }
    void observe(const measure::IMUSample& s, float dt) { solver_.observe(s.acc_, dt); }

    // 外部 yaw 注入（F4 设计）：Mahony 不支持，忽略；EKF 批实现
    void observe_heading(float /*heading_rad*/, float /*dt*/) {}

    void reset() { solver_.reset(); }
    void reset(const math::Quatf& q) { solver_.reset(q); }

    const math::Quatf& quaternion() const { return solver_.quaternion(); }
    math::Vec3f euler() const { return solver_.euler(); }

private:
    Solver solver_;
};

} // namespace foucault
```

---

## 批次 4a-1：修 P0-4 —— `is_meas_` 布尔闩锁 → 残差年龄计数器（2026-09-12 ✅ 已在 /tmp 编译+实测验证）

**目标**：`core/solver/mahony.hpp` + `mahony.cpp` 两个文件，**整文件重抄**（改动处已用【】标注）。

**为什么必须先修这个**：它是 `DESIGN §3.6.5 ④` 的最小落地，**也是批次 4a 里 `heading_age_` 的同一套机制** —— 先在小处做对，再复制到 yaw 通道，避免"设计对了但实现又写成布尔"。

**抄前必懂（三句话）**：

1. **旧代码的 bug（实测）**：`is_meas_` 一旦 `true` **永不回退**。加速度计在 t=100s 断线后，`predict` 仍然拿着**冻结的** `e_last_` 当"新鲜残差"用：

   ```
   ω += Kp · e_last_(冻结)   ← 一个恒定的假角速度（≈14°/s）
   q̂ → v̂ → e → e_last_      ← 这条反馈环【断了】（e 不再更新）
   ⇒ 匀加速跑飞
   ```

2. **年龄计数器的语义**：一个浮点数，同时覆盖两种"没有量测"的情况 —— 启动未对准（初值 = 哨兵值）和中途失效（随时间增长）。**旧的 bool 只能表达第一种。**

   ```
   observe 到来 → acc_age_ = 0          （残差新鲜）
   每个 predict → acc_age_ += dt        （时间在流逝）
   用之前判断   → acc_age_ <= acc_timeout_ 才用残差修正
   ```

3. **`+dt` 放在"判断之后"**：这样"本周期刚喂过 acc"时 `age_ = 0`，正好被用上；连续 N 个周期没喂，`age_ = N·dt` 如实累积。

**修前 / 修后实测对比（NAV2 320s，t=100s 断掉加速度计、`predict` 照跑）**：

| | 全程 RMSE | 断后 RMSE | 断后峰值 | 恶化到 >10° 用时 |
|---|---|---|---|---|
| 修前 `is_meas_` | 97.510° | **117.582°** | 185.169° | **1.48 s** |
| 修后 `acc_age_` | 4.954° | **5.541°** | 13.943° | **198.56 s**（≈纯陀螺自然漂移）|

**正常工况零退化**（ctest 5/5 全绿；回放 roll 2.152° / pitch 2.645° / yaw −14.450°，与修前**逐位一致**）。

### 文件 10：core/solver/mahony.hpp（整文件，重抄）

```cpp
#pragma once

#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {


struct MahonyConfig {
    float kp_ = 5.0f;
    float ki_ = 0.3f;
    float integral_limit_ = 10.0f;
    float acc_timeout_ = 0.1f;      // 【新】acc 残差有效期(s)：超过它视为"没有量测"（P0-4）
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
    // 【新】"从未收到过量测"的哨兵年龄：足够大（永远 > acc_timeout_），且 +dt 不溢出、不丧失精度
    static constexpr float k_never_measured_ = 1.0e6f;

    // 【新】量测是否新鲜 —— 取代原来的 bool is_meas_
    bool has_valid_acc() const { return acc_age_ <= cfg_.acc_timeout_; }

    MahonyConfig cfg_;
    math::Quatf q_;         // 姿态状态(估算器的输出)
    math::Vec3f e_int_;     // integral (零偏估计)
    math::Vec3f e_last_;    // 最近一次残差
    float acc_age_ = k_never_measured_;   // 【改】距上次有效 acc 量测的秒数（原为 bool 闩锁）
};




}
```

### 文件 11：core/solver/mahony.cpp（整文件，重抄）

```cpp
#include "mahony.hpp"

#include "../math/scalar_ops.hpp"

namespace foucault::solver {


void Mahony::reset() {
    q_ = math::Quatf::identity();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;       // 【改】重置为"从未有过量测"
}

void Mahony::reset(const math::Quatf& q) {
    q_ = q;
    q_.normalize();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;       // 【改】同上
}

void Mahony::predict(const math::Vec3f& gyro, float dt) {
    math::Vec3f omega = gyro;
    // 【改】bool 闩锁 → 年龄门控：残差【新鲜】才拿它修正
    if (has_valid_acc()) {
        // 修正折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
        omega.x_ += cfg_.kp_ * e_last_.x_ + cfg_.ki_ * e_int_.x_;
        omega.y_ += cfg_.kp_ * e_last_.y_ + cfg_.ki_ * e_int_.y_;
        omega.z_ += cfg_.kp_ * e_last_.z_ + cfg_.ki_ * e_int_.z_;
    }
    acc_age_ += dt;                     // 【新】时间推进：残差在不新鲜度上累积
    q_.integrate(omega, dt);
}

void Mahony::observe(const math::Vec3f& acc, float dt) {
    math::Vec3f a = acc;
    a.normalize();                                        // only fetch direction

    math::Vec3f g_world(0, 0, 1);                   // 世界系重力
    math::Vec3f v = q_.conjugated().rotate(g_world);      // 猜出来的姿态下 重力此时在哪

    math::Vec3f e = a.cross(v);                           // 算error

    // error integral (also zero offset)
    e_int_.x_ += e.x_ * dt;
    e_int_.y_ += e.y_ * dt;
    e_int_.z_ += e.z_ * dt;
    // integral limit
    e_int_.x_ = math::clamp(e_int_.x_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.y_ = math::clamp(e_int_.y_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.z_ = math::clamp(e_int_.z_, -cfg_.integral_limit_, cfg_.integral_limit_);

    e_last_ = e;
    acc_age_ = 0.0f;                    // 【改】残差新鲜 → 年龄清零（原为 is_meas_ = true）
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
```

### ⚠️ 抄录时已踩到的陷阱（2026-09-12）

**把 `acc_age_` 的初始化从"类内"搬到"构造函数的初始化列表"会引入 bug。**

若重构为 `float acc_age_;` + `Mahony(const MahonyConfig& cfg) : cfg_(cfg), acc_age_(k_never_measured_) {}`，则：

1. **编译期**：`constexpr Mahony() = default;` 无法再是 `constexpr`（它不初始化 `acc_age_`）→ **3 个 error**；
2. **语义**（即使编译通过）：`Mahony m;` 走默认构造 → `acc_age_` 是**未初始化的栈垃圾** → 随机表现为"有量测 / 无量测"，且**非确定性**——这是最难查的一类 bug。

**正确做法（单一真相源）**：初始化留在**类内**，它对**所有**构造函数生效：

```cpp
    float acc_age_ = k_never_measured_;   // 类内初始化：覆盖默认构造 + 有参构造
```

（有参构造里再补一遍也不会错，只是冗余。）

> 推广到整个 core：**任何有"哨兵初值"的成员，初始化一律放类内**，别依赖某个构造函数。
> 实测：加回类内初始化后，ctest 5/5、回放金标 2.645°/−14.450° 不变、P0-4 实验复现 5.541°/198.56s。

### 验收（抄完自己跑）

- [ ] 编译零告警（`-Wall -Wextra -Werror`）
- [ ] `ctest` **5/5 全绿**（现有 4 套锚点 + math 审计 115 断言都不受影响）
- [ ] 回放数字**逐位不变**：roll 2.152° / pitch 2.645° / yaw −14.450° ← **回归金标**
- [ ] （可选）复现 P0-4 实验：t=100s 后停止 `observe`，姿态应表现为**纯陀螺自然漂移**（>10° 需 ≈199s），而不是 1.48s 就跑飞

**只改这 4 处**：`acc_timeout_`（config）/ `acc_age_`（成员）/ `has_valid_acc()`（helper）/ 三处赋值 + `predict` 的 `+= dt`。其余一字不动。

**下一步（批次 4a 主体）**：yaw 注入通道 —— 按 `DESIGN §3.7`（`v_` 提升为成员、`e_heading_`、`heading_offset_`、`heading_age_`、`trust` 形参、`OutputStatus`）。

## 批次 4a：Mahony 外部 yaw 注入（2026-09-12 ✅ 已在 /tmp 编译 + 实测验证）

**目标**：让 6 轴下不可观的 yaw 被外部航向源锁住。设计依据 `DESIGN §3.7`，决策 Y1~Y7 已定案。

**为什么这一批最重要**：实测 6 轴基线 **yaw 漂移 −14.450°/320s**；注入外部参考后 → **RMSE 0.085°**（设计目标 < 2°）。

### 抄前必懂（四句话）

1. **修正量往哪加**：`ω += Kp_heading·e_heading·v̂`，其中 `v̂ = q̂*·(0,0,1)` 是**重力方向在机体系里的表达**。
   在机体系里沿 `v̂` 加角速度 ≡ 在**世界系里绕 z 轴转** → 纯 yaw 修正。
   ❌ **不要**加机体系 z（`(0,0,1)`）：机体一旦有 roll/pitch（斜坡/机动），绕机体系 z 转 ≠ 绕重力轴转 → yaw 误差泄漏成 roll/pitch 误差。

2. **为什么 acc 通道管不了 yaw（数学证明，`DESIGN §3.7.2`）**：
   `e_acc = a × v̂` 必垂直于两个乘数 ⇒ `e_acc ⊥ v̂` ⇒ **沿重力轴的分量恒为 0**。
   → acc 通道对 yaw **结构性失明**。yaw 通道不是"补第二次修正"，是**填补一个数学上无人负责的自由度**。

3. **残差每周期重算，不要冻结**：`e_heading = wrap(ψ_ref + offset − ψ̂)`。
   若只在收到参考时算一次再复用，等效增益 = `Kp_heading · T_ref` 会随**参考速率**变化（5Hz 参考时 `Kp·T=1` → 临界振荡）。

4. **首次观测只做"自动对齐"，不产生修正**（本批最大的行为陷阱，见下方 ⚠️）。

### 文件 12：core/math/scalar_ops.hpp（**只加一个函数，其余不动**）

> ⚠️ 这是 `core/math/` 归档后的**唯一一次解冻**（`DESIGN §4.3.1` 规定：改动须同步更新 `test_math_audit`）。
> 已在 `test_math_audit.cpp` 补 **B9a~B9j 共 10 条断言**，ctest 全绿。

```cpp
// 角度归一化到 [−π, π]：角度的【差分】必须 wrap，否则 179° 与 −179° 会算出 358° 造成猛转
// 快路径：atan2 类角度的差分绝大多数已在范围内 → 零开销直接返回
template <typename T>
inline T wrap_pi(T a) {
    const T pi = T(3.14159265358979323846);
    if (a > pi || a < -pi) {
        const T two_pi = T(2) * pi;
        a = std::fmod(a + pi, two_pi);
        if (a < T(0)) { a += two_pi; }
        a -= pi;
    }
    return a;
}
```

### 文件 13：core/solver/mahony.hpp（整文件，重抄）

> ⚠️ **本批内容已被「批次 4a-2」取代**（残差改为 predict 现算、`e_last_`→`acc_`、`observe` 去掉 `dt`）。
> 抄写请用 **批次 4a-2 的文件 17/18**；此处仅作历史记录。

```cpp
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

    void reset();           // 姿态复位为单位元
    void reset(const math::Quatf& q);   // 指定初始姿态(初始对齐F5？)

    // 异步多速率
    // predict 同imu同速率，observe事件驱动
    void predict(const math::Vec3f& gyro, float dt);
    void observe(const math::Vec3f& acc, float dt);
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
    math::Vec3f e_last_;                        // 最近一次残差
    float acc_age_ = k_never_measured_;         // 距离上一次有效acc量测时间间隔

    float heading_ref_ =0.0f;                       // 最近的一次航向参考
    float heading_offset_ = 0.0f;                   // 首次观测的自动对齐量
    float heading_age_ = k_never_measured_;         // 距上次航向参考的秒数
    float heading_trust_ = 1.0;                     // 最近一次参考的可信度
    bool is_heading_aligned_ = false;               // 是否已完成首次对齐
};




}
```

### 文件 14：core/solver/mahony.cpp（整文件，重抄）

> ⚠️ **本批内容已被「批次 4a-2」取代** —— 见文件 13 的说明。

```cpp
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
    e_last_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;
    reset_heading_channel();
}

void Mahony::reset(const math::Quatf& q) {
    q_ = q;
    q_.normalize();
    e_int_ = math::Vec3f(0, 0, 0);
    e_last_ = math::Vec3f(0, 0, 0);
    acc_age_ = k_never_measured_;
    reset_heading_channel();
}

void Mahony::predict(const math::Vec3f& gyro, float dt) {
    math::Vec3f omega = gyro;
    if (is_acc_valid()) {
        // 修正折进角速度：ω_corrected = ω + Kp·e + Ki·∫e
        omega.x_ += cfg_.kp_ * e_last_.x_ + cfg_.ki_ * e_int_.x_;
        omega.y_ += cfg_.kp_ * e_last_.y_ + cfg_.ki_ * e_int_.y_;
        omega.z_ += cfg_.kp_ * e_last_.z_ + cfg_.ki_ * e_int_.z_;
    }
    if (is_heading_valid()) {
        // yaw residual
        float roll, pitch, psi;
        q_.to_euler(roll, pitch, psi);
        float e_heading = math::wrap_pi(heading_ref_ + heading_offset_ - psi);
        e_heading = math::clamp(e_heading, -k_half_pi, k_half_pi);
        // 绕世界z轴转 == body系中沿重力方向v加角速度
        // v由当前q_现算
        const math::Vec3f v = q_.conjugated().rotate(k_gravity_world);
        const float k = cfg_.kp_heading_ * heading_trust_ * e_heading;

        omega.x_ += k * v.x_;
        omega.y_ += k * v.y_;
        omega.z_ += k * v.z_;
    }
    acc_age_ += dt;
    heading_age_ += dt;
    q_.integrate(omega, dt);
}

void Mahony::observe(const math::Vec3f& acc, float dt) {
    math::Vec3f a = acc;
    a.normalize();                                                  // only fetch direction

    math::Vec3f v = q_.conjugated().rotate(k_gravity_world);      // 猜出来的姿态q_下 重力此时在哪

    math::Vec3f e = a.cross(v);                                     // 算error

    // error integral (also zero offset)
    e_int_.x_ += e.x_ * dt;
    e_int_.y_ += e.y_ * dt;
    e_int_.z_ += e.z_ * dt;
    // integral limit
    e_int_.x_ = math::clamp(e_int_.x_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.y_ = math::clamp(e_int_.y_, -cfg_.integral_limit_, cfg_.integral_limit_);
    e_int_.z_ = math::clamp(e_int_.z_, -cfg_.integral_limit_, cfg_.integral_limit_);

    e_last_ = e;
    acc_age_ = 0.0f;                                       // acc_age_唯一清零口
}

void Mahony::update(const math::Vec3f& gyro, const math::Vec3f& acc, float dt) {
    observe(acc, dt);
    predict(gyro, dt);
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
    heading_age_ = k_never_measured_;
    heading_trust_ = 1.0f;
    is_heading_aligned_ = false;
}



}
```

### 文件 15：core/estimator.hpp（**改 1 处**）

```cpp
// 改前：空实现，调用被【静默忽略】
    void observe_heading(float /*heading_rad*/, float /*dt*/) {}

// 改后：转发给求解器（真实通道）
    // 【4a】外部航向注入：转发给求解器（原为空实现，调用被静默忽略 —— 现为真实通道）
    //   注意：第二参数语义由 dt 改为 trust（DESIGN §3.7.8 Y5）
    void observe_heading(float heading_rad, float trust = 1.0f) { solver_.observe_heading(heading_rad, trust); }
```

### ⚠️ 两个必须知道的行为陷阱（本项目实测踩到 3 次）

**① 首次观测只做自动对齐，不产生任何修正**

```cpp
est.reset(Quatf::from_euler(0, 0, 0.2f));   // 估计器当前 yaw = 0.2
est.observe_heading(1.0f);                      // 首次 → 记下 offset = 0.2 − 1.0 = −0.8
                                            // 目标 = 1.0 + (−0.8) = 0.2 = 当前值 → 残差 0，不动
```

**这是设计行为**（`DESIGN §3.7.3`）：参考的"起源"是任意的（里程计积分起点由开机时刻决定），
所以只跟踪**变化量**，保留估计器已有的初始航向。

- ✅ 想让 yaw 收敛到某个值：**从当前 yaw 出发斜坡过去**，或先 `reset(目标姿态)` 再注入
- ❌ 不要写"`reset()` 后直接 `observe_heading(目标)` 期望它转过去" —— 不会转

**② 第二参数语义变了：《dt》→《trust》**（类型相同，**编译器不会报错**）

```cpp
est.observe_heading(1.0f, 0.02f);   // 旧代码：dt=0.02s
est.observe_heading(1.0f, 0.02f);   // 新代码：trust=0.02 ← 参考几乎不被采纳，静默行为改变！
```

本项目已同步修正 `test_estimator.cpp` 中的调用。**你若有别处调用，务必一起改。**

### 验收（已实测，结果如下）

```bash
./build/replay_nav2 data/NAV2_data.bin -y gt          # 理想里程计 50Hz
./build/replay_nav2 data/NAV2_data.bin -y gt_slow     # 10Hz 多速率
./build/replay_nav2 data/NAV2_data.bin -y gt_noisy    # σ≈2° 噪声
./build/replay_nav2 data/NAV2_data.bin -y gt_drop     # t∈[100,130)s 断线
```

| 验收项（`DESIGN §3.7.7`）| 通过线 | **实测** |
|---|---|---|
| ① yaw 漂移 | −14.450° → RMSE < 2° | **0.085°**（MAX 0.701°）|
| ② roll/pitch 不退化 | RMSE < 5° | **2.152° / 2.645°** —— 与 6 轴基线**逐位相同**（正交性的最强证据）|
| ③ 10Hz 间歇参考 | 仍收敛 | **RMSE 0.095°** |
| ④ 加噪参考 σ≈2° | 不发散不猛转 | **RMSE 1.266°**（MAX 2.051°）|
| ⑤ （追加）30s 断线 | 优雅降级 | **RMSE 0.385°**，恢复后无残留 |

**单元测试**：`test_mahony` 新增 9 锚点（无参考漂移 / 锁定 / 跟随 / 自动对齐 / 失效降级 / 正交性 / 斜坡滞后 / 保持收敛 / 残差限幅），
`test_math_audit` +10，`test_estimator` 改 3 项 —— **全绿**；sanitizer（ASan+UBSan）无告警。

**理论吻合点**（不是"调出来的"，是可预测的）：
- 恒定参考的稳态误差 = 零偏/`Kp_heading` = 0.05/5 = **0.01 rad** → 实测 0.0100 ✓
- 斜坡跟随滞后 = 斜率×τ = 0.2×(1/5) = **0.04 rad** → 实测 0.0400 ✓

### 本批不做的（避免一次太多）

- ❌ acc 通道的 `trust`（只在航向通道落地）→ 与仲裁判据一起进 4b
- ❌ `OutputStatus` 完整结构（本批只给 `is_acc_valid()` / `is_heading_valid()` 两个查询口）
- ❌ 绝对航向源（磁力计/RTK）的语义 —— 自动对齐按"相对参考"设计，绝对源需要单独定语义（记入待办）

---

## 批次 4a-2：统一消费口 —— 残差每周期现算（2026-09-12 ✅ 已在 /tmp 编译 + 实测验证）

**目标**：消除 `observe` 与 `observe_heading` 的结构不对称，并修掉 acc 通道一个**潜伏的增益-速率耦合缺陷**。
改动：`core/solver/mahony.hpp` / `mahony.cpp`（整文件重抄）+ `core/estimator.hpp`（1 行）。

### 抄前必懂：这一批解决什么（四句话）

1. **问题：`v̂` 被算了两遍**
   ```cpp
   observe():            v = q̂*·(0,0,1)   ← 算 acc 残差用
   predict() → heading:  v = q̂*·(0,0,1)   ← 当旋转轴用（同一个公式、同一个 q̂、同一个周期）
   ```
   `v̂` 是**纯状态投影**（只由 `q̂` 决定，不需要任何量测）→ 应该只在**唯一消费点**算一次。

2. **更深的问题：`e_last_` 被"冻结"了一个量测周期**

   旧结构里 `observe` 算好残差存进 `e_last_`，`predict` 每个周期直接用它。若 acc 比循环慢（如 acc 100Hz、循环 400Hz），`e_last_` 会被**复用 4 次** —— 等效增益变成 `Kp × 复用次数 × dt`，**随量测速率漂移**。

   实测（从倾斜误差收敛到 1/e 的时间常数 τ，理论值 = 1/Kp = 0.2000 s）：

   | acc 速率（predict 固定 500Hz）| 旧结构 τ | 新结构 τ |
   |---|---|---|
   | 500 Hz | 0.1980 s | 0.1980 s |
   | 100 Hz | 0.1940 s | **0.1980 s** |
   | 20 Hz | 0.1740 s（快 12% ✗）| **0.1980 s** |

   → **旧结构在 acc 慢于循环时，增益会悄悄变化。** 这与 `§3.7.3` 为航向通道写下的原则是同一条：
   **残差必须在 `predict` 里用【当前】`q̂` 现算，不能冻结。**

3. **新结构：`observe*` 只收集，`predict` 是唯一消费点**

   ```cpp
   void observe(const Vec3f& acc);                        // 只存【量测】（已归一化），不带 dt
   void observe_heading(float heading_ref, float trust);  // 只存【参考】，不带 dt

   void predict(const Vec3f& gyro, float dt) {
       const Vec3f v = q̂*·(0,0,1);          // ★ 统一消费口：本周期只算一次
       if (is_acc_valid())     { e = acc_ × v;  e_int_ += e·dt;  ω += Kp·e + Ki·e_int_; }
       if (is_heading_valid()) { e = wrap(ref+offset−ψ̂);          ω += Kp·e·v; }
       ...
   }
   ```

   **两个入口现在形状完全一致**：都是"把量测装进盒子并清零年龄"，物理全在 `predict`。

4. **代价：`observe` 的签名变了**（破坏性）
   ```cpp
   est.observe(s, kDt);   // 旧：带 dt（用于 e_int_ += e·dt）
   est.observe(s);        // 新：不带（积分累加搬到 predict）
   ```

### 文件 17：core/solver/mahony.hpp（整文件，重抄）

> ⚠️ **已被「批次 4a-3」取代**（predict 拆成 `acc_correction` / `heading_correction`）。抄写请用 **文件 20/21**。

```cpp#pragma once

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
```

### 文件 18：core/solver/mahony.cpp（整文件，重抄）

> ⚠️ **已被「批次 4a-3」取代** —— 见文件 17 的说明。

```cpp#include "mahony.hpp"

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
```

### 文件 19：core/estimator.hpp（**改 1 行**）

```cpp
// 改前
    void observe(const measure::IMUSample& s, float dt) { solver_.observe(s.acc_, dt); }

// 改后（去掉 dt：求解器不再需要它）
    void observe(const measure::IMUSample& s) { solver_.observe(s.acc_); }
```

### 验收（已实测）

| 检查项 | 结果 |
|---|---|
| 严格编译 `-Wconversion -Wshadow -Werror -pedantic -fno-exceptions -fno-rtti` | 零告警 |
| ASan + UBSan | 0 告警 |
| `ctest` | 5/5 |
| `test_mahony` 15 锚点 | ALL PASS |
| 五项回放验收 | **与批次 4a 逐位一致**：gt 0.085° / gt_slow 0.095° / gt_noisy 1.266° / gt_drop 0.385° |
| **速率无关性** | τ 在 500/100/20Hz 下**恒为 0.1980 s**（旧结构 0.1980/0.1940/0.1740）|

> 1:1 同速率时两版**逐位相同** —— 所以这不是"改了行为"，而是"消掉了一个只在多速率下暴露的缺陷"。

---

## 批次 4a-3：统一出口 —— 每通道产出一个 ω 修正项（2026-09-12 ✅ 已在 /tmp 编译 + 实测验证）

**目标**：把 `predict` 从"两个 if 块"改成"两项相加"。只动 `core/solver/mahony.hpp` / `mahony.cpp`（整文件重抄）。
**行为零变化**（纯结构重构）—— 五项回放验收与批次 4a-2 **逐位相同**。

### 抄前必懂：为什么"不在线 → 返回 0"是对的

修正是**加法项**：

```cpp
ω = gyro + 项₁ + 项₂ + …
```

加法的**幺元就是 0** —— 所以：

> **门控的语义 = "这一项的值为 0"，而不是"跳过一段代码"。**

```cpp
if (is_acc_valid()) { …修正… }        // 描述【控制流】
return math::Vec3f(0,0,0);            // 描述【数学】（不在线 = 该项不存在）
```

后者更贴近本质：**"通道不在线"从一个分支降级成一次取值**。实测：不给任何量测、纯 `predict` 2s，姿态原地不动（两项确实为 0）。

### 结构收益

```cpp
// 重构后：predict 只剩骨架（函数体 42 行 → 16 行）
void Mahony::predict(const math::Vec3f& gyro, float dt) {
    const math::Vec3f v = q̂*·(0,0,1);                    // 【统一消费口】
    const math::Vec3f omega = gyro
                            + correction_acc(v, dt)      // 【统一出口】
                            + correction_heading(v);
    acc_age_ += dt;
    heading_age_ += dt;
    q_.integrate(omega, dt);
}
```

| 收益 | 说明 |
|---|---|
| **加第三个通道 = 加一行** | `+ correction_mag(v)`（`DESIGN §3.8.4` 的 ①向量观测通道与 acc 同构）|
| **每通道可独立推理/测试** | 将来能直接断言"门控失败时该项为 0" |
| **骨架一屏可见** | 算轴 → 求和 → 计时 → 积分 |
| **零开销** | `Vec3f` 12 字节且同 TU 内联 → 编译后无差别（数字逐位相同已证）|

### 两处刻意的"不对称"（不是缺陷）

1. **函数名用 `correction_*` 而不是 `*_calc`**
   `calc` 暗示纯函数，但 `correction_acc` **有副作用**：它更新 `e_int_`（积分累加）。
   （命名定案：族名前置 → `correction_acc` / `correction_heading` / 未来 `correction_mag` 成组出现）
   → 声明处已注明 `// 副作用：更新 e_int_`。不标的话，下一个读代码的人会以为能随便调用。

2. **参数故意不同：`acc_correction(v, dt)` vs `heading_correction(v)`**
   只有 acc 通道需要 `dt`（积分累加）；heading 是**纯 P**，不需要。
   → **不为"看起来对称"塞一个用不到的 `dt`**（`DESIGN §3.7.8 Y5` 已定：不留死参数）。

### 至此"对称三部曲"完成

```
① 入口统一（4a-2）  observe(acc) / observe_heading(ref, trust) —— 形状一致：只收集
        ↓
② 消费统一（4a-2）  predict 里 v̂ 只算一次，两通道共用同一个轴
        ↓
③ 出口统一（4a-3）  每通道产出一个 ω 修正项；不在线 → 0
```

### 文件 20：core/solver/mahony.hpp（整文件，重抄）

> ⚠️ **已被「批次 4a-4」取代**（`observe_heading` 收缩 + 新增 `align_heading`）。抄写请用 **文件 22/23**。

```cpp#pragma once

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

    math::Vec3f correction_acc(const math::Vec3f& v, float dt);     // ⚠ 副作用：更新e_int_（积分累加）
    math::Vec3f correction_heading(const math::Vec3f& v);           //   无副作用

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
```

### 文件 21：core/solver/mahony.cpp（整文件，重抄）

> ⚠️ **已被「批次 4a-4」取代** —— 见文件 20 的说明。

```cpp#include "mahony.hpp"

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
```

### 验收（已实测）

| 检查项 | 结果 |
|---|---|
| 严格编译 `-Wconversion -Wshadow -Werror -pedantic -fno-exceptions -fno-rtti` | 零告警 |
| `ctest` | 5/5 |
| `test_mahony` **17 锚点**（新增第 14 条"零契约"）| ALL PASS |
| 五项回放验收 | **与 4a-2 逐位相同**（gt 0.085° / gt_slow 0.095° / gt_noisy 1.266° / gt_drop 0.385°）|

**本批不改行为，只改结构** —— 所以验收标准就是"数字一模一样"。

---

## 批次 4a-4：抽出 `align_heading`（把"对齐"从收集口分离）（2026-09-13 ✅ 已在 /tmp 编译 + 实测验证）

**目标**：`observe_heading` 里混着**三条不同性质的职责**，把其中的"对齐"抽成独立公开接口。
只动 `core/solver/mahony.hpp` / `mahony.cpp`（整文件重抄）。**行为零变化**。

### 抄前必懂：`observe_heading` 原本在干三件事

```cpp
void Mahony::observe_heading(float heading_ref, float trust) {
    if (!is_heading_aligned_) {                       // ① 【对齐】只做一次
        float roll, pitch, psi;
        q_.to_euler(roll, pitch, psi);                //    读 q̂、写 offset —— 这是【初始化】，不是"收到量测"
        heading_offset_ = math::wrap_pi(psi - heading_ref);
        is_heading_aligned_ = true;
    }
    heading_ref_ = heading_ref;                       // ② 【收集】每次：参考值
    heading_trust_ = math::clamp(trust, 0.0f, 1.0f);  // ② 【收集】每次：可信度
    heading_age_ = 0.0f;                              // ③ 【清年龄】每次
}
```

**修 ③ 破坏了「入口统一」原则**（批次 4a-3 刚建立）：对照 acc 通道 ——

```cpp
void Mahony::observe(const math::Vec3f& acc) {        // 只做"收集"一件事
    acc_ = acc; acc_.normalize(); acc_age_ = 0.0f;
}
```

> **`observe*` 的契约 = 只收集量测。**
> "对齐"是**初始化**动作（和 `reset()` 同类），不该藏在收集口里。

### 抽取后的结构

```cpp
// 收集口：纯收集 + 首次兜底（兜底只为"忘了手动对齐时也能工作"，不引入静默失效）
void Mahony::observe_heading(float heading_ref, float trust) {
    if (!is_heading_aligned_) { align_heading(heading_ref); }   // 首次自动兜底
    heading_ref_ = heading_ref;                        // 收集：参考值
    heading_trust_ = math::clamp(trust, 0.0f, 1.0f);   // 收集：可信度
    heading_age_ = 0.0f;                               // 收集：清零年龄
}

// 【新公开接口】航向对齐：只动"航向零点"，不动 roll/pitch
void Mahony::align_heading(float heading_ref) {
    float roll, pitch, psi;
    q_.to_euler(roll, pitch, psi);
    heading_offset_ = math::wrap_pi(psi - heading_ref);
    heading_ref_ = heading_ref;     // 让本周期目标立刻自洽（否则会拿旧 ref 算残差 → 跳一下）
    is_heading_aligned_ = true;
}
```

**顺序（2026-09-13 用户定案）**：先更新 `heading_ref_`，再用它算 `offset` —— 数学上与原顺序等价（实测逐位相同），但不变式 `target = ref + offset = ψ̂` **一眼可验**。**两行都必须注释说明目的**，尤其 ① 行，因为它存在的理由不在这两行之内。

**关键细节（`heading_ref_ = heading_ref` 那一行）**：
对齐时若**只**改 offset、不更新 `heading_ref_`，本周期 `predict` 会拿**旧 ref** 算残差 → 目标 `old_ref + new_offset ≠ ψ̂` → **姿态跳一下**。
写上这一行后：`target = heading_ref + offset = heading_ref + (ψ̂ − heading_ref) = ψ̂` → **对齐瞬间姿态纹丝不动**。实测 `Δ < 1e-6`。

### `align_heading` 解锁的能力（原来做不到）

| 场景 | 之前 | 现在 |
|---|---|---|
| 车被搬走，重新标定航向 | 只能 `reset(q)` —— **连 roll/pitch 一起清零** | `align_heading(当前里程计读数)` —— **只动航向零点** |
| 里程计重启（读数归零/换原点）| 无解（offset 还是旧起点）| 重新对齐一次 |
| 上电时已知朝向 | 只能"猜"初值 | 等参考稳定后再对齐 |

### 一个已知隐患（本批**故意不动**）

对齐用的是 `q_.to_euler().z_` 取 yaw，而 **roll/pitch 的精度会耦合进 yaw 的提取**。若"首次自动兜底"发生在加速度计还没收敛时（车上很可能：里程计比 IMU 收敛快），offset 里就烧进了当时的 roll/pitch 误差，且**只算一次、不再更新**。

→ **对策是"让应用层决定何时对齐"**（本批已提供 `align_heading`）；**"什么时候算收敛"是策略问题，等上车看真实数据再定**，现在不猜。

### 文件 22：core/solver/mahony.hpp（整文件，重抄）

> 📝 **注释语言分工**（2026-09-13 定案）：**公开接口用英文 Doxygen**（面向外部使用者），**私有实现保持中文**（团队内部）。这是企业常见分工，不必强求全文件统一。

```cpp#pragma once

#include "../math/quat.hpp"
#include "../math/vec3.hpp"

namespace foucault::solver {


/// Tuning parameters for Mahony.
struct MahonyConfig {
    float kp_ = 5.0f;               ///< Accelerometer proportional gain.
    float ki_ = 0.3f;               ///< Accelerometer integral gain (gyro bias estimate).
    float integral_limit_ = 10.0f;  ///< Integral term clamp [rad/s].
    float acc_timeout_ = 0.1f;      ///< Accelerometer residual validity [s].
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
    /// @param acc Raw reading, any unit/scale
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

    float heading_ref_ =0.0f;                       // 最近的一次航向参考
    float heading_offset_ = 0.0f;                   // 首次观测的自动对齐量
    float heading_age_ = k_never_measured_;         // 距上次航向参考的秒数
    float heading_trust_ = 1.0;                     // 最近一次参考的可信度
    bool is_heading_aligned_ = false;               // 是否已完成首次对齐
};




}
```

### 文件 23：core/solver/mahony.cpp（整文件，重抄）

```cpp#include "mahony.hpp"

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
    q_.to_euler(roll, pitch, psi);              // 初始化
    heading_ref_ = heading_ref;                              // not下一行内联，而是用于predict内部更新 (不可删)
    heading_offset_ = math::wrap_pi(psi - heading_ref_);
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

// Vec3的修正
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

// 一维标量的修正
math::Vec3f Mahony::correction_heading(const math::Vec3f& v) {
    // Gating
    if (!is_heading_valid()) { return math::Vec3f(0.0f, 0.0f, 0.0f); }
    // 航向残差 = 参考 − 估计；必须 wrap 到 [−π,π] 再限幅（每周期用当前 q̂ 现算，不得冻结）
    float roll, pitch, psi;
    q_.to_euler(roll, pitch, psi);      // 只取psi
    float e_heading = math::wrap_pi(heading_ref_ + heading_offset_ - psi);
    e_heading = math::clamp(e_heading, -k_half_pi, k_half_pi);
    // 绕世界z轴转 == body系中沿重力方向 v 加角速度（v 由上面的统一消费口给出，本周期只算一次）
    const float k = cfg_.kp_heading_ * heading_trust_ * e_heading;      // 增益
    // 该通道的角速度修正项 = 大小 k 沿重力轴 v̂（纯 P）
    return math::Vec3f(k * v.x_, k * v.y_, k * v.z_);
}

}
```

### 验收（已实测）

| 检查项 | 结果 |
|---|---|
| 严格编译 `-Wconversion -Wshadow -Werror -pedantic -fno-exceptions -fno-rtti` | 零告警 |
| ASan + UBSan | 0 告警 |
| `ctest` | 5/5 |
| `test_mahony` **18 锚点**（新增第 15 条）| ALL PASS |
| 五项回放验收 | **与 4a-3 逐位相同**（gt 0.085° / gt_slow 0.095° / gt_noisy 1.266° / gt_drop 0.385°）|

**第 15 条判别测试**（锁死本批的三条语义）：
```
① 对齐【之后】的参考空窗期不跳变（0.2s 内 < 1e-4） ← 锁住 heading_ref_ 那一行
② 空窗结束后跟踪【新参考的变化量】（0.3 → 0.8）     ← 锁住 offset 算法
③ roll/pitch 不受扰（仍在 0.2/−0.1）                ← 锁住"只动航向零点"
```

> ⚠️ **测试设计教训（AI 自查发现并修正，2026-09-13）**：本条测试**初版**断言的是"**调用 `align_heading()` 前后 `euler()` 不变**" —— 这是**同义反复**：`align_heading` 只写 `heading_offset_` / `heading_ref_`，**根本不碰 `q_`**，所以断言恒真、抓不到任何东西。
> **实测证据**：把 `heading_ref_ = heading_ref;` 整行删掉，初版测试**依然 PASS**。
>
> 真正的判别点在对齐**之后**：`offset` 已换到新原点，若 `heading_ref_` 没跟着更新，下一个 `predict` 会拿**旧 ref** 算残差。实测（`align_heading(10.0)` 后不喂新参考）：
> ```
> 正确代码：yaw 0.299997 → 0.299998   （10 个周期漂 ~1e-6）
> 删掉那行：yaw 0.299997 → 1.856112   （每周期 +8.98° → 0.2s 后 +89.16°）
> ```
> **教训**：判别测试必须断言**被改动的行为**，而不是"调用某个函数后某个状态没变" —— 后者若该函数压根不碰那个状态，就是恒真断言。

### 文件 16：测试与回放工具（AI 已写，无需抄录）

- `tests/unit/test_math.cpp`（21 项）/ `test_math_audit.cpp`（**125 项**：含 wrap_pi B9a~B9j）/ `test_mahony.cpp`（**18 锚点**：4 基础 + 批次 4a 系列 14 条，含 P0-4 / 零契约 / 手动对齐判别测试）/ `test_estimator.cpp`（**8 锚点**）
- **批次 4a 的两条判别测试**（它们专门锁死下面两个易错点，抄漏了会红）：
  · `4a-⑩ 参考恢复`：参考断线 10s 后再恢复，漂移必须被拉回 → 锁死「对齐只做一次」（`is_heading_aligned_`，**不能**用 `is_heading_valid()`）
  · `4a-⑪ trust 越界`：负 trust 必须被 clamp 到 0 → 锁死 `math::clamp(trust, 0.0f, 1.0f)`
  · `第13条 acc 失效`：一帧坏量测后断线，冻结残差**不得**被持续复用 → 锁死 P0-4 的年龄门控（实测：改回布尔闩锁会 FAIL）
  · `第14条 零契约`：两通道都不在线 → 两个修正项都为 0（统一出口的语义）
  · `第15条 手动对齐`：对齐【后空窗期】不跳变、之后跟踪新参考、roll/pitch 不受扰
- `host/replay/replay_nav2.cpp`：读 NAV2 数据集（文本 12 列：acc3+gyro3+mag3+真值 euler3，弧度，50Hz）→ 跑 `Estimator<solver::Mahony>` → 输出 roll/pitch 的 RMSE/MAX（度）+ yaw 漂移；`-y` 选项注入外部航向（`none|gt|gt_slow|gt_noisy|gt_drop`）；可选导出 CSV
- **验收标准**：`ctest` 5/5；
  · 无外部航向时 roll/pitch RMSE < 5°，yaw 漂移是 6 轴预期行为（记录即可，不算失败）；
  · 有外部航向时 **yaw RMSE < 2°** 且 roll/pitch 不退化。

---

**批次预告**：批次 4 = EKF（先补 Level 3 理论：状态空间/雅可比/可观测性/Q-R/卡方，再动手）；届时 `Estimator<solver::EKF>` 一行切换。
