// tests/unit/test_math_audit.cpp —— core/math 独立审计测试（AI 编写，2026-09-12）
//
// ═══ 独立性声明（4 条，逐条可查）═══
//  1) 不引用 tests/unit/test_math.cpp 的任何断言或结论；用例从数学定义重新设计。
//  2) 参考实现自写且方法不同：Hamilton 积用 4x4 左乘矩阵形式、旋转用 Rodrigues 公式，
//     而 core/math 用的是分量展开式 —— 两条独立代码路径交叉验证。
//  3) 金标数据由 tests/tools/gen_math_golden.py 生成（Python 纯数学），
//     非从 C++ 输出反填 —— 语言、实现、作者三重独立。
//  4) 除正确性外，量化三类"性质"：精度预算（inv_sqrt / normalize）、
//     积分收敛阶（是否真的一阶）、运动学约定（R 到底是 body→world 还是反向）。
//
// ═══ 覆盖范围 ═══
//  A scalar_ops   B vec3 代数      C quat 代数群     D quat 旋转
//  E 欧拉角约定    F integrate 动力学  G 退化/边界      H double 实例化
//
// 运行：ctest -R test_math_audit  （CMakeLists 已挂 ASan+UBSan）
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <limits>
#include <type_traits>

#include "math/scalar_ops.hpp"
#include "math/vec3.hpp"
#include "math/quat.hpp"
#include "math_golden.hpp"

using namespace foucault::math;

// ───────────────────────── 微型断言框架（带数值诊断） ─────────────────────────

static int g_pass = 0, g_fail = 0;

static void section(const char* s) { std::printf("\n── %s ──\n", s); }

static void check(bool cond, const char* fmt, ...)
{
    char name[256];
    va_list ap; va_start(ap, fmt); std::vsnprintf(name, sizeof(name), fmt, ap); va_end(ap);
    std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", name);
    if (cond) ++g_pass; else ++g_fail;
}

// 数值断言：打印实际值/期望值/偏差/容差，便于归档时复查
static void near(double got, double want, double tol, const char* fmt, ...)
{
    char name[256];
    va_list ap; va_start(ap, fmt); std::vsnprintf(name, sizeof(name), fmt, ap); va_end(ap);
    const double d = std::fabs(got - want);
    const bool ok = (d <= tol);
    std::printf("  [%s] %-52s got=%+.9g want=%+.9g |Δ|=%.2g tol=%.0g\n",
                ok ? "PASS" : "FAIL", name, got, want, d, tol);
    if (ok) ++g_pass; else ++g_fail;
}

// ───────────────────────── 自写参考实现（方法必须与 core/math 不同） ─────────────────────────

// Hamilton 积：4x4 左乘矩阵形式（core/math 用分量展开式）
static void ref_mul(const double a[4], const double b[4], double out[4])
{
    const double M[4][4] = {
        { a[0], -a[1], -a[2], -a[3] },
        { a[1],  a[0], -a[3],  a[2] },
        { a[2],  a[3],  a[0], -a[1] },
        { a[3], -a[2],  a[1],  a[0] },
    };
    for (int i = 0; i < 4; ++i) {
        double s = 0;
        for (int j = 0; j < 4; ++j) s += M[i][j] * b[j];
        out[i] = s;
    }
}

// 从 rotate() 反推 3x3 矩阵（列 j = rotate(e_j)）—— 这是 C++ 侧唯一的"约定探针"
static void rot_to_matrix(const Quatf& q, double M[3][3])
{
    const Vec3f basis[3] = { Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1) };
    for (int j = 0; j < 3; ++j) {
        const Vec3f c = q.rotate(basis[j]);
        M[0][j] = c.x_; M[1][j] = c.y_; M[2][j] = c.z_;
    }
}

static double det3(const double M[3][3])
{
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
         - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
         + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

static Vec3f v3(const float* a) { return Vec3f(a[0], a[1], a[2]); }
static Quatf qf(const float* a) { return Quatf(a[0], a[1], a[2], a[3]); }

// ───────────────────────── A. scalar_ops ─────────────────────────

static void test_scalar_ops()
{
    section("A. scalar_ops");

    // A1 inv_sqrt(float) 相对误差界：宣称"位魔法 + 两轮牛顿"
    double worst = 0; double worst_x = 0;
    for (double x = 1e-4; x < 1e4; x *= 1.37) {
        const float xf = static_cast<float>(x);
        const double got  = inv_sqrt(xf);
        const double want = 1.0 / std::sqrt(static_cast<double>(xf));
        const double rel  = std::fabs(got - want) / want;
        if (rel > worst) { worst = rel; worst_x = x; }
    }
    std::printf("  [INFO] inv_sqrt(float) 最坏相对误差 = %.3g @ x=%.4g\n", worst, worst_x);
    check(worst < 1e-5, "A1 inv_sqrt(float) 相对误差 < 1e-5（实测最坏 4.6e-6，非机器精度）");

    // A2 inv_sqrt(double) 走精确路径
    near(inv_sqrt(4.0), 0.5, 1e-15, "A2 inv_sqrt(double) 精确路径");

    // A3 退化输入：float 路径是位魔法，行为与 double 路径（1/sqrt）不同
    {
        const float z = inv_sqrt(0.0f);
        const double zd = inv_sqrt(0.0);
        std::printf("  [INFO] inv_sqrt(0)   float=%.6g   double=%.6g  真值=inf\n",
                    static_cast<double>(z), zd);
        check(std::isinf(zd), "A3a inv_sqrt(0.0) [double] → inf（数学正确）");
        check(!std::isinf(z) && z > 1e18f,
              "A3b inv_sqrt(0.0f) [float] → 巨大有限值而非 inf（位魔法副作用，非 UB）");

        const float n = inv_sqrt(-1.0f);
        const double nd = inv_sqrt(-1.0);
        std::printf("  [INFO] inv_sqrt(-1)  float=%.6g   double=%.6g  真值=nan\n",
                    static_cast<double>(n), nd);
        check(std::isnan(nd), "A3c inv_sqrt(-1.0) [double] → nan（数学正确）");
        check(std::isinf(n), "A3d inv_sqrt(-1.0f) [float] → inf（位魔法路径；无定义域检查、不报错）");
        check(!std::isinf(inv_sqrt(0.0f)) && inv_sqrt(0.0f) > 1e18f,
              "A3e inv_sqrt(0.0f) [float] → 巨大有限值而非 inf（float/double 路径行为不一致）");
    }

    // A4 clamp 边界
    check(clamp(5.0f, 0.0f, 1.0f) == 1.0f, "A4a clamp 超上限 → 上限");
    check(clamp(-5.0f, 0.0f, 1.0f) == 0.0f, "A4b clamp 超下限 → 下限");
    check(clamp(0.5f, 0.0f, 1.0f) == 0.5f, "A4c clamp 区间内不变");
    check(clamp(0.0f, 0.0f, 1.0f) == 0.0f && clamp(1.0f, 0.0f, 1.0f) == 1.0f,
          "A4d clamp 端点闭区间");
    check(clamp(2.0f, 3.0f, 1.0f) == 3.0f, "A4e clamp 反边界(lo>hi) → 返回 lo（v<lo 判定优先，不崩）");

    // A5 角度换算
    near(deg_to_rad(180.0f), M_PI, 2e-7, "A5a deg_to_rad(180) = π");
    near(rad_to_deg(static_cast<float>(M_PI)), 180.0, 1e-4, "A5b rad_to_deg(π) = 180");
    near(rad_to_deg(deg_to_rad(37.5f)), 37.5, 1e-5, "A5c deg↔rad 往返");
}

// ───────────────────────── B. vec3 ─────────────────────────

static void test_vec3()
{
    section("B. vec3 代数");

    // B1 三种构造
    {
        const Vec3f d;
        const Vec3f b(2.0f);
        const Vec3f t(1, 2, 3);
        check(d.x_ == 0 && d.y_ == 0 && d.z_ == 0, "B1a 默认构造 = 零向量");
        check(b.x_ == 2 && b.y_ == 2 && b.z_ == 2, "B1b 单参数构造 = 广播到三分量");
        check(t.x_ == 1 && t.y_ == 2 && t.z_ == 3, "B1c 三参数构造");
    }

    // B2 右手系基底叉积
    {
        const Vec3f X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
        check(X.cross(Y).z_ == 1 && X.cross(Y).x_ == 0 && X.cross(Y).y_ == 0, "B2a x̂ × ŷ = ẑ");
        check(Y.cross(Z).x_ == 1, "B2b ŷ × ẑ = x̂");
        check(Z.cross(X).y_ == 1, "B2c ẑ × x̂ = ŷ（右手系）");
    }

    // B3 代数恒等式（拉格朗日 / 反对称 / 双线性 / 混合积）
    {
        const Vec3f a(1.3f, -2.7f, 0.9f), b(-0.4f, 1.1f, 3.2f), c(2.0f, 0.5f, -1.7f);
        const double la = a.norm(), lb = b.norm();

        const Vec3f axb = a.cross(b);
        const double lhs = static_cast<double>(axb.norm_squared()) + static_cast<double>(a.dot(b)) * a.dot(b);
        near(lhs, la * la * lb * lb, 1e-3, "B3a 拉格朗日恒等式 |a×b|²+(a·b)² = |a|²|b|²");

        const Vec3f bxa = b.cross(a);
        near(axb.x_, -bxa.x_, 1e-6, "B3b 反对称 a×b = -(b×a) [x]");
        near(a.cross(a).norm(), 0.0, 1e-6, "B3c a×a = 0");

        near(a.cross(b).dot(c), b.cross(c).dot(a), 1e-4, "B3d 混合积 a·(b×c) = b·(c×a)");
        near(a.cross(b.cross(c)).dot(Vec3f(1, 1, 1)), (a.dot(c) * b - a.dot(b) * c).dot(Vec3f(1, 1, 1)),
             1e-4, "B3e 向量三重积 a×(b×c) = (a·c)b - (a·b)c");
        near(axb.dot(a), 0.0, 1e-5, "B3f (a×b)·a = 0（正交性）");
    }

    // B4 normalize
    {
        Vec3f a(3, 4, 0);
        a.normalize();
        near(a.norm(), 1.0, 1e-6, "B4a normalize 后模长 = 1（Vec3 走精确 1/sqrt）");
        near(a.x_, 0.6, 1e-6, "B4b normalize 保持方向 (3,4,0)→(0.6,0.8,0)");

        Vec3f z(0, 0, 0);
        z.normalize();
        check(z.x_ == 0 && z.y_ == 0 && z.z_ == 0, "B4c 零向量 normalize 保持零（显式防御，非 NaN）");
    }

    // B5 operator[] 语义（越界行为已写明=P1-4“既不检查也不再静默”已定为“调用方保证”）
    {
        Vec3f v(1, 2, 3);
        check(v[0] == 1 && v[1] == 2 && v[2] == 3, "B5a operator[] 0/1/2 → x/y/z");
        check(v[3] == 3, "B5b operator[] 越界 [3] 返回 z（P1-4 已文档化为“调用方保证 0..2”）");
        check(v[-1] == 3, "B5c operator[] 负索引 [-1] 同样返回 z（P1-4 同上）");
        v[1] = 9.0f;
        check(v.y_ == 9.0f, "B5d operator[] 非 const 版本可写");
    }

    // B9 wrap_pi（2026-09-12 批次 4a 新增；core/math 解冻的唯一内容，DESIGN §3.7.3）
    {
        const float pi = 3.14159265358979f;
        near(wrap_pi(0.0f), 0.0f, 1e-7, "B9a wrap_pi(0) = 0");
        near(wrap_pi(1.0f), 1.0f, 1e-7, "B9b 域内原样返回（快路径，正）");
        near(wrap_pi(-1.0f), -1.0f, 1e-7, "B9c 域内原样返回（快路径，负）");
        near(wrap_pi(2.0f * pi), 0.0f, 1e-4, "B9d wrap_pi(2π) = 0（整圈回零）");
        near(wrap_pi(3.0f * pi / 2.0f), -pi / 2.0f, 1e-4, "B9e wrap_pi(3π/2) = −π/2（跨圈）");
        near(wrap_pi(-3.0f * pi / 2.0f), pi / 2.0f, 1e-4, "B9f wrap_pi(−3π/2) = +π/2");
        near(wrap_pi(100.0f * pi + 0.5f), 0.5f, 1e-2, "B9g 多圈（100π+0.5）仍正确");
        // ★ 这就是 yaw 残差必须 wrap 的理由：179° 与 −179° 只差 2°，不 wrap 会算出 358°
        const float deg2rad = pi / 180.0f;
        near(wrap_pi(-179.0f * deg2rad - 179.0f * deg2rad), 2.0f * deg2rad, 1e-5,
             "B9h ★ wrap_pi(179° 与 −179° 之差) = 2°（不 wrap 会得 358°）");
        check(wrap_pi(pi) > 0.0f && wrap_pi(pi) <= pi, "B9i 端点 π 落在 [−π, π] 内");
        near(wrap_pi(4.0f), 4.0f - 2.0f * pi, 1e-5, "B9j 略超 π（4.0 rad）单次回卷到 −2.283");
    }

    // B6 单参数构造已加 explicit（P1-3，2026-09-12）：隐式转换必须【不可编译】
    {
        static_assert(!std::is_convertible_v<float, Vec3f>,
                      "P1-3: float 不得隐式转成 Vec3f（Vec3(T x) 应为 explicit）");
        static_assert(std::is_constructible_v<Vec3f, float>,
                      "P1-3: 显式构造 Vec3f(2.5f) 仍应可用");
        const Vec3f bcast(2.5f);
        check(bcast.x_ == 2.5f && bcast.y_ == 2.5f && bcast.z_ == 2.5f,
              "B6a 显式单值构造 Vec3f(2.5f) = (2.5,2.5,2.5)");
        check(!std::is_convertible_v<float, Vec3f>,
              "B6b float → Vec3f 隐式转换已被 explicit 禁掉");
    }

    // B8 补全运算符覆盖：一元取负 / -= / 复合赋值恒等
    {
        const Vec3f a(1.5f, -2.5f, 3.0f), b(-0.5f, 4.0f, -1.0f);
        const Vec3f na = -a;
        check(na.x_ == -1.5f && na.y_ == 2.5f && na.z_ == -3.0f, "B8a 一元取负 -a 逐分量取反");
        check((a + (-a)).norm() == 0.0f, "B8b a + (-a) = 0");
        Vec3f c = a; c -= b;
        near(c.x_, 2.0f, 1e-6, "B8c -= 逐分量相减 [x]");
        near(c.z_, 4.0f, 1e-6, "B8d -= 逐分量相减 [z]");
        Vec3f d2 = a; d2 += b;
        check(d2.x_ == 1.0f && d2.y_ == 1.5f, "B8e += 与二元 + 一致");
        Vec3f e2 = a; e2 *= 3.0f;
        near(e2.norm(), 3.0f * a.norm(), 1e-5, "B8f *= 标量与模长成比例");
        near((a - b).dot(b), (a.dot(b) - b.dot(b)), 1e-5, "B8g 分配律 (a-b)·b = a·b - b·b");
    }

    // B7 除零（无守卫，记录现状）
    {
        const Vec3f v(1, 1, 1);
        const Vec3f r = v / 0.0f;
        check(std::isinf(r.x_) && std::isinf(r.z_), "B7 除零 → inf（无守卫，记录现状）");
    }
}

// ───────────────────────── C. quat 代数 ─────────────────────────

static void test_quat_algebra()
{
    section("C. quat 代数群");

    // C1 单位元
    {
        const Quatf d;
        check(d.q0_ == 1 && d.q1_ == 0 && d.q2_ == 0 && d.q3_ == 0, "C1a 默认构造 = 单位元(1,0,0,0)");
        check(Quatf::identity().q0_ == 1, "C1b identity()");

        const Quatf a = Quatf::from_euler(0.3f, -0.4f, 1.2f);
        const Quatf la = a * Quatf::identity();
        const Quatf ra = Quatf::identity() * a;
        near(la.q0_, a.q0_, 1e-6, "C1c q ⊗ 1 = q");
        near(ra.q3_, a.q3_, 1e-6, "C1d 1 ⊗ q = q");
    }

    // C2 模长与共轭
    {
        const Quatf a(0.5f, -0.5f, 0.5f, -0.5f);
        near(a.norm_squared(), 1.0, 1e-6, "C2a norm_squared = 1");
        near(a.norm(), 1.0, 1e-6, "C2b norm = 1");

        const Quatf b(0.3f, 0.4f, -0.5f, 0.7f);
        const Quatf bb = b * b.conjugated();
        near(bb.q0_, b.norm_squared(), 1e-6, "C2c q ⊗ q* = |q|² (标量部)");
        near(bb.q1_, 0.0, 1e-6, "C2d q ⊗ q* 向量部 = 0");
    }

    // C3 群性质：结合律 / 模长乘性 / 共轭反同态 / 不交换
    {
        const Quatf a(0.5f, -0.5f, 0.5f, -0.5f);
        const Quatf b(0.3f, 0.4f, -0.5f, 0.7f);
        const Quatf c(0.6f, -0.2f, 0.3f, 0.5f);

        const Quatf ab_c = (a * b) * c;
        const Quatf a_bc = a * (b * c);
        near(ab_c.q0_, a_bc.q0_, 1e-5, "C3a 结合律 (a⊗b)⊗c = a⊗(b⊗c) [w]");
        near(ab_c.q3_, a_bc.q3_, 1e-5, "C3b 结合律 [z]");

        const double nab = (a * b).norm();
        near(nab, a.norm() * b.norm(), 1e-6, "C3c 模长乘性 |a⊗b| = |a|·|b|");

        const Quatf ab_cj = (a * b).conjugated();
        const Quatf bcj = b.conjugated() * a.conjugated();
        near(ab_cj.q1_, bcj.q1_, 1e-6, "C3d 共轭反同态 (a⊗b)* = b*⊗a* [x]");

        const Quatf ba = b * a;
        check(std::fabs(ab_c.q1_ - ba.q1_) > 1e-3, "C3e 不满足交换律 a⊗b ≠ b⊗a（旋转本就不可交换）");
    }

    // C4 对金标数据（独立 4x4 矩阵实现）
    {
        for (const auto& m : golden::kMul) {
            const Quatf a = qf(m.a_), b = qf(m.b_);
            const Quatf got = a * b;
            const float got4[4] = { got.q0_, got.q1_, got.q2_, got.q3_ };
            double af[4], bf[4], ref[4], refb[4];
            for (int i = 0; i < 4; ++i) { af[i] = m.a_[i]; bf[i] = m.b_[i]; }
            ref_mul(af, bf, ref);
            ref_mul(bf, af, refb);
            double e = 0, eb = 0;
            for (int i = 0; i < 4; ++i) {
                e  += std::fabs(got4[i] - ref[i]);
                eb += std::fabs(got4[i] - refb[i]);
            }
            near(e, 0.0, 1e-6, "C4  Hamilton 积对独立 4x4 矩阵实现 [%s] (与 ba 差 %.3g)", m.name_, eb);
        }
    }
}

// ───────────────────────── D. quat 旋转 ─────────────────────────

static void test_quat_rotate()
{
    section("D. quat 旋转");

    // D1 对 Rodrigues 金标数据（6 组轴角 × 5 个向量 = 30 个比对）
    {
        double worst = 0; const char* wname = "";
        for (const auto& c : golden::kRot) {
            const Quatf q = qf(c.q_);
            for (int i = 0; i < 5; ++i) {
                const Vec3f got = q.rotate(v3(c.in_[i]));
                const double e = std::fabs(got.x_ - c.out_[i][0]) + std::fabs(got.y_ - c.out_[i][1])
                               + std::fabs(got.z_ - c.out_[i][2]);
                if (e > worst) { worst = e; wname = c.name_; }
            }
        }
        near(worst, 0.0, 1e-5, "D1  30 组 rotate 对 Rodrigues 独立实现（最坏 %s）", wname);
    }

    // D2 保模长 / 单位元 / 转轴不变
    {
        const Quatf q = Quatf::from_euler(0.4f, -0.7f, 2.1f);
        const Vec3f v(1.0f, -2.0f, 0.5f);
        near(q.rotate(v).norm(), v.norm(), 1e-5, "D2a rotate 保模长（单位四元数）");
        near(Quatf::identity().rotate(v).x_, v.x_, 1e-7, "D2b 单位元旋转 = 恒等");

        // 转轴不变式：绕 z 转任意角，ẑ 不动
        const Quatf qz = qf(golden::kRot[0].q_);            // z90
        const Vec3f zr = qz.rotate(Vec3f(0, 0, 1));
        near(zr.z_, 1.0, 1e-6, "D2c 绕 z 旋转不改变 ẑ");
    }

    // D3 同态性 R(a⊗b) = R(a)·R(b) 与逆旋转
    {
        const Quatf a = Quatf::from_euler(0.3f, -0.5f, 1.1f);
        const Quatf b = Quatf::from_euler(-0.9f, 0.2f, 0.4f);
        const Vec3f v(0.7f, -0.3f, 1.9f);

        const Vec3f lhs = (a * b).rotate(v);
        const Vec3f rhs = a.rotate(b.rotate(v));
        near(lhs.x_ - rhs.x_ + lhs.y_ - rhs.y_ + lhs.z_ - rhs.z_, 0.0, 1e-5,
             "D3a 同态性 rotate(a⊗b, v) = rotate(a, rotate(b, v))");

        const Vec3f back = a.conjugated().rotate(a.rotate(v));
        near(back.x_, v.x_, 1e-5, "D3b 共轭 = 逆旋转 (q*∘q = 恒等) [x]");
    }

    // D4 矩阵性质：正交 + det=+1（真旋转，非镜像）
    {
        const Quatf q = Quatf::from_euler(0.7f, 1.2f, -2.5f);
        double M[3][3]; rot_to_matrix(q, M);
        double orth = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double s = 0;
                for (int k = 0; k < 3; ++k) s += M[k][i] * M[k][j];
                orth += std::fabs(s - (i == j ? 1.0 : 0.0));
            }
        near(orth, 0.0, 1e-5, "D4a RᵀR = I（正交性）");
        near(det3(M), 1.0, 1e-5, "D4b det(R) = +1（真旋转，不是镜像）");
    }

    // D5 双覆盖：q 与 -q 表示同一旋转
    {
        const Quatf q = Quatf::from_euler(0.3f, -0.5f, 1.1f);
        const Quatf nq(-q.q0_, -q.q1_, -q.q2_, -q.q3_);
        const Vec3f v(1.3f, -0.4f, 0.8f);
        const Vec3f r1 = q.rotate(v), r2 = nq.rotate(v);
        near(r1.x_ - r2.x_ + r1.y_ - r2.y_ + r1.z_ - r2.z_, 0.0, 1e-5,
             "D5  q 与 -q 给出同一旋转（双覆盖）");
    }

    // D6 非单位四元数：rotate 不再是旋转，且快速展开式 ≠ 精确 qvq*（量化 normalize 的硬需求）
    {
        const Quatf bad(1.0f, 0.0f, 0.0f, 0.30f);       // |q|² = 1.09
        const Vec3f v(1.0f, 0.0f, 0.0f);
        const double got = bad.rotate(v).norm();
        const double exact = static_cast<double>(bad.norm_squared());   // |q v q*| = |q|²|v|
        std::printf("  [INFO] 非单位四元数 q=(1,0,0,0.30)  |q|²=%.4f  |q v q*| 应为 %.6f\n",
                    static_cast<double>(bad.norm_squared()), exact);
        std::printf("         rotate() 快速展开式实际给出 %.6f（≠|q|²）\n"
                    "         ⟹ 快速公式 v+2q₀(qᵥ×v)+2qᵥ×(qᵥ×v) 仅在 |q|=1 时等于 q v q*\n", got);
        near(got, 1.0160708, 1e-5,
             "D6a |q|≠1 时 rotate 输出模长 1.01607（既非 1，也非 |q|²=1.09）");
        check(std::fabs(got - exact) > 0.05,
             "D6b ★快速展开式偏离精确 qvq* 达 0.074 ⟹ normalize 不是优化而是正确性前提");
        near(Quatf(bad).normalize().rotate(v).norm(), 1.0, 1e-5,
             "D6c 同一旋转先 normalize 再 rotate → 模长回到 1（正常）");
    }
}

// ───────────────────────── E. 欧拉角约定 ─────────────────────────

static void test_euler()
{
    section("E. 欧拉角（内旋 ZYX）约定");

    // E1 独立验证约定：from_euler → rotate → 矩阵，必须等于 Python 独立构造的 Rz(y)Ry(p)Rx(r)
    {
        double worst = 0; const char* wname = "";
        for (const auto& c : golden::kEul) {
            const Quatf q = Quatf::from_euler(c.euler_[0], c.euler_[1], c.euler_[2]);
            near(q.norm(), 1.0, 1e-6, "E1x from_euler[%s] 产出单位四元数", c.name_);
            double M[3][3]; rot_to_matrix(q, M);
            double e = 0;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) e += std::fabs(M[i][j] - c.R_[i][j]);
            if (e > worst) { worst = e; wname = c.name_; }
        }
        near(worst, 0.0, 1e-5,
             "E1  from_euler+rotate 的矩阵 == 独立构造的 ZYX 矩阵（7 组，最坏 %s）", wname);
    }

    // E2 往返（主值分支内）
    {
        double worst = 0; const char* wname = "";
        for (const auto& c : golden::kEul) {
            if (std::fabs(c.euler_[1]) > 1.5) continue;      // 跳过万向锁邻域（E3 单独测）
            const Quatf q = Quatf::from_euler(c.euler_[0], c.euler_[1], c.euler_[2]);
            float r, p, y; q.to_euler(r, p, y);
            const double e = std::fabs(r - c.euler_[0]) + std::fabs(p - c.euler_[1]) + std::fabs(y - c.euler_[2]);
            if (e > worst) { worst = e; wname = c.name_; }
        }
        near(worst, 0.0, 1e-5, "E2  to_euler(from_euler(e)) = e（最坏 %s）", wname);
    }

    // E3 万向锁 pitch=±90°：欧拉三元组退化，但旋转矩阵仍然正确
    {
        for (const auto& c : golden::kEul) {
            if (std::fabs(c.euler_[1]) < 1.5) continue;
            const Quatf q = Quatf::from_euler(c.euler_[0], c.euler_[1], c.euler_[2]);
            double M[3][3]; rot_to_matrix(q, M);
            double e = 0;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) e += std::fabs(M[i][j] - c.R_[i][j]);
            float r, p, y; q.to_euler(r, p, y);
            std::printf("  [INFO] 万向锁[%s] 矩阵误差=%.2g；往返后 euler=(%.4f,%.4f,%.4f) vs 原 (%.4f,%.4f,%.4f)\n",
                        c.name_, e, r, p, y, c.euler_[0], c.euler_[1], c.euler_[2]);
            check(e < 1e-5, "E3a 万向锁[%s] 旋转矩阵仍正确（奇点只在欧拉表示层）", c.name_);
            // asin 在 ±1 附近导数 1/√(1-x²)→∞：浮点里 2·q0·q2=0.99999994 会被放大 ~2900 倍
            // 故万向锁邻域的 pitch 只能到 ~2e-4 rad 量级（并非实现缺陷，是 asin 公式固有）
            near(std::fabs(p), static_cast<double>(M_PI) / 2, 5e-4,
                 "E3b 万向锁[%s] pitch 仍解出 ±90°（asin 奇点放大 ~2e-4 rad）", c.name_);
        }
    }

    // E4 输出范围
    {
        const Quatf q = Quatf::from_euler(3.0f, 0.2f, 0.6f);
        float r, p, y; q.to_euler(r, p, y);
        check(std::fabs(r) <= static_cast<float>(M_PI) + 1e-5f, "E4a roll ∈ [-π,π]");
        check(std::fabs(p) <= static_cast<float>(M_PI) / 2 + 1e-5f, "E4b pitch ∈ [-π/2,π/2]（asin 值域）");
        check(std::fabs(y) <= static_cast<float>(M_PI) + 1e-5f, "E4c yaw ∈ [-π,π]");
    }
}

// ───────────────────────── F. integrate 动力学 ─────────────────────────

static void test_integrate()
{
    section("F. integrate 动力学与约定");

    // F1 静止：ω=0 不应改变姿态
    {
        Quatf q = Quatf::from_euler(0.4f, -0.2f, 1.1f);
        const Quatf before = q;
        q.integrate(Vec3f(0, 0, 0), 0.02f);
        near(q.q0_ - before.q0_ + q.q1_ - before.q1_ + q.q2_ - before.q2_ + q.q3_ - before.q3_,
             0.0, 2e-5, "F1  ω=0 → 姿态不变（integrate 仍走 normalize，±4e-6 位魔法缩放）");
    }

    // F2 金标：常角速度积分 T 秒 == 绕 ω 轴转 |ω|T（轴角参考四元数）
    {
        const double dt = 0.001;
        for (const auto& c : golden::kInteg) {
            Quatf q = Quatf::identity();
            const int n = static_cast<int>(c.T_ / dt + 0.5);
            for (int i = 0; i < n; ++i) q.integrate(v3(c.w_), static_cast<float>(dt));
            const double e = std::fabs(q.q0_ - c.q_fine_[0]) + std::fabs(q.q1_ - c.q_fine_[1])
                           + std::fabs(q.q2_ - c.q_fine_[2]) + std::fabs(q.q3_ - c.q_fine_[3]);
            near(e, 0.0, 2e-4, "F2  细步长积分 == 轴角解析解 [%s]", c.name_);
        }
    }

    // F3 观测阶（Richardson 自比较，不依赖外部参考解）：
    //    文档称"一阶欧拉"，此处用实测阶数判定真实精度。
    //    注意"步长一致"是必须的：若用 dt/2 与 dt 两个不同总时长的解相减，得到的是误差本身而非阶数。
    {
        const double T = 2.0;
        auto qdist = [](const Quatf& a, const Quatf& b) {
            const float A[4] = { a.q0_, a.q1_, a.q2_, a.q3_ };
            const float B[4] = { b.q0_, b.q1_, b.q2_, b.q3_ };
            double d1 = 0, d2 = 0;
            for (int i = 0; i < 4; ++i) { d1 += (A[i]-B[i])*(A[i]-B[i]); d2 += (A[i]+B[i])*(A[i]+B[i]); }
            return std::sqrt(d1 < d2 ? d1 : d2);        // 处理双覆盖 q ≡ -q
        };
        auto run = [&](int mode, float dt) {
            Quatf q = Quatf::identity();
            const int n = static_cast<int>(T / dt + 0.5);
            for (int i = 0; i < n; ++i) {
                const float t = (i + 0.5f) * dt;
                const Vec3f w = (mode == 0) ? Vec3f(0.6f, -0.8f, 0.0f)
                                            : Vec3f(0.9f * std::sin(2.3f * t),
                                                    0.7f * std::cos(1.7f * t), -0.5f);
                q.integrate(w, dt);
            }
            return q;
        };
        const char* tag[2] = { "常角速度", "时变 ω(t)" };
        for (int m = 0; m < 2; ++m) {
            float dt = 0.08f; double prev = 0, acc = 0; int cnt = 0;
            for (int k = 0; k < 4; ++k, dt /= 2.0f) {
                const double e = qdist(run(m, dt), run(m, dt * 0.5f));
                if (prev > 0) { acc += std::log2(prev / e); ++cnt; }
                prev = e;
            }
            const double p = acc / cnt;
            std::printf("  [INFO] %s 观测阶 p = %.3f（1=一阶  2=二阶  3=三阶）\n", tag[m], p);
            near(p, 2.0, 0.4,
                 "F3%c %s：实测全局二阶（normalize 恰好消掉二阶项：纯四元数 x 有 x²=-|x|²）",
                 m ? 'b' : 'a', tag[m]);
        }
    }

    // F4 模长保持（每步 normalize；含位魔法精度）
    {
        Quatf q = Quatf::identity();
        double worst = 0;
        for (int i = 0; i < 20000; ++i) {
            q.integrate(Vec3f(0.3f, -0.5f, 0.8f), 0.001f);
            worst = std::max(worst, std::fabs(static_cast<double>(q.norm()) - 1.0));
        }
        std::printf("  [INFO] 2 万步积分后模长最大偏离 = %.3g（inv_sqrt 位魔法精度约 4e-6）\n", worst);
        near(worst, 0.0, 1e-5, "F4  normalize 后模长始终 ≈ 1（未漂移）");
    }

    // F5 ★运动学约定锁定★：R 满足 Ṙ = R·[ω]×  ⇒ R 是 body→world
    //    这是全库最容易搞反的一处；本测试用独立数值微分判定方向。
    {
        const Quatf q0 = Quatf::from_euler(0.3f, -0.5f, 1.1f);
        const Vec3f w(0.7f, -0.4f, 0.9f);
        const float dt = 1e-2f;                            // 兼顾截断误差与浮点抵消
        double R0[3][3], R1[3][3];
        rot_to_matrix(q0, R0);
        Quatf q1 = q0; q1.integrate(w, dt);
        rot_to_matrix(q1, R1);

        const double W[3][3] = { { 0, -w.z_, w.y_ }, { w.z_, 0, -w.x_ }, { -w.y_, w.x_, 0 } };
        double e_body2world = 0, e_world2body = 0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                const double dR = (R1[i][j] - R0[i][j]) / dt;
                double rw = 0, wr = 0;
                for (int k = 0; k < 3; ++k) { rw += R0[i][k] * W[k][j]; wr += W[i][k] * R0[k][j]; }
                e_body2world += (dR - rw) * (dR - rw);
                e_world2body += (dR + wr) * (dR + wr);
            }
        const double nb = std::sqrt(e_body2world), nw = std::sqrt(e_world2body);
        std::printf("  [INFO] ‖Ṙ - R[ω]×‖ = %.4f    ‖Ṙ + [ω]×R‖ = %.4f   (|ω| = %.3f)\n",
                    nb, nw, static_cast<double>(w.norm()));
        check(nb * 20 < nw,
              "F5a R = q.rotate 是 body→world（Ṙ=R[ω]× 残差远小于反向假设）");
        check(nb < 0.02 * w.norm(),
              "F5b body→world 残差 < 2%%|ω|（若为反向假设则超 200%%）");
        // 与 observe 里 v = q.conjugated().rotate(g) 的一致性：重力在 body 系
        const Vec3f g_body = q0.conjugated().rotate(Vec3f(0, 0, 1));
        const Vec3f g_world = q0.rotate(g_body);
        near(g_world.z_, 1.0, 1e-5, "F5c q.rotate(q*.rotate(ẑ)) = ẑ（共轭=逆，observe 用此方向取重力）");
    }
}

// ───────────────────────── G. 退化与边界 ─────────────────────────

static void test_degenerate()
{
    section("G. 退化与边界输入");

    // G1 零四元数 normalize → 复位为单位元（显式防御）
    {
        Quatf z(0, 0, 0, 0);
        z.normalize();
        check(z.q0_ == 1 && z.q1_ == 0 && z.q2_ == 0 && z.q3_ == 0,
              "G1  零四元数 normalize → 单位元（quat.hpp 显式回退，不产生 NaN）");
    }

    // G2 ★NaN/Inf 经 normalize 的行为★ —— 本轮审计最重的发现（P0-5，已修）
    {
        const float nan = std::nanf("");
        Quatf q(nan, 0, 0, 0);
        q.normalize();
        std::printf("  [INFO] normalize((nan,0,0,0)) = (%.1f,%.1f,%.1f,%.1f)\n", q.q0_, q.q1_, q.q2_, q.q3_);
        check(std::isnan(q.q0_),
              "G2a ★NaN 四元数 normalize 后 q0_ 仍为 NaN（脏值原样保留，不再被洗成单位元）");
        check(std::isnan(q.norm_squared()),
              "G2a2 norm_squared() 为 NaN ⇒ 调用方【可检出】；旧行为返回 1、完全看不出异常");

        Quatf qi(std::numeric_limits<float>::infinity(), 0, 0, 0);
        qi.normalize();
        std::printf("  [INFO] normalize((inf,0,0,0)) = (%.1f,%.1f,%.1f,%.1f)\n", qi.q0_, qi.q1_, qi.q2_, qi.q3_);
        check(std::isinf(qi.q0_) && std::isnan(qi.q2_),
              "G2b +Inf 走 if 分支 → (inf,nan,nan,nan)：非有限值，明显异常可检出");

        Quatf qn(1, 0, 0, 0);
        const Vec3f r = qn.rotate(Vec3f(nan, nan, nan));
        check(std::isnan(r.x_), "G2c rotate(NaN 向量) → NaN 传播（rotate 不吞错，只有 normalize 会）");
        std::printf("  [NOTE] 修复后下游行为：脏量测仍会污染 e_int_，但 quaternion() 会【返回 NaN】，\n"
                    "         而不再是一个看上去正常的单位元 —— “静默错”变成“响亮错”。真正的隔离仍需\n"
                    "         上游输入校验（P1-1）。修前/修后对照见待办 P0-5。\n");
    }

    // G3 精确 180° 旋转（sin/cos 边界）
    {
        const float ax[3] = {0, 0, 1};
        Quatf q = qf(golden::kRot[0].q_);
        Quatf half(static_cast<float>(std::cos(M_PI / 2)), 0, 0, static_cast<float>(std::sin(M_PI / 2))); // 180° about z
        const Vec3f r = half.rotate(Vec3f(1, 0, 0));
        near(r.x_, -1.0, 1e-6, "G3a 绕 z 180°: x̂ → -x̂");
        near(half.norm(), 1.0, 1e-6, "G3b 180° 四元数 (0,0,0,1) 是单位四元数");
        (void)q; (void)ax;
    }

    // G4 极小角度（大数抵消敏感区）
    {
        Quatf q = Quatf::identity();
        q.integrate(Vec3f(0, 0, 1e-7f), 0.001f);
        check(std::isfinite(q.q0_) && std::isfinite(q.q3_), "G4a 极小角速度不产生 NaN/Inf");
        near(q.norm(), 1.0, 1e-5, "G4b 极小角速度后仍为单位四元数（±位魔法精度）");
    }

    // G5 dt=0（非法输入，无校验 —— 记录现状）
    {
        Quatf q = Quatf::from_euler(0.2f, 0.3f, 0.4f);
        const Quatf before = q;
        q.integrate(Vec3f(1, 2, 3), 0.0f);
        near(q.q0_ - before.q0_ + q.q3_ - before.q3_, 0.0, 2e-5, "G5a dt=0 → 姿态不变（±位魔法缩放）");
        q.integrate(Vec3f(1, 2, 3), -0.01f);
        check(std::isfinite(q.q0_), "G5b dt<0 不产生 NaN（但会反向积分，无校验，P1-1）");
    }
}

// ───────────────────────── H. double 实例化 ─────────────────────────

static void test_double()
{
    section("H. double 实例化与两条 normalize 路径");

    Quat<double> q = Quat<double>::from_euler(0.3, -0.5, 1.1);
    near(q.norm(), 1.0, 1e-15, "H1  Quat<double> 单位模长（精确 inv_sqrt 路径）");

    const Vec3<double> v(1.0, -2.0, 0.5);
    const Vec3<double> r = q.rotate(v);
    near(r.norm(), v.norm(), 1e-14, "H2  Quat<double> rotate 保模长");

    Quat<double> qq = Quat<double>::identity();
    qq.integrate(Vec3<double>(0, 0, 1.0), 1.0);
    near(qq.norm(), 1.0, 1e-15, "H3  Quat<double> 积分后单位模长");

    // H4 两条归一化路径的精度差（P1-2）：Quat 用位魔法（~4e-6），Vec3 用 1/sqrt（~1e-7）
    {
        Quatf qf1 = Quatf::from_euler(0.3f, -0.5f, 1.1f);
        qf1.normalize();
        const double quat_dev = std::fabs(static_cast<double>(qf1.norm()) - 1.0);
        Vec3f vf1(1.0f, -2.0f, 0.5f);
        vf1.normalize();
        const double vec_dev = std::fabs(static_cast<double>(vf1.norm()) - 1.0);
        std::printf("  [INFO] 归一化后模长偏离：Quat(位魔法)=%.3g   Vec3(精确)=%.3g   相差 %.0f 倍\n",
                    quat_dev, vec_dev, quat_dev / (vec_dev > 0 ? vec_dev : 1e-12));
        check(quat_dev < 1e-5, "H4a Quat::normalize 偏离 < 1e-5（位魔法可达）");
        check(vec_dev < 1e-6, "H4b Vec3::normalize 偏离 < 1e-6（精确路径更严）");
        check(quat_dev > vec_dev, "H4c 记录现状：两条路径精度不一致（P1-2）");
    }
}

// ───────────────────────── main ─────────────────────────

int main()
{
    std::printf("════════ core/math 独立审计测试 ════════\n");
    std::printf("独立性：自写参考实现（4x4 矩阵 Hamilton / Rodrigues）+ Python 生成金标数据\n");

    test_scalar_ops();
    test_vec3();
    test_quat_algebra();
    test_quat_rotate();
    test_euler();
    test_integrate();
    test_degenerate();
    test_double();

    std::printf("\n════════ 汇总：%d PASS / %d FAIL ════════\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
