// foucault tests/unit/test_math.cpp —— 数学内核行为测试（AI 编写，2026-08-30）
// 测的是行为不变式（单位模长、旋转正确性、积分结果、欧拉往返），不是魔法数（F3 锚点测试）
#include <cmath>
#include <cstdio>
#include "math/scalar_ops.hpp"
#include "math/vec3.hpp"
#include "math/quat.hpp"

using namespace foucault::math;

static int g_fail = 0;

static void expect(bool ok, const char* name)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

int main()
{
    // ── scalar_ops ──
    {
        // 位魔法版相对误差 ~4.4e-6（归一化用途绰绰有余），容差按此放宽
        expect(std::fabs(inv_sqrt(4.0f) - 0.5f) < 1e-5f,  "inv_sqrt(4) ≈ 0.5 (float, 位魔法精度 ~4e-6)");
        expect(std::fabs(inv_sqrt(4.0) - 0.5) < 1e-12,    "inv_sqrt(4) ≈ 0.5 (double)");
        expect(clamp(5.0f, 0.0f, 1.0f) == 1.0f,           "clamp 超上限 → 上限");
        expect(clamp(-1.0f, 0.0f, 1.0f) == 0.0f,          "clamp 超下限 → 下限");
        expect(clamp(0.5f, 0.0f, 1.0f) == 0.5f,           "clamp 区间内不变");
        expect(std::fabs(deg_to_rad(180.0f) - 3.14159265f) < 1e-6f, "deg_to_rad(180) ≈ π");
        expect(std::fabs(rad_to_deg(3.14159265f) - 180.0f) < 1e-4f, "rad_to_deg(π) ≈ 180");
    }

    // ── Vec3 ──
    {
        Vec3f a(1, 2, 3), b(4, 5, 6);
        expect(a.dot(b) == 32.0f, "dot = 1·4+2·5+3·6 = 32");

        Vec3f c = a.cross(b);
        expect(std::fabs(c.x_ - (-3.0f)) < 1e-5f &&
               std::fabs(c.y_ -   6.0f) < 1e-5f &&
               std::fabs(c.z_ - (-3.0f)) < 1e-5f, "cross 数值 (-3, 6, -3)");

        Vec3f u(3, 4, 0);
        expect(std::fabs(u.norm() - 5.0f) < 1e-6f, "norm = 5");
        u.normalize();
        expect(std::fabs(u.norm() - 1.0f) < 1e-6f, "normalize 后模长 = 1");

        Vec3f z(0, 0, 0);
        z.normalize();
        expect(z.norm() == 0.0f, "零向量 normalize 保持零向量（防御）");

        Vec3f s = 2.0f * Vec3f(1, 2, 3);
        expect(s.x_ == 2.0f && s.y_ == 4.0f && s.z_ == 6.0f, "标量×向量（自由函数）");

        Vec3f f(7);
        expect(f.x_ == 7.0f && f.y_ == 7.0f && f.z_ == 7.0f, "fill 构造：三分量 = 7");
    }

    // ── Quat ──
    {
        Quatf q = Quatf::identity();
        expect(std::fabs(q.norm() - 1.0f) < 1e-6f, "单位四元数模长 = 1");

        // 共轭 = 反向旋转：q ⊗ q* = 单位元
        Quatf r = Quatf::from_euler(0.3f, -0.2f, 0.5f);
        Quatf rr = r * r.conjugated();
        expect(std::fabs(rr.q0_ - 1.0f) < 1e-5f &&
               std::fabs(rr.q1_) < 1e-5f &&
               std::fabs(rr.q2_) < 1e-5f &&
               std::fabs(rr.q3_) < 1e-5f, "q ⊗ q* = 单位元");

        // rotate：绕 z 转 90°，x 轴 → y 轴
        Quatf qz = Quatf::from_euler(0, 0, 3.14159265f / 2);
        Vec3f vy = qz.rotate(Vec3f(1, 0, 0));
        expect(std::fabs(vy.x_) < 1e-5f && std::fabs(vy.y_ - 1.0f) < 1e-5f && std::fabs(vy.z_) < 1e-5f,
               "rotate 绕 z 转 90°：x → y");

        Vec3f v(1, 2, 3);
        expect(std::fabs(qz.rotate(v).norm() - v.norm()) < 1e-5f, "rotate 保模长");

        // integrate：ω=0 姿态不变（normalize 用 inv_sqrt，容差同精度）
        Quatf qi = Quatf::identity();
        qi.integrate(Vec3f(0, 0, 0), 0.01f);
        expect(std::fabs(qi.norm() - 1.0f) < 1e-5f, "ω=0 积分姿态不变");

        // integrate：绕 z 1 rad/s 转 1s（100 步）→ yaw ≈ 1 rad
        Quatf qw = Quatf::identity();
        for (int i = 0; i < 100; ++i) qw.integrate(Vec3f(0, 0, 1.0f), 0.01f);
        float rr2, pp2, yy2;
        qw.to_euler(rr2, pp2, yy2);
        expect(std::fabs(yy2 - 1.0f) < 1e-3f &&
               std::fabs(rr2) < 1e-3f &&
               std::fabs(pp2) < 1e-3f, "积分 1s 后 yaw ≈ 1 rad");

        // 欧拉往返：from_euler → to_euler 回到原角度
        float roll = 0.3f, pitch = -0.4f, yaw = 0.8f;
        Quatf qe = Quatf::from_euler(roll, pitch, yaw);
        float r3, p3, y3;
        qe.to_euler(r3, p3, y3);
        expect(std::fabs(r3 - roll) < 1e-5f &&
               std::fabs(p3 - pitch) < 1e-5f &&
               std::fabs(y3 - yaw) < 1e-5f, "欧拉往返");

        // 零四元数 normalize → 单位元（防御）
        Quatf qz2(0, 0, 0, 0);
        qz2.normalize();
        expect(std::fabs(qz2.q0_ - 1.0f) < 1e-6f, "零四元数 normalize → 单位元");
    }

    if (g_fail == 0)
    {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILED\n", g_fail);
    return 1;
}
