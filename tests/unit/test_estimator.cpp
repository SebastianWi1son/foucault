// foucault tests/unit/test_estimator.cpp —— 门面行为测试（AI 编写，2026-08-30）
// 锚点（F3）：门面与直接调用等价、observe_heading 无害、reset 指定姿态、Dimension 构造
#include <cmath>
#include <cstdio>
#include "estimator.hpp"
#include "measure/measure.hpp"

using namespace foucault;
using math::Quatf;
using math::Vec3f;

static int g_fail = 0;

static void expect(bool ok, const char* name)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++g_fail;
}

int main()
{
    // 1) 门面与直接调用 Mahony 等价：同一串输入，输出一致
    {
        Estimator<> est(Dimension::d2);
        solver::Mahony raw(make_mahony_config(Dimension::d2));
        for (int i = 0; i < 200; ++i)
        {
            measure::IMUSample s{Vec3f(0.01f, 0.02f, 0.99f), Vec3f(0.05f, -0.02f, 0.10f)};
            est.observe(s);
            est.predict(s, 0.02f);
            raw.observe(s.acc_);
            raw.predict(s.gyro_, 0.02f);
        }
        Vec3f e1 = est.euler(), e2 = raw.euler();
        expect(std::fabs(e1.x_ - e2.x_) < 1e-6f &&
               std::fabs(e1.y_ - e2.y_) < 1e-6f &&
               std::fabs(e1.z_ - e2.z_) < 1e-6f, "门面输出 == 直接调用 Mahony");
    }

    // 2) observe_heading 已是【真实通道】（批次 4a）：第二参数语义由 dt 改为 trust（DESIGN §3.7.8 Y5）
    {
        Estimator<> est;                            // a) 门面转发有效：参考能把 yaw 拉过去
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        est.reset(Quatf::from_euler(0, 0, 0.2f));
        for (int i = 0; i < 1000; ++i)
        {
            // 参考从当前 yaw(0.2) 出发斜坡到 1.0 再保持
            // （若一上来就给 1.0，首次观测只做自动对齐、不产生修正 —— 见 test_mahony 4a-④）
            const float ref = 0.2f + 0.8f * (float)(i < 500 ? i : 500) / 500.0f;
            est.observe(s);
            est.observe_heading(ref);                   // 注意：第二参数是 trust，不是 dt（Y5）
            est.predict(s, 0.02f);
        }
        Vec3f e = est.euler();
        expect(std::fabs(e.x_) < 1e-3f && std::fabs(e.y_) < 1e-3f && std::fabs(e.z_ - 1.0f) < 3e-2f,
               "observe_heading 门面转发：yaw 收敛到外部参考（roll/pitch 不受扰）");
    }
    {
        Estimator<> est;                            // b) 不注入 → 行为不变（优雅降级）
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        est.reset(Quatf::from_euler(0, 0, 0.2f));
        for (int i = 0; i < 500; ++i) { est.observe(s); est.predict(s, 0.02f); }
        expect(std::fabs(est.euler().z_ - 0.2f) < 1e-4f, "未注入航向：yaw 保持初值（优雅降级）");
    }
    {
        Estimator<> est;                            // c) trust = 0 → 参考被完全忽略
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        est.reset(Quatf::from_euler(0, 0, 0.2f));
        for (int i = 0; i < 500; ++i)
        {
            est.observe(s);
            est.observe_heading(1.0f, 0.0f);            // trust = 0
            est.predict(s, 0.02f);
        }
        expect(std::fabs(est.euler().z_ - 0.2f) < 1e-4f, "trust = 0：参考被完全忽略");
    }

    // 3) reset(q)：指定初始姿态生效（F5 外部指定模式）
    {
        Estimator<> est;
        Quatf q = Quatf::from_euler(0.3f, 0, 0);
        est.reset(q);
        Vec3f e = est.euler();
        expect(std::fabs(e.x_ - 0.3f) < 1e-5f && std::fabs(e.y_) < 1e-5f, "reset(q) 指定初始姿态生效");
    }

    // 4) 三种 Dimension 构造 + 运行正常
    {
        Estimator<> g(Dimension::d2), c(Dimension::d25), i(Dimension::d3);
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        g.observe(s); g.predict(s, 0.02f);
        c.observe(s); c.predict(s, 0.02f);
        i.observe(s); i.predict(s, 0.02f);
        Vec3f eg = g.euler(), ec = c.euler(), ei = i.euler();
        expect(eg.norm() < 1e-3f && ec.norm() < 1e-3f && ei.norm() < 1e-3f,
               "三种 Dimension 构造 + 运行正常");
    }

    if (g_fail == 0)
    {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILED\n", g_fail);
    return 1;
}
