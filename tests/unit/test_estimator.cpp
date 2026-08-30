// foucault tests/unit/test_estimator.cpp —— 门面行为测试（AI 编写，2026-08-30）
// 锚点（F3）：门面与直接调用等价、observe_yaw 无害、reset 指定姿态、Scene 构造
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
        Estimator<> est(Scene::car);
        solver::Mahony raw(make_mahony_config(Scene::car));
        for (int i = 0; i < 200; ++i)
        {
            measure::IMUSample s{Vec3f(0.01f, 0.02f, 0.99f), Vec3f(0.05f, -0.02f, 0.10f)};
            est.observe(s, 0.02f);
            est.predict(s, 0.02f);
            raw.observe(s.acc_, 0.02f);
            raw.predict(s.gyro_, 0.02f);
        }
        Vec3f e1 = est.euler(), e2 = raw.euler();
        expect(std::fabs(e1.x_ - e2.x_) < 1e-6f &&
               std::fabs(e1.y_ - e2.y_) < 1e-6f &&
               std::fabs(e1.z_ - e2.z_) < 1e-6f, "门面输出 == 直接调用 Mahony");
    }

    // 2) observe_yaw 无害：调用后姿态仍正常（Mahony 忽略外部 yaw）
    {
        Estimator<> est;
        est.observe_yaw(1.0f, 0.02f);
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        est.observe(s, 0.02f);
        est.predict(s, 0.02f);
        Vec3f e = est.euler();
        expect(std::fabs(e.x_) < 1e-3f && std::fabs(e.y_) < 1e-3f, "observe_yaw 后姿态仍正常");
    }

    // 3) reset(q)：指定初始姿态生效（F5 外部指定模式）
    {
        Estimator<> est;
        Quatf q = Quatf::from_euler(0.3f, 0, 0);
        est.reset(q);
        Vec3f e = est.euler();
        expect(std::fabs(e.x_ - 0.3f) < 1e-5f && std::fabs(e.y_) < 1e-5f, "reset(q) 指定初始姿态生效");
    }

    // 4) 三种 Scene 构造 + 运行正常
    {
        Estimator<> g(Scene::gimbal), c(Scene::car), i(Scene::infantry);
        measure::IMUSample s{Vec3f(0, 0, 1), Vec3f(0, 0, 0)};
        g.observe(s, 0.02f); g.predict(s, 0.02f);
        c.observe(s, 0.02f); c.predict(s, 0.02f);
        i.observe(s, 0.02f); i.predict(s, 0.02f);
        Vec3f eg = g.euler(), ec = c.euler(), ei = i.euler();
        expect(eg.norm() < 1e-3f && ec.norm() < 1e-3f && ei.norm() < 1e-3f,
               "三种 Scene 构造 + 运行正常");
    }

    if (g_fail == 0)
    {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILED\n", g_fail);
    return 1;
}
