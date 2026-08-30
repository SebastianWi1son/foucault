// foucault tests/unit/test_mahony.cpp —— Mahony 求解器行为测试（AI 编写）
// 锚点（F3）：静止保持、恒定横滚收敛、纯积分、复位
#include <cmath>
#include <cstdio>
#include "solver/mahony.hpp"

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
    // 1) 静止水平：加速度计 (0,0,1)（=重力方向），从单位元开始 → 姿态保持零
    {
        solver::Mahony f;
        for (int i = 0; i < 500; ++i) f.update(Vec3f(0, 0, 0), Vec3f(0, 0, 1), 0.002f);
        Vec3f e = f.euler();
        expect(std::fabs(e.x_) < 1e-3f && std::fabs(e.y_) < 1e-3f && std::fabs(e.z_) < 1e-3f,
               "静止水平：姿态保持零");
    }

    // 2) 恒定横滚 30°：目标姿态下重力在机体系的读数 = q_t*⊗g⊗q_t，
    //    从单位元开始（陀螺零速），靠加速度计应收敛到 30°
    {
        solver::Mahony f;
        const float deg = 3.14159265f / 180.0f;
        float roll_target = 30.0f * deg;
        Quatf q_t = Quatf::from_euler(roll_target, 0, 0);
        Vec3f acc = q_t.conjugated().rotate(Vec3f(0, 0, 1));
        for (int i = 0; i < 2000; ++i) f.update(Vec3f(0, 0, 0), acc, 0.002f);
        Vec3f e = f.euler();
        expect(std::fabs(e.x_ - roll_target) < 2e-2f && std::fabs(e.y_) < 2e-2f,
               "横滚 30° 加速度计收敛");
    }

    // 3) 纯陀螺积分（先 observe 一次使残差=0，再纯 predict）：z 轴 1 rad/s，2s → yaw ≈ 2 rad
    {
        solver::Mahony f;
        f.observe(Vec3f(0, 0, 1), 0.01f);           // 静止量测，残差 = 0
        for (int i = 0; i < 200; ++i) f.predict(Vec3f(0, 0, 1), 0.01f);
        Vec3f e = f.euler();
        expect(std::fabs(e.z_ - 2.0f) < 2e-2f, "纯陀螺积分 2s：yaw ≈ 2 rad");
    }

    // 4) 复位：update 产生姿态后 reset → 回单位元
    {
        solver::Mahony f;
        f.update(Vec3f(0, 0, 1), Vec3f(0, 1, 0), 0.01f);
        f.reset();
        Vec3f e = f.euler();
        expect(std::fabs(e.x_) < 1e-6f && std::fabs(e.y_) < 1e-6f && std::fabs(e.z_) < 1e-6f,
               "reset 回零");
    }

    if (g_fail == 0)
    {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILED\n", g_fail);
    return 1;
}
