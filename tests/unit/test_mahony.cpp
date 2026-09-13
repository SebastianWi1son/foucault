// foucault tests/unit/test_mahony.cpp —— Mahony 求解器行为测试（AI 编写）
// 锚点（F3）：静止保持、恒定横滚收敛、纯积分、复位 | 批次 4a：航向注入（漂移/锁定/跟随/对齐/失效/正交）
// 批次 4a-5：脏数据守门人（NaN / 全零 / 饱和 / dt 非法 / 干净数据零干预）
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
        f.observe(Vec3f(0, 0, 1));           // 静止量测，残差 = 0
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

    // ═══ 批次 4a：外部航向注入（DESIGN §3.7）═══
    const float DT = 0.02f;
    const Vec3f gyro_bias(0, 0, 0.05f);      // z 轴 0.05 rad/s 零偏（模拟陀螺漂移）

    // 5) 无参考 → yaw 自由漂移（基线：证明后面几项不是白测的）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        for (int i = 0; i < 1000; ++i) f.update(gyro_bias, Vec3f(0, 0, 1), DT);   // 20s
        float d = f.euler().z_ - 0.3f;
        expect(d > 0.9f && d < 1.1f, "4a-① 无外部参考：yaw 自由漂移 ≈ +1.0 rad/20s");
    }

    // 6) 恒定参考 → yaw 被锁住（稳态误差 = 零偏/Kp_heading = 0.05/5 = 0.01 rad）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        for (int i = 0; i < 1000; ++i)
        {
            f.observe(Vec3f(0, 0, 1));
            f.observe_heading(0.3f);
            f.predict(gyro_bias, DT);
        }
        float err = f.euler().z_ - 0.3f;
        expect(std::fabs(err - 0.01f) < 3e-3f, "4a-② 恒定参考：yaw 锁定，稳态误差 ≈ 零偏/Kp_heading = 0.01 rad");
    }

    // 7) 参考变化 → ψ̂ 跟随（保持初始 offset）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        for (int i = 0; i < 2000; ++i)
        {
            float ref = 0.3f + 0.7f * (float)(i < 1000 ? i : 1000) / 1000.0f;   // 0.3 → 1.0
            f.observe(Vec3f(0, 0, 1));
            f.observe_heading(ref);
            f.predict(gyro_bias, DT);
        }
        expect(std::fabs(f.euler().z_ - 1.0f) < 3e-2f, "4a-③ 参考 0.3→1.0：ψ̂ 跟随（误差 < 0.03 rad）");
    }

    // 8) 首次观测自动对齐：参考起源与估计器初始值差很远【不猛转】，只跟踪变化量
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        f.observe(Vec3f(0, 0, 1));
        f.observe_heading(10.0f);                 // 起源差 9.7 rad
        f.predict(Vec3f(0, 0, 0), DT);
        bool no_jerk = std::fabs(f.euler().z_ - 0.3f) < 1e-3f;
        for (int i = 0; i < 1000; ++i)
        {
            f.observe(Vec3f(0, 0, 1));
            f.observe_heading(10.3f);             // 参考动了 +0.3
            f.predict(Vec3f(0, 0, 0), DT);
        }
        expect(no_jerk && std::fabs(f.euler().z_ - 0.6f) < 3e-2f,
               "4a-④ 自动对齐：起源不同不猛转，只跟踪 Δ（0.3 → 0.6）");
    }

    // 9) 参考失效（超过 heading_timeout）→ 退化为纯陀螺积分（age 门控）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        for (int i = 0; i < 500; ++i)          // 先锁定
        {
            f.observe(Vec3f(0, 0, 1)); f.observe_heading(0.3f); f.predict(gyro_bias, DT);
        }
        bool valid_before = f.is_heading_valid();
        for (int i = 0; i < 100; ++i) f.predict(gyro_bias, DT);   // 断参考 2s（> 0.3s 超时）
        bool invalid_after = !f.is_heading_valid();
        float yaw_at_drop = f.euler().z_;
        for (int i = 0; i < 500; ++i) f.predict(gyro_bias, DT);   // 再纯积分 10s
        float drift = f.euler().z_ - yaw_at_drop;
        expect(valid_before && invalid_after && std::fabs(drift - 0.5f) < 5e-2f,
               "4a-⑤ 参考失效：is_heading_valid() 变 false，退化为纯陀螺积分（+0.5 rad/10s）");
    }

    // 10) §3.7.2 正交性 + 一阶 P 的定量行为
    //     注意：参考必须从"当前估计值"出发（否则首次观测只做自动对齐、不产生修正，见 4a-④）
    {
        solver::Mahony a, b;
        const Quatf q0 = Quatf::from_euler(0.2f, -0.1f, 0);
        const Vec3f acc = q0.conjugated().rotate(Vec3f(0, 0, 1));
        a.reset(q0);
        b.reset(q0);
        const int N_RAMP = 250;                    // 5s 斜坡：0 → 1.0 rad（斜率 0.2 rad/s）
        float lag_at_ramp_end = 0.0f;
        for (int i = 0; i < 2 * N_RAMP; ++i)
        {
            const float ref = (i < N_RAMP) ? (1.0f * (float)i / (float)N_RAMP) : 1.0f;
            a.observe(acc); a.predict(Vec3f(0, 0, 0), DT);                       // 纯 6 轴
            b.observe(acc); b.observe_heading(ref); b.predict(Vec3f(0, 0, 0), DT);   // 加航向通道
            if (i == N_RAMP - 1) lag_at_ramp_end = 1.0f - b.euler().z_;
        }
        Vec3f ea = a.euler(), eb = b.euler();
        // ① 正交性：航向通道对 roll/pitch 零影响
        expect(std::fabs(ea.x_ - eb.x_) < 1e-6f && std::fabs(ea.y_ - eb.y_) < 1e-6f,
               "4a-⑥ 正交性：航向通道对 roll/pitch 零影响（实测逐位相同，容差 1e-6）");
        // ② 一阶 P 的斜坡滞后 = 斜率 × τ，τ = 1/Kp_heading = 0.2s → 0.2 × 0.2 = 0.04 rad
        expect(std::fabs(lag_at_ramp_end - 0.04f) < 5e-3f,
               "4a-⑦ 斜坡跟随滞后 = 斜率×τ = 0.04 rad（与一阶 P 理论精确吻合）");
        // ③ 斜坡结束后保持 → 残差被吃干净
        expect(std::fabs(eb.z_ - 1.0f) < 1e-3f, "4a-⑧ 参考保持后 yaw 收敛到 1.0（残留 < 1e-3）");
    }

    // 10b) 参考【恢复】：断线期间的漂移必须被拉回
    //      ← 这条专门锁死"对齐只做一次"：若把 is_heading_aligned_ 换成 is_heading_valid()，
    //        每次参考失效后的第一条 observe_heading 都会【重新对齐】→ 漂移被固化成新基准（永久偏）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0, 0, 0.3f));
        for (int i = 0; i < 500; ++i)                 // ① 先锁定
        {
            f.observe(Vec3f(0, 0, 1)); f.observe_heading(0.3f); f.predict(gyro_bias, DT);
        }
        for (int i = 0; i < 500; ++i) f.predict(gyro_bias, DT);   // ② 断参考 10s → 漂移 +0.5
        const float drifted = f.euler().z_;
        for (int i = 0; i < 2000; ++i)                // ③ 参考恢复（真值仍是 0.3）
        {
            f.observe(Vec3f(0, 0, 1)); f.observe_heading(0.3f); f.predict(gyro_bias, DT);
        }
        expect(std::fabs(drifted - 0.8f) < 0.1f && std::fabs(f.euler().z_ - 0.3f) < 3e-2f,
               "4a-⑩ 参考恢复：断线漂移被拉回 0.3（对齐只做一次，不因失效重来）");
    }

    // 11) 残差限幅 ±π/2：参考【突变】时修正有界（防视觉误识别/里程计跳变造成猛转）
    {
        solver::Mahony f;
        f.reset();                                  // yaw = 0
        f.observe(Vec3f(0, 0, 1));
        f.observe_heading(0.0f);                        // 先对齐（offset = 0）
        f.predict(Vec3f(0, 0, 0), DT);
        f.observe(Vec3f(0, 0, 1));
        f.observe_heading(3.0f);                        // 参考突跳 +3.0 rad（> π/2）
        f.predict(Vec3f(0, 0, 0), DT);
        const float after_one = f.euler().z_;
        // 单帧修正量上限 = Kp_heading × clamp(π/2) × dt = 5 × 1.5708 × 0.02 = 0.157 rad
        expect(after_one > 0.0f && after_one < 0.16f,
               "4a-⑨ 残差限幅：参考突跳 3 rad，单帧修正被限在 ±Kp·(π/2)·dt = 0.157 内");
    }

    // 12) trust 越界防护：trust 是【控制输入】，负值必须被 clamp 到 0，不得让它反向驱动
    //     （实测：不 clamp 时 trust=-1 会让 yaw 从 0 发散到 −2.28 rad）
    {
        solver::Mahony f;
        f.reset();
        for (int i = 0; i < 1000; ++i)
        {
            const float ref = 1.0f * (float)i / 1000.0f;   // 参考 0 → 1.0
            f.observe(Vec3f(0, 0, 1));
            f.observe_heading(ref, -1.0f);                     // 非法 trust（负）
            f.predict(Vec3f(0, 0, 0), DT);
        }
        expect(std::fabs(f.euler().z_) < 0.1f,
               "4a-⑪ trust 越界：负 trust 被 clamp 到 0（参考被忽略，不发散）");
    }

    // 13) acc 失效：一帧"坏"量测之后断线 → 冻结的残差【不得】被持续复用（P0-4 判别测试）
    //     旧结构（bool is_meas_ 闩锁）：残差永不失效 → 每周期都加 Kp·e → 持续朝错误方向转过去
    {
        solver::Mahony f;                                    // 起始水平，acc 竖直
        for (int i = 0; i < 500; ++i) { f.observe(Vec3f(0, 0, 1)); f.predict(Vec3f(0, 0, 0), DT); }
        f.observe(Vec3f(0.5f, 0.0f, 0.866f));                // 一帧"倾斜"量测（残差很大，想把 pitch 拉到 30°）
        f.predict(Vec3f(0, 0, 0), DT);
        for (int i = 0; i < 500; ++i) f.predict(Vec3f(0, 0, 0), DT);   // 断线 10s（远超 acc_timeout_ = 0.1s）
        const float after = f.euler().y_;
        expect(!f.is_acc_valid() && std::fabs(after) < 0.35f,
               "13) acc 失效：坏残差在超时后停止被复用（判据：pitch 停在小幅，而非一路上冲）");
    }

    // 14) 统一出口的"零契约"：两个通道都不在线 → 两个修正项都为 0
    //     （门控语义 = "该项取 0"，不是"跳过一段代码"；predict = gyro + 0 + 0）
    {
        solver::Mahony f;
        f.reset(Quatf::from_euler(0.3f, 0.2f, 0.1f));      // 非平凡初值
        for (int i = 0; i < 100; ++i) f.predict(Vec3f(0, 0, 0), DT);   // 无任何量测，陀螺也全 0
        Vec3f e = f.euler();
        const bool still = std::fabs(e.x_ - 0.3f) < 1e-4f && std::fabs(e.y_ - 0.2f) < 1e-4f
                        && std::fabs(e.z_ - 0.1f) < 1e-4f;
        expect(!f.is_acc_valid() && !f.is_heading_valid() && still,
               "14) 统一出口零契约：两通道都不在线 → 修正项为 0，姿态原地不动");
    }

    // 15) 手动重新对齐 align_heading：只动"航向零点"，不动 roll/pitch；且不产生跳变
    //     ⚠ 判别点在对齐【之后】的参考空窗期 —— align_heading 本身不动 q_，
    //       所以"对齐瞬间 euler() 不变"是同义反复，抓不到任何东西。
    //       真正的风险：offset 换成新原点后，若 heading_ref_ 没跟着更新，
    //       下一个 predict 会用【旧 ref】算残差 → 目标错成 ψ̂−9.7 → 姿态被打飞。
    {
        solver::Mahony f;
        const Quatf q0 = Quatf::from_euler(0.2f, -0.1f, 0.3f);
        const Vec3f acc = q0.conjugated().rotate(Vec3f(0, 0, 1));
        f.reset(q0);
        for (int i = 0; i < 500; ++i)                       // ① 正常工作（首次自动兜底对齐）
        {
            f.observe(acc); f.observe_heading(0.3f); f.predict(Vec3f(0, 0, 0), DT);
        }
        const Vec3f before = f.euler();

        f.align_heading(10.0f);                             // ② 参考换新原点（10.0）
        float worst_yaw = 0.0f, worst_rp = 0.0f;
        for (int i = 0; i < 10; ++i)                        // ②′ 参考空窗期（0.2s < timeout 0.3s）
        {
            f.observe(acc); f.predict(Vec3f(0, 0, 0), DT);  //    故意不调 observe_heading
            const Vec3f e = f.euler();
            worst_yaw = std::fmax(worst_yaw, std::fabs(e.z_ - before.z_));
            worst_rp  = std::fmax(worst_rp, std::fmax(std::fabs(e.x_ - 0.2f), std::fabs(e.y_ + 0.1f)));
        }
        const bool no_jump = worst_yaw < 1e-4f && worst_rp < 2e-3f;

        for (int i = 0; i < 1000; ++i)                      // ③ 之后参考从 10.0 → 10.5
        {
            const float ref = 10.0f + 0.5f * (float)(i < 500 ? i : 500) / 500.0f;
            f.observe(acc); f.observe_heading(ref); f.predict(Vec3f(0, 0, 0), DT);
        }
        const Vec3f end = f.euler();
        expect(no_jump && std::fabs(end.z_ - 0.8f) < 3e-2f && std::fabs(end.x_ - 0.2f) < 2e-2f,
               "15) 手动重新对齐：参考空窗期不跳变 → 跟踪新参考（0.3→0.8），roll/pitch 不受扰");
    }

    // ═══════════════ 批次 4a-5：脏数据守门人 ═══════════════
    // 16) NaN 量测一律丢弃（不污染状态）
    {
        solver::Mahony f;
        for (int i = 0; i < 100; ++i) { f.observe(Vec3f(0, 0, 1)); f.predict(Vec3f(0, 0, 0), DT); }
        const float yaw0 = f.euler().z_;
        const float nan = std::nanf("");
        for (int i = 0; i < 50; ++i)
        {
            f.observe(Vec3f(nan, nan, nan));                 // 脏 acc
            f.predict(Vec3f(nan, nan, nan), DT);             // 脏 gyro（dt 合法）
        }
        const Vec3f e = f.euler();
        const bool finite = std::isfinite(e.x_) && std::isfinite(e.y_) && std::isfinite(e.z_);
        // 判别：无守门人时 q_ 直接变 NaN（回放实测 roll RMSE = nan）
        expect(finite && f.rejected_count() == 100 && std::fabs(e.z_ - yaw0) < 1e-4f,
               "16) 守门人：50 帧 NaN acc+gyro 全部丢弃（计数 100），状态保持有限且不漂");
    }

    // 17) acc = (0,0,0)（掉线读回全零）→ 拒绝 + 门控【自然】关闭
    {
        solver::Mahony f;
        for (int i = 0; i < 100; ++i) { f.observe(Vec3f(0, 0, 1)); f.predict(Vec3f(0, 0, 0), DT); }
        expect(f.is_acc_valid(), "17a) 前置：正常喂入时 acc 通道有效");
        const unsigned c0 = f.rejected_count();
        for (int i = 0; i < 20; ++i)                         // 20 帧 = 0.4s > acc_timeout_ 0.1s
        {
            f.observe(Vec3f(0, 0, 0));
            f.predict(Vec3f(0, 0, 0), DT);
        }
        // 判别：无守门人时 Vec3::normalize 的零向量防御让 acc_=(0,0,0)
        //       → 残差恒 0，但 acc_age_ 被清零 → is_acc_valid() 【谎报有效】（P1-1 静默失效）
        expect(!f.is_acc_valid() && f.rejected_count() == c0 + 20,
               "17) 守门人：acc 全零被拒 → 年龄继续增长 → 门控自然关闭（无守门人则谎报有效）");
    }

    // 18) |acc| 越界（饱和 / 过小）被拒，合格帧放行
    {
        solver::Mahony f;
        for (int i = 0; i < 100; ++i) { f.observe(Vec3f(0, 0, 1)); f.predict(Vec3f(0, 0, 0), DT); }
        const unsigned c0 = f.rejected_count();
        f.observe(Vec3f(0, 0, 9.0f));          // 9 g 饱和（acc_max_ = 2 g）
        f.observe(Vec3f(0, 0, 0.1f));          // 0.1 g 过小（acc_min_ = 0.5 g）
        f.observe(Vec3f(0, 0, 1.0f));          // 1 g 合格
        expect(f.rejected_count() == c0 + 2 && f.is_acc_valid(),
               "18) 守门人：|acc| 越界（9g / 0.1g）被拒，合格帧放行并续期");
    }

    // 19) dt 非法（0 / 负 / NaN）→ 整帧不推进（时钟倒退防护）
    {
        solver::Mahony f;
        for (int i = 0; i < 100; ++i) { f.observe(Vec3f(0, 0, 1)); f.predict(Vec3f(0, 0, 0), DT); }
        const Vec3f before = f.euler();
        const unsigned c0 = f.rejected_count();
        f.predict(Vec3f(0.1f, 0, 0), 0.0f);                  // dt = 0
        f.predict(Vec3f(0.1f, 0, 0), -DT);                   // dt < 0（时钟倒退）
        f.predict(Vec3f(0.1f, 0, 0), std::nanf(""));         // dt = NaN
        const Vec3f after = f.euler();
        const bool same = std::fabs(after.x_ - before.x_) < 1e-6f
                       && std::fabs(after.y_ - before.y_) < 1e-6f
                       && std::fabs(after.z_ - before.z_) < 1e-6f;
        // 判别：无守门人时 dt=NaN 会让 q_ 变 NaN；dt<0 会倒着积分
        expect(same && f.rejected_count() == c0 + 3,
               "19) 守门人：dt = 0 / 负 / NaN → 整帧不推进，姿态零变化（计数 +3）");
    }

    // 20) 干净数据零干预；reset() 清零计数
    {
        solver::Mahony f;
        const Quatf q0 = Quatf::from_euler(0.2f, -0.1f, 0.3f);
        const Vec3f acc = q0.conjugated().rotate(Vec3f(0, 0, 1));
        f.reset(q0);
        for (int i = 0; i < 500; ++i)
        {
            f.observe(acc); f.observe_heading(0.3f); f.predict(Vec3f(0, 0, 0), DT);
        }
        const bool clean = (f.rejected_count() == 0);        // 干净数据一帧不拦
        const float nan = std::nanf("");
        for (int i = 0; i < 3; ++i) { f.observe(Vec3f(nan, 0, 0)); }
        const unsigned dirty = f.rejected_count();
        f.reset();
        expect(clean && dirty == 3 && f.rejected_count() == 0,
               "20) 守门人：干净数据零干预（计数 0）；3 帧脏数据后 reset() 清零计数");
    }

    if (g_fail == 0)
    {
        std::printf("\nALL PASS\n");
        return 0;
    }
    std::printf("\n%d FAILED\n", g_fail);
    return 1;
}
