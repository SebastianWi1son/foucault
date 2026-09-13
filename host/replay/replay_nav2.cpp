// foucault host/replay/replay_nav2.cpp —— NAV2 数据集回放对比（AI 编写，2026-08-30）
//
// 用法：replay_nav2 <数据集路径> [输出CSV路径] [-y 模式] [--fault 故障]
//   -y none      不注入外部航向（默认，6 轴基线）
//   -y gt        注入真值 yaw @50Hz（理想里程计）
//   -y gt_slow   注入真值 yaw @10Hz（每 5 帧一次，验证多速率）
//   -y gt_noisy  注入真值 yaw + σ=2° 噪声 @50Hz（验证不爆炸）
//   -y gt_drop   50Hz，但 t∈[100,130)s 断线（验证 age 门控降级）
//   --fault none|acc_nan|acc_zero|acc_sat|gyro_nan  每 100 帧注入一帧脏数据
//                 （批次 4a-5 验收：实测一帧脏数据的后果 + 守门人的拦截效果）
// 数据集默认在 data/NAV2_data.bin（源：参考库 NAV2 数据集，reference/ 不入 git）
//
// 数据集格式（扩展名 .bin 实为文本）：每行 12 列，空格分隔
//   0-2 acc3 (g)   3-5 gyro3 (rad/s)   6-8 mag3 (本批忽略)   9-11 真值 euler3 (rad, roll/pitch/yaw)
// 采样率 50Hz（dt = 0.02s）
//
// 坐标系约定（重要，实测确认 2026-08-30）：
//   数据集 = 世界系 z-down（NED 风格）：静止水平时 acc = (0,0,-1)
//   本 core  = 世界系 z-up：静止水平时 acc 应为 (0,0,+1)
//   → 喂入前对 acc 取反（gyro 是机体系物理量，无需翻转；roll/pitch 真值
//     只由重力方向定义，与水平轴选择无关，可直接对比）
//
// 输出：roll/pitch 的 RMSE 与 MAX（度）+ yaw 漂移量（去掉初始偏差后的增长）
// 预期：6 轴 Mahony 下 roll/pitch 收敛有界（RMSE < 5°），yaw 漂移 = 预期行为（yaw 不可观）
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "estimator.hpp"
#include "measure/measure.hpp"

using namespace foucault;
using math::Vec3f;

namespace {

constexpr float kDt = 0.02f;      // 数据集采样周期（50Hz）
constexpr float kDeg = 57.29578f; // rad → deg

// 外部航向注入模式（批次 4a 验收）
enum class HeadingMode { none, gt, gt_slow, gt_noisy, gt_drop };

// 故障注入模式（批次 4a-5 验收）
enum class FaultKind { none, acc_nan, acc_zero, acc_sat, gyro_nan };
constexpr size_t kFaultPeriod = 100;          // 每 N 帧注入一帧脏数据

// 确定性伪随机（无需 <random>，保证回放可复现）
inline float pseudo_noise(unsigned i) {
    unsigned x = i * 1664525u + 1013904223u;      // LCG
    x ^= x >> 16;
    return (float)(x & 0xFFFFu) / 65535.0f * 2.0f - 1.0f;   // [-1,1]
}

struct Row
{
    Vec3f acc_;
    Vec3f gyro_;
    Vec3f gt_;   // 真值 euler（rad）
};

bool load_dataset(const char* path, std::vector<Row>& rows)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string line;
    while (std::getline(f, line))
    {
        std::istringstream ss(line);
        double v[12];
        int n = 0;
        while (n < 12 && (ss >> v[n])) ++n;
        if (n < 12) continue;
        rows.push_back({Vec3f((float)v[0], (float)v[1], (float)v[2]),
                        Vec3f((float)v[3], (float)v[4], (float)v[5]),
                        Vec3f((float)v[9], (float)v[10], (float)v[11])});
    }
    return !rows.empty();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf("用法: replay_nav2 <数据集路径> [输出CSV路径] [-y none|gt|gt_slow|gt_noisy|gt_drop]"
                    " [--fault none|acc_nan|acc_zero|acc_sat|gyro_nan]\n");
        return 1;
    }

    // 解析 -y 模式 + 位置参数（数据集 / CSV）
    HeadingMode heading_mode = HeadingMode::none;
    FaultKind   fault        = FaultKind::none;
    const char* data_path = nullptr;
    const char* csv_path = nullptr;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "-y" && i + 1 < argc)
        {
            std::string m = argv[++i];
            if      (m == "none")     heading_mode = HeadingMode::none;
            else if (m == "gt")       heading_mode = HeadingMode::gt;
            else if (m == "gt_slow")  heading_mode = HeadingMode::gt_slow;
            else if (m == "gt_noisy") heading_mode = HeadingMode::gt_noisy;
            else if (m == "gt_drop")  heading_mode = HeadingMode::gt_drop;
            else { std::printf("未知 -y 模式: %s\n", m.c_str()); return 1; }
        }
        else if (std::string(argv[i]) == "--fault" && i + 1 < argc)
        {
            std::string m = argv[++i];
            if      (m == "none")      fault = FaultKind::none;
            else if (m == "acc_nan")   fault = FaultKind::acc_nan;
            else if (m == "acc_zero")  fault = FaultKind::acc_zero;
            else if (m == "acc_sat")   fault = FaultKind::acc_sat;
            else if (m == "gyro_nan")  fault = FaultKind::gyro_nan;
            else { std::printf("未知 --fault 模式: %s\n", m.c_str()); return 1; }
        }
        else if (!data_path) data_path = argv[i];
        else if (!csv_path)  csv_path  = argv[i];
    }
    if (!data_path) { std::printf("缺少数据集路径\n"); return 1; }

    std::vector<Row> rows;
    if (!load_dataset(data_path, rows))
    {
        std::printf("无法打开或解析数据集: %s\n", data_path);
        return 1;
    }
    std::printf("数据集: %zu 行 (%.1f s @50Hz)\n", rows.size(), rows.size() * kDt);

    // 初始对准：yaw 用真值起步（F5 外部指定模式；回放评估相对误差），
    // roll/pitch 由加速度计在头一秒内自然收敛
    Estimator<> est(Dimension::d3);
    est.reset(math::Quatf::from_euler(0, 0, rows[0].gt_.z_));

    // 统计量
    double sum_r = 0, sum_p = 0;
    double max_r = 0, max_p = 0;
    double yaw_err0 = 0, yaw_errN = 0;
    double sum_y = 0, max_y = 0;          // 有外部航向时的 yaw RMSE/MAX（相对真值）
    size_t n_heading_used = 0;                // 实际被估计器采纳的参考帧数
    size_t n_fault = 0;                       // 注入的脏帧数（批次 4a-5）
    const size_t n = rows.size();

    std::FILE* csv = csv_path ? std::fopen(csv_path, "w") : nullptr;
    if (csv)
        std::fprintf(csv, "t,roll,pitch,yaw,gt_roll,gt_pitch,gt_yaw\n");

    for (size_t i = 0; i < n; ++i)
    {
        // 坐标系适配：数据集 acc 是世界系 z-down 约定，本 core 是 z-up → 取反
        measure::IMUSample s{-rows[i].acc_, rows[i].gyro_};
        // 批次 4a-5 验收：故障注入（每 kFaultPeriod 帧一帧脏数据）
        if (fault != FaultKind::none && i > 0 && i % kFaultPeriod == 0)
        {
            switch (fault)
            {
                case FaultKind::acc_nan:  s.acc_  = Vec3f(NAN, NAN, NAN);    break;
                case FaultKind::acc_zero: s.acc_  = Vec3f(0.0f, 0.0f, 0.0f); break;
                case FaultKind::acc_sat:  s.acc_  = Vec3f(0.0f, 0.0f, 9.0f);  break;   // 9 g 饱和
                case FaultKind::gyro_nan: s.gyro_ = Vec3f(NAN, NAN, NAN);    break;
                default: break;
            }
            ++n_fault;
        }
        est.observe(s);

        // 批次 4a：外部航向注入（用真值 yaw 列当"理想里程计"）
        bool feed_heading = false, gap = false;
        float heading_ref = rows[i].gt_.z_;
        switch (heading_mode)
        {
            case HeadingMode::none:     break;
            case HeadingMode::gt:       feed_heading = true; break;
            case HeadingMode::gt_slow:  feed_heading = (i % 5 == 0); break;          // 10Hz
            case HeadingMode::gt_noisy: feed_heading = true;
                                    heading_ref += pseudo_noise((unsigned)i) * 0.0349f;  // σ≈2°（幅 ±2°）
                                    break;
            case HeadingMode::gt_drop:  feed_heading = true;
                                    gap = (i * kDt >= 100.0f && i * kDt < 130.0f);
                                    break;
        }
        if (gap) feed_heading = false;
        if (feed_heading) { est.observe_heading(heading_ref); ++n_heading_used; }

        est.predict(s, kDt);
        Vec3f e = est.euler();

        double er = (e.x_ - rows[i].gt_.x_) * kDeg;
        double ep = (e.y_ - rows[i].gt_.y_) * kDeg;
        double ey = (e.z_ - rows[i].gt_.z_) * kDeg;

        sum_r += er * er;
        sum_p += ep * ep;
        if (std::fabs(er) > max_r) max_r = std::fabs(er);
        if (std::fabs(ep) > max_p) max_p = std::fabs(ep);
        if (i == 0) yaw_err0 = ey;
        if (i == n - 1) yaw_errN = ey;
        sum_y += ey * ey;
        if (std::fabs(ey) > max_y) max_y = std::fabs(ey);

        if (csv)
            std::fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                         i * kDt, e.x_ * kDeg, e.y_ * kDeg, e.z_ * kDeg,
                         rows[i].gt_.x_ * kDeg, rows[i].gt_.y_ * kDeg, rows[i].gt_.z_ * kDeg);
    }
    if (csv) std::fclose(csv);

    double rms_r = std::sqrt(sum_r / n);
    double rms_p = std::sqrt(sum_p / n);
    double yaw_drift = yaw_errN - yaw_err0;

    std::printf("\n===== 回放报告（Estimator<Mahony>，6 轴 acc+gyro，%zu 行）=====\n", n);
    std::printf("roll : RMSE %6.3f°  MAX %6.3f°\n", rms_r, max_r);
    std::printf("pitch: RMSE %6.3f°  MAX %6.3f°\n", rms_p, max_p);
    double rms_y = std::sqrt(sum_y / n);
    if (heading_mode == HeadingMode::none)
        std::printf("yaw  : 漂移 %+7.3f°（初始偏差已扣除；6 轴下 yaw 不可观，漂移为预期行为）\n", yaw_drift);
    else
        std::printf("yaw  : RMSE %6.3f°  MAX %6.3f°  漂移 %+7.3f°（外部航向已注入，%zu 帧参考）\n",
                    rms_y, max_y, yaw_drift, n_heading_used);
    if (csv)
        std::printf("CSV 已导出: %s（供画图：t/roll/pitch/yaw/gt_*，单位度）\n", csv_path);
    std::printf("守门人: 注入脏帧 %zu 帧，拦截 %u 帧%s\n", n_fault, est.rejected_count(),
                n_fault ? (est.rejected_count() == n_fault ? "（全部拦下 ✓）" : "（★ 有漏网）") : "");

    // 验收：roll/pitch RMSE 有界（真值对照）
    bool ok = rms_r < 5.0 && rms_p < 5.0;
    std::printf("\n[%s] ① roll/pitch RMSE < 5°（无退化）\n", ok ? "PASS" : "FAIL");
    if (heading_mode != HeadingMode::none)
    {
        bool ok_y = rms_y < 2.0;
        ok = ok && ok_y;
        std::printf("[%s] ② yaw RMSE < 2°（当前 %.3f°）\n", ok_y ? "PASS" : "FAIL", rms_y);
    }
    return ok ? 0 : 1;
}
