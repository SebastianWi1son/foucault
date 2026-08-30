// foucault host/replay/replay_nav2.cpp —— NAV2 数据集回放对比（AI 编写，2026-08-30）
//
// 用法：replay_nav2 <数据集路径> [输出CSV路径]
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

struct Row
{
    Vec3f acc;
    Vec3f gyro;
    Vec3f gt;   // 真值 euler（rad）
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
        std::printf("用法: replay_nav2 <数据集路径> [输出CSV路径]\n");
        return 1;
    }

    std::vector<Row> rows;
    if (!load_dataset(argv[1], rows))
    {
        std::printf("无法打开或解析数据集: %s\n", argv[1]);
        return 1;
    }
    std::printf("数据集: %zu 行 (%.1f s @50Hz)\n", rows.size(), rows.size() * kDt);

    // 初始对准：yaw 用真值起步（F5 外部指定模式；回放评估相对误差），
    // roll/pitch 由加速度计在头一秒内自然收敛
    Estimator<> est(Scene::gimbal);
    est.reset(math::Quatf::from_euler(0, 0, rows[0].gt.z_));

    // 统计量
    double sum_r = 0, sum_p = 0;
    double max_r = 0, max_p = 0;
    double yaw_err0 = 0, yaw_errN = 0;
    const size_t n = rows.size();

    std::FILE* csv = (argc > 2) ? std::fopen(argv[2], "w") : nullptr;
    if (csv)
        std::fprintf(csv, "t,roll,pitch,yaw,gt_roll,gt_pitch,gt_yaw\n");

    for (size_t i = 0; i < n; ++i)
    {
        // 坐标系适配：数据集 acc 是世界系 z-down 约定，本 core 是 z-up → 取反
        measure::IMUSample s{-rows[i].acc, rows[i].gyro};
        est.observe(s, kDt);
        est.predict(s, kDt);
        Vec3f e = est.euler();

        double er = (e.x_ - rows[i].gt.x_) * kDeg;
        double ep = (e.y_ - rows[i].gt.y_) * kDeg;
        double ey = (e.z_ - rows[i].gt.z_) * kDeg;

        sum_r += er * er;
        sum_p += ep * ep;
        if (std::fabs(er) > max_r) max_r = std::fabs(er);
        if (std::fabs(ep) > max_p) max_p = std::fabs(ep);
        if (i == 0) yaw_err0 = ey;
        if (i == n - 1) yaw_errN = ey;

        if (csv)
            std::fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                         i * kDt, e.x_ * kDeg, e.y_ * kDeg, e.z_ * kDeg,
                         rows[i].gt.x_ * kDeg, rows[i].gt.y_ * kDeg, rows[i].gt.z_ * kDeg);
    }
    if (csv) std::fclose(csv);

    double rms_r = std::sqrt(sum_r / n);
    double rms_p = std::sqrt(sum_p / n);
    double yaw_drift = yaw_errN - yaw_err0;

    std::printf("\n===== 回放报告（Estimator<Mahony>，6 轴 acc+gyro，%zu 行）=====\n", n);
    std::printf("roll : RMSE %6.3f°  MAX %6.3f°\n", rms_r, max_r);
    std::printf("pitch: RMSE %6.3f°  MAX %6.3f°\n", rms_p, max_p);
    std::printf("yaw  : 漂移 %+7.3f°（初始偏差已扣除；6 轴下 yaw 不可观，漂移为预期行为）\n", yaw_drift);
    if (csv)
        std::printf("CSV 已导出: %s（供画图：t/roll/pitch/yaw/gt_*，单位度）\n", argv[2]);

    // 验收：roll/pitch RMSE 有界（真值对照），yaw 不判失败
    bool ok = rms_r < 5.0 && rms_p < 5.0;
    std::printf("\n[%s] roll/pitch RMSE < 5° 验收%s\n", ok ? "PASS" : "FAIL", ok ? "" : "（不通过，需排查）");
    return ok ? 0 : 1;
}
