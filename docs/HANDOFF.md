# HANDOFF.md — 交接文档（2026-08-30 生成）

> 用途：用户准备刷机（H7 板），本文件供下一个 agent 零上下文快速接手。
> **开工前必读：本文件 + AGENT.md**；冲突时以 AGENT.md 与 docs/ 文档体系为准。

---

## 1. 项目一句话

**foucault = 通用可移植姿态解算算法核心**（C++17、零依赖、core 禁 STL/异常/RTTI/动态内存）：
可插拔增益求解器（Mahony 现役 → EKF 后续）× 可裁剪维度（2D/2.5D/3D，当前仅 3D）× 可注入量测源
（6 轴现役 / 外部 yaw 下一批 / 9 轴磁力计大概率出局），目标 STM32H7，
场景：云台 / 地面小车 / 立体步兵。**当前主线 = 平面+斜坡小车 + 里程计 yaw**。

## 2. 用户（wilson）工作模式（铁律，违反 = 返工）

1. **全程中文**；先讲原理（最土的话 + 类比）；**决策点前置**（列表 + 推荐）→ 用户确认 → 落文档
2. **AI 默认只读源码**：用户抄录 docs/CODE.md 代码进源文件；测试/CMake/回放工具由 AI 写（用户授权）
3. **CODE.md 是唯一施工来源**（代码必须 ```cpp 围栏内）；**文档 = 归档非讨论载体**（答疑在对话）
4. 覆盖/删除前必须可回退（git 已提交 / /tmp 备份），三者皆无 = 禁止
5. **commit 由用户主导**（2026-08-30 定案）：AI 默认不 commit；message 用用户看得懂的中文
6. **外部输入先验证**（坐标系/单位/列顺序；教训见复盘.md）
7. 未完成批次不算完成：改完必回归编译 + 全量 ctest

## 3. 当前状态（验收全绿）

- git：唯一提交 `1360d9d`「里程碑：批次 1-3 完成，Mahony 可独立跑」，main 分支
- ctest 4/4：test_math 21 / test_mahony 4 / test_estimator 4 / replay_nav2（真实数据回归）
- 回放指标（NAV2 数据集 320s@50Hz，6 轴 acc+gyro）：roll RMSE 2.15° / pitch RMSE 2.65° / yaw 漂移 −14.5°
- ⚠️ **工作区 3 个未提交改动**（commit 由用户主导，等他发话）：
  ① docs/待办.md ② .gitignore（课程工程条目清理）③ **Scene→Dimension 全套改名**（config/estimator/测试/回放/文档）

## 4. 代码地图（★ = 用户抄录，AI 只读需授权；☆ = AI 写）

| 文件 | 内容 | 状态 |
|---|---|---|
| `core/math/{scalar_ops,vec3,quat}.hpp` ★ | 数学内核（inv_sqrt/clamp/deg_to_rad、Vec3、Quat：integrate/from_euler/to_euler/rotate） | ✅ 验收 |
| `core/solver/{mahony.hpp,mahony.cpp}` ★ | Mahony：predict/observe/update/reset；成员 cfg_/q_/e_int_/e_last_/is_meas_ | ✅ 验收 |
| `core/measure/measure.hpp` ★ | `IMUSample{acc_, gyro_}`（**注意：全大写 IMU 是用户命名，勿改回**） | ✅ 验收 |
| `core/config.hpp` ★ | `Dimension{d2,d25,d3}` + `make_mahony_config()`（2026-08-30 用户定案弃产品枚举 Scene） | ✅ 验收 |
| `core/estimator.hpp` ★ | 门面 `template<Solver=Mahony>`：predict/observe/observe_heading（**空占位**）/reset/quaternion/euler | ✅ 验收 |
| `tests/unit/test_{math,mahony,estimator}.cpp` ☆ | 三套锚点测试 | ✅ 验收 |
| `host/replay/replay_nav2.cpp` ☆ | 回放工具（读 data/NAV2_data.bin，输出 RMSE 报告 + 可选 CSV） | ✅ 验收 |
| `data/` | NAV2 数据集（文本 12 列：acc3 gyro3 mag3 真值 euler3；**世界系 z-down → 喂入前 acc 取反**） | ✅ 入库 |
| `reference/` | 4 个只读参考库（各自 .git；CLion 已屏蔽见 .idea/vcs.xml；gitignore 排除不入库） | 只读 |
| `docs/` | 00-README / RESEARCH / DESIGN（F1~F13 决策表）/ DEV（开工顺序）/ 复盘 / 待办 / CODE（施工文档） | — |
| `.idea/vcs.xml` | 子仓库 VCS 已置 None（只显示 foucault 的 git log） | 本地配置 |

## 5. 关键决策速查（详表见 DESIGN.md §5）

- **F10** namespace 子分层 `foucault::{math, measure, solver}`；**F11** 决策编号入代码注释
- **F12** 成员统一尾下划线（x_/q0_/e_int_），**无条件**、struct/class 不作区分；参数与局部**不带** `_`；函数/变量 snake_case；bool 用 `is_` 前缀 + 尾下划线（用户风格）
- **F13** 门面 = 模板策略；config 暴露数学抽象 Dimension（产品语义属应用层，映射靠注释）
- **本质公式**：q̂̇ = ½·q̂⊗(ω−b̂) + K·(z−h(q̂))；Mahony = K 折进角速度（PI 控制器，Ki·∫e = 零偏估计）
- 世界系 **z-up**（g_world=(0,0,1)）；欧拉角 = 内旋 ZYX（to_euler 返回 (roll,pitch,yaw) rad）
- 6 轴下 yaw 不可观 → yaw 漂移是预期行为；**观测入口 observe_heading() 已预留（F4）**
- **传感器配置（用户 2026-08-30 确认）**：里程计确定有 / 视觉待定 / 磁力计大概率没有 → 主线 6 轴 + 外部 yaw 注入
- 保留模板的理由：实例化后零开销 / 桌面 double 回放排查数值 / 纯 float 项目先例

## 6. 下一步任务：批次 4a — Mahony 外部 yaw 注入（方案已提交，待用户确认）

> ⚠️ **本节为 2026-08-30 原稿，已过时**。最新版见 **`DESIGN.md §3.7`**（已吸收 P0-4 禁用布尔闩锁、F14 trust 通道等后续结论）。
> 尤其注意：本节 D6 的 `is_yaw_meas_` 布尔标志**已被判定为 P0-4 同类缺陷**，改法见 DESIGN §3.7.5。

**下个 agent 第一步**：向用户确认下面 D1~D7（用户回复"你来/确认"即视为定案），然后：
写 CODE.md 批次 4a（用户抄 3 处）→ AI 写测试/回放 → 回归验收。完整方案：

- **原理**：`ω_corrected = ω + Kp·e_acc + Ki·∫e_acc + Kp_heading·e_heading·v̂`
  （v̂ = 预测重力方向 = 世界 z 在机体系表达，observe 里已有；e_heading = wrap(ψ_ref + offset − ψ_est)）
- **D1** yaw 修正沿 v̂ 方向（绕重力轴转，斜坡不耦合 roll/pitch）——**不要**直接加机体系 z
- **D2** yaw 纯 P 不进积分（里程计打滑会污染零偏估计；Ki 只对 acc 残差）
- **D3** 首次观测自动对齐 `heading_offset_ = ψ_est − ψ_ref`；`reset()` 后重新对齐
- **D4** e_heading wrap 到 (−π, π] 并 clamp ±π/2（防大跳变猛转）
- **D5** `MahonyConfig` 加 `kp_heading_`（起步 5.0f，回放标定）
- **D6** `is_yaw_meas_` 标志：无参考时行为不变（优雅降级）
- **D7** 门面 observe_heading 从空占位改为转发 `solver_.observe_heading(heading_rad, dt)`

改动面：`core/solver/mahony.hpp`（+observe_heading/+e_heading_/heading_offset_/v_/is_yaw_meas_/+kp_heading_）、
`mahony.cpp`（observe_heading 实现 + predict 修正项 + reset 重置）、`core/estimator.hpp`（转发一行）——用户抄；
`tests/unit/test_mahony.cpp`（新锚点：对齐/收敛/wrap 环绕/无参考不变）+ `host/replay/replay_nav2.cpp`
（加 -y 选项：none|gt 理想里程计|gt_noisy|gt_slow 10Hz 间歇）——AI 写。

**验收 4 项**（用 NAV2 真值 yaw 列当理想里程计，无需真实里程计）：
① yaw 漂移 −14.5° → RMSE < 2° ② roll/pitch RMSE 不退化（< 5°，证明无耦合）
③ 10Hz 间歇参考仍收敛 ④ 加噪参考不爆炸。

## 7. 后续批次（原 DEV.md §5 顺序调整后）

| 批次 | 内容 | 前置 |
|---|---|---|
| 4a | Mahony 外部 yaw 注入（本批） | 无 |
| 4b | EKF + 卡方/渐消/量测仲裁 | **用户研究 Solà 论文后**（PDF 在 docs/） |
| 5 | 2D/2.5D 维度模式 + kf_1d | 4b 后 |
| 6 | platforms/arm_cmsis + H7 实测（对照参考库 Mahony 1.33µs / EKF 29.4µs） | — |
| host 辅助 | allan（Allan 方差→Q/R）/ calib / plot / bench | 不急 |

## 8. 常用命令与环境

```bash
cmake --build build && ctest --test-dir build --output-on-failure   # 全量回归
./build/replay_nav2 data/NAV2_data.bin                               # 单跑回放
./build/replay_nav2 data/NAV2_data.bin /tmp/out.csv                  # 导出 CSV 画图
```
- 编译：C++17；`-Wall -Wextra -Werror` 零告警；测试带 ASan/UBSan
- git 全局已配（wilson / liaopanyi2020@gmail.com），唯一分支 main，无远端
- 用户正在准备刷机（H7 板）——回来后可能咨询移植/硬件问题（批次 6 未开始；
  参考库 `reference/CtrBoard-H7_IMU_Altitude` 是 H723 实测工程，含 Mahony 1.33µs / EKF 29.4µs 数据）

## 9. 已踩的坑（全文见复盘.md，开工前必读）

1. **数据集坐标系**：NAV2 数据集世界系 z-down（静止 acc=(0,0,−1)），core 是 z-up →
   host 边界 `acc_adapted = −acc`，core 不动（首跑 roll RMSE 179° 的教训）
2. **文档必须跟随用户实际代码**：用户会改命名（IMUSample、成员尾下划线 kp_/acc_）→ CODE.md 要同步 token 级一致
3. **用户抄录常见笔误**：文件名拼错（estimater）、类名小写（solver::mahony）、成员名新旧混用 →
   编译报错定位后只改错字，保留用户结构与注释
4. **ctest Not Run ≠ 路径问题**：先查是否某 target 编译失败连带（replay 曾因链接漏 mahony.cpp 而 Not Run）
5. **CLion 嵌套仓库**：reference 各库自带 .git 会被注册为 VCS 根 → .idea/vcs.xml 置 vcs="" 屏蔽
