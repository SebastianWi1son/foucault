---
class: log
generated: false
---
> **类：A 日志（append-only）** —— 冻结/交接快照，**带日期，不代表当前状态**。
> 当前状态见 [`../STATUS.md`](../STATUS.md)；进度见 [`../待办.md`](../待办.md)。

# HANDOFF.md — 交接文档（2026-09-13 冻结时更新）

> 用途：项目在「批次 4a-5 完成」后**冻结**，等硬件就位复工。本文件供下一个 agent / 未来的自己零上下文快速接手。
> **开工前必读：本文件 + AGENT.md**；冲突时以 AGENT.md 与 docs/ 文档体系为准。

---

## 1. 项目一句话

**foucault = 通用可移植姿态解算算法核心**（C++17、零依赖、core 禁 STL/异常/RTTI/动态内存）：
可插拔增益求解器（Mahony 现役 → EKF 后续）× 可裁剪维度（2D/2.5D/3D，当前仅 3D）× 可注入量测源
（6 轴 acc+gyro 现役 / 外部航向已落地 / 9 轴磁力计出局），目标 **STM32H743XBH6**，
场景：云台 / 地面小车 / 立体步兵。**当前主线 = 2.5D 四麦轮小车 + 定时器增量式编码器里程计**。

**硬件（2026-09-13 用户确认）**：MCU **STM32H743XBH6**（480MHz Cortex-M7 + 双精度 FPU + D-cache）；
IMU **ICM-20602**（SPI，±2g / ±250~2000dps）；**4 路增量式编码器走定时器**；调试 **串口**。
> ⚠️ 硬件尚未就绪 —— 这正是本次冻结的原因。

## 2. 用户（wilson）工作模式（铁律，违反 = 返工）

1. **全程中文**；先讲原理（最土的话 + 类比，见 `答疑.md` 的三行式：物理 → 数学 → 代码）；
   **决策点前置**（列表 + 推荐）→ 用户确认 → 落文档
2. **AI 默认只读源码**：用户抄录 `docs/CODE.md` 代码进源文件
3. **测试归 AI**（2026-09-13 用户定案）：`tests/`、`host/replay/`、CMake 测试配置由 AI 直接改仓库并验证
4. **`CODE.md` 是唯一施工来源**；**代码块 = 改动定位图，不是粘贴源**（2026-09-13 定案）：
   旧注释剥掉、只在改动处留 `★批次号` 标记 → 用户**逐段替换**，不整文件覆盖
5. **文档 = 归档非讨论载体**；**决策进 DESIGN，行动进待办**；**文档与代码不一致 = 缺陷**
6. 覆盖/删除前必须可回退（git 已提交 / `/tmp` 备份），三者皆无 = 禁止
7. **commit 由用户主导**：AI 默认不 commit；message 用中文且用户看得懂
8. 笔记（Obsidian vault `~/Develop/Notes/Projects/Foucault/`）**属工作手册范围，AI 只读**；
   AI 生成的图/文档**一律另起新文件**，严禁覆盖用户笔记
9. 未完成批次不算完成：改完必回归**三档编译 + 全量 ctest + 回放金标**

## 3. 当前状态（2026-09-13 冻结，验收全绿）

- **分支** main；**已提交** `8ae5534`（4a-4 微调）→ 冻结提交见 §7
- **三档编译全零告警**：默认 `-Wall -Wextra -Werror` / 严格 `-Wconversion -Wshadow -pedantic -fno-exceptions -fno-rtti` / ASan+UBSan
- **ctest 5/5**：`test_math` 22 / `test_math_audit` **125** / `test_mahony` **24 条断言（23 锚点 + 17a 前置）** / `test_estimator` 6
- **回放金标**（NAV2 数据集 320s@50Hz，6 轴 acc+gyro）：
  ```
  基线    : roll RMSE 2.152° MAX 12.167° / pitch 2.645° MAX 13.510° / yaw 漂移 −14.450°
  -y gt   : yaw RMSE 0.085° MAX 0.701°      漂移 −0.022°
  -y gt_slow:     0.095°      0.998°              −0.024°
  -y gt_noisy:    1.266°      2.051°              −1.121°
  -y gt_drop:     0.385°      2.392°              −0.022°
  ```
- **故障注入验收**（`--fault`，每 100 帧一帧脏数据，共 159 帧）：
  `acc_nan` / `acc_zero` / `acc_sat` / `gyro_nan` → **全部拦下 159/159**，RMSE 均为基线 2.152°
  （拆掉守门人时 `acc_nan` / `gyro_nan` 的 RMSE = **nan**，永久损坏）

## 4. 代码地图（★ = 用户抄录，AI 只读需授权；☆ = AI 写）

| 文件 | 内容 | 状态 |
|---|---|---|
| `core/math/{scalar_ops,vec3,quat}.hpp` ★ | 数学内核（`inv_sqrt`/`clamp`/**`wrap_pi`**、Vec3、Quat） | ✅ **已冻结**（守门测试 `test_math_audit`）|
| `core/math/{mat3,mat6}.hpp` | EKF 前置（四元数→旋转矩阵） | ⬜ **不存在**（批次 4 EKF 时新增，不触动冻结文件）|
| `core/solver/{mahony.hpp,mahony.cpp}` ★ | Mahony：`predict`/`observe`/`observe_heading`/`align_heading`/`update`/`reset`/品质查询 | ✅ 验收至 4a-5 |
| `core/measure/measure.hpp` ★ | `IMUSample{acc_, gyro_}`（**全大写 IMU 是用户命名，勿改**） | ✅ 验收 |
| `core/config.hpp` ★ | `Dimension{d2,d25,d3}` + `make_mahony_config()` | ⚠️ 三档参数仍是同一组（降维未实现）|
| `core/estimator.hpp` ★ | 门面 `template<Solver=Mahony>` | ⚠️ 可插拔是假的（P0-1：config 反向依赖 solver）|
| `tests/unit/*.cpp` ☆ | 四套测试（math / math_audit / mahony / estimator） | ✅ 验收 |
| `host/replay/replay_nav2.cpp` ☆ | 回放工具（`-y none\|gt\|gt_slow\|gt_noisy\|gt_drop` + `--fault`） | ✅ 验收 |
| `data/` | NAV2 数据集（文本 12 列；**世界系 z-down → 喂入前 acc 取反**） | ✅ 入库 |
| `reference/` | 4 个只读参考库（各自 .git，不入库） | 只读 |
| `docs/` | DESIGN（权威）/ CODE（施工）/ 答疑 / DEV / HANDOFF / 待办 | — |

## 5. 关键决策速查（详表见 DESIGN.md §5）

**架构**
- 门面 = **模板策略**（F13），无虚函数；namespace 子分层 `foucault::{math, measure, solver}`（F10）
- **保留 `predict` 命名**（F14）：Mahony 把修正折进 ω，**没有独立的 `correct()`**（observe ≠ correct）
- **可插拔抽象线画在"数据进/结果出"**：core 门面入口**专用**（编译期类型安全、零开销），
  **可信度语义统一**（每入口都有 `trust`）；外部仲裁器才用统一载荷 `Measurement`（在 core 之上）
- **按数学形式切分**量测（①向量观测 ②标量航向 ③位姿/速度），**不按传感器种类**

**Mahony 管线（批次 4a 系列成果）**
```
predict(gyro, dt):
    v̂ = q̂*·(0,0,1)                          ← 统一消费口（每周期只算一次）
    ω = gyro + correction_acc(v̂,dt) + correction_heading(v̂)   ← 统一出口（不在线 = 加 0）
    age += dt; q̂.integrate(ω, dt)
```
- **残差必须每周期现算，不得冻结**（4a-2）→ 否则等效增益随量测速率漂移
- **门控失败 → 该项返回 0（加法幺元）**：从控制流分支降级为一次取值（4a-3）
- **`v̂` 不提升为成员**：纯状态投影，与量测无关 → 两通道实现级解耦
- **对齐 = 初始化，不是收集**（4a-4）：`align_heading()` 公开（只动航向零点，不动 roll/pitch）；
  `observe_heading()` 首次自动兜底。`align_heading` 里 `heading_ref_ = heading_ref` **不可删**
- **入口守门人**（4a-5）：脏帧 = "量测不作数" → 丢弃（不清年龄）→ 年龄门控自然接管（**零新状态**）
- **命名分层**：`yaw` = 那个角（数学量/真值/误差）；`heading_*` = 处理那个角的通道/参考

**数值约定**
- 世界系 **z-up**，`g_world = (0,0,1)`；欧拉角 = 内旋 ZYX（`to_euler` 返回 (roll,pitch,yaw) rad）
- `q.rotate()` = **body → world**（实验 A/B/C 三重验证）
- `Quat::normalize` 用位魔法 `inv_sqrt`（~4e-6）；`Vec3::normalize` 用精确 `1/sqrt`（~6e-8）——**有意的性能取舍**
- `Quat::normalize` 只在 `n2 == 0` 时回退 identity；**NaN/Inf 原样传播**（脏值必须可见）
- **`observe(acc)` 的输入单位 = g**（`acc_min_`/`acc_max_` 也是 g）；`acc_max_ = 2.0f` 对应 ±2g 量程
- `MahonyConfig` 默认：`kp_ 5.0 / ki_ 0.3 / integral_limit_ 10.0 / acc_timeout_ 0.1 /
  acc_min_ 0.5 / acc_max_ 2.0 / kp_heading_ 5.0 / heading_timeout_ 0.3`
- 6 轴下 yaw 不可观 → 漂移是**预期行为**（唯一 yaw 基准是里程计）

**测试约定**
- **判别测试必须断言"被改动的行为"**，而不是"调用某函数后某状态没变"——后者可能是**恒真断言**
  （教训：`align_heading` 第 15 条初版；见 CODE.md 批次 4a-4）
- 每条新测试都要验证**判别力**（把目标改动拆掉 → 必须 FAIL）

## 6. 下一步任务（复工清单）

**★ 分水岭：批次 4a′ 上车实测** —— 在拿到真实麦轮数据之前，任何"仲裁要多复杂"的判断都是猜测
（`DESIGN §3.8.1` 已证明"模长门控"这种猜测几乎无效）。

**硬件就位后的顺序**：

1. **平台层移植**（`platforms/arm_cmsis` + CMake 交叉编译）—— 全量构建
   - ⚠️ H743 有 **D-cache**：SPI-DMA 写入可缓存内存必须做 cache maintenance 或配 non-cacheable MPU 区
   - ⚠️ 串口波特率：1kHz IMU + 4 路编码器全量落盘，115200 远远不够
2. **数据记录通路**（★ 4a′ 的唯一前提）：能 dump 成文件的
   `acc(3) + gyro(3) + 时间戳` 与 `4×轮计数 + 时间戳` → 塞进现有 replay 流程离线分析
3. **真值方案**：实车没有 gt 列。需定：闭环法（跑回起点看漂移）/ 场地标记 / 只做相对一致性
4. **跑代表性动作**：直行 / 原地转 / 急停 / 上坡 / **故意打滑**（打滑数据是 4b 仲裁的唯一输入）
5. **批次 4b 量测仲裁**（数据驱动，**不提前做**）：打滑检测（`ω_odom` vs `ω_gyro` 主判据 +
   4 轮自洽性超定冗余）、残差门控、静止检测、卡方、时变 `trust`
6. **批次 5**：2.5D 维度模式 + EKF（建议合并；`mat3.hpp`/`mat6.hpp` 前置）

**沿途可顺手清的债**（`docs/待办.md`）：
P0-1 门面可插拔是假的 / P0-2 量测模型硬编码重力 / P0-3 门面签名与 DESIGN 不一致 /
P2-2 缺 `-fno-exceptions -fno-rtti` + replay 无 sanitizer / P2-3 文档结构树把"计划"当"已存在" 等

## 7. 冻结说明（2026-09-13）

**冻结原因**：批次 4a-5（入口守门人）完成 —— 至此 **4a 系列全部落地**，
「小车能上车的最小闭环」在**纯软件层面已走完**，剩下的都依赖硬件（`DESIGN §3.9`）。

**冻结基线**：本次提交 + 全绿验收（§3）。
**复工触发器**：H743 板 + 麦轮小车 + ICM-20602 + 4 路编码器 + 串口通路就绪。

**复工第一步**：读本文件 → 读 `AGENT.md` → 跑一遍 §3 的全量验收（确认基线未漂移）→ 从 §6 第 1 步开始。

> ⚠️ **`/tmp` 会被清理**：构建目录、`/tmp/fgd*` 等验证副本都不保证存在，复工时重新 `cmake -S . -B build`。
> ⚠️ **本次冻结期间不得改 `core/math/`**（已冻结），除非走"解冻一次"流程并同步补守门测试。
