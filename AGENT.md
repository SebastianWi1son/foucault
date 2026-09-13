# AGENT.md — foucault 代理指南

> 依据 `AGENT_TEMPLATE.md` 生成（2026-08-28），按本项目细节填充。
> 核心三条铁律（§2）不可违反。

## 1. 项目定位

- 一句话定位：**foucault = 通用可移植姿态解算（attitude estimation）算法核心** —— 一个内核框架 + 可插拔增益求解器（Mahony/Madgwick/EKF/1D-KF）× 可裁剪维度模式（2D/2.5D/3D）× 可注入量测源（6轴/9轴/外部 yaw），C++17 零依赖，面向云台 / 地面小车 / 立体步兵三类场景。
- 目录结构：
  - `core/` ★ 算法核心（无 STL/异常/RTTI/动态内存）：`math/`（vec3/quat/mat/scalar_ops）、`model/`（状态模型 2D/2.5D/3D）、`measure/`（量测与仲裁）、`solver/`（增益求解器）、`estimator.h`（门面）、`config.h`、`output.h`
  - `host/` 主机工具（桌面，允许 STL）：`replay/`（数据集回放+真值对比）、`allan/`（Allan 方差→Q/R）、`calib/`（校准）、`plot/`
  - `platforms/` 平台适配示例：`arm_cmsis/`（H7）、`desktop/`
  - `tests/` 单元测试 + 数据集回归 + bench
  - `reference/` 三个参考库（只读）：CtrBoard-H7_IMU_Altitude / IMU_Attitude_Estimator(gaochq) / MahonyAHRS(PaulStoffregen)
  - `docs/` 文档体系（见下）
- 权威文档索引：
  - **`docs/DESIGN.md` = 设计权威**（本质公式 / 场景覆盖 / 设计决策 / 概念结构 / 决策点状态表）—— 改设计先改它
  - `docs/RESEARCH.md` = 调研（算法全景 / 开源库 / 参考库分析）—— 查"有什么"用
  - `docs/DEV.md` = 开发流程（体量 / 理论路径 / 工程规范 / 开工顺序）—— 开工必读
  - `docs/复盘.md` = 错误清单（阶段开工/复查前必读）
  - `docs/待办.md` = 悬挂待办清单
  - **`docs/答疑.md` = 概念问答录**（理解过程中的问题与解答；含直觉/推导/代码索引；错答修正痕迹也保留）
  - **`docs/HANDOFF.md` = 交接文档**（2026-08-30 生成：项目状态/铁律/下一步任务速查，agent 接手必读）
  - **`docs/CODE.md` = 代码施工文档**（用户边抄边学的唯一代码来源；批次 = 真实代码或伪代码，按用户选择）
  - **文档与代码不一致 = 缺陷**，须指出并修正文档
- 参考实现：`reference/` 三库只读；迁移清单见 RESEARCH.md §3.6/§4.4

## 2. 角色铁律（三条，不可违反）

1. **代码修改权限**：AI 默认**只读**，除非用户明确指示，否则**不得修改任何项目源码**。
   授权例外（默认允许）：写 `tests/` 测试（**用户指示：测试一律由 AI 编写**，2026-08-30）、`examples/` 示例、`docs/` 文档、生成新文件。
   **可撤销性**：任何覆盖/删除/移动前必须可回退（git 已提交 / 已备份 /tmp），三者皆无 = 禁止执行。
   （注意：项目根目录尚未 `git init`，删除/移动前一律先 `cp` 备份到 /tmp。）
2. **开发流程**：项目讨论确定方案 → 按计划架构，**AI 给伪代码/参考代码/讲解，用户敲具体代码**。
   AI 的角色是指导、讲解、复查、贴参考（参考库行号/论文公式号）；不替用户写实现。
3. **命名一致性检查**：用户实际敲的变量名/函数名/类名与指导文档不一致时，
   AI 须按 §3 命名评估框架**判断新名的合理性**——合理则接受并同步文档，**不合理则及时提醒用户**，
   不得沉默接受、也不得盲目坚持文档。

## 3. 命名评估框架（出现任何新名字时按此评估）

```
① 信息增量：后缀/前缀必须添加新信息，否则删除（反例 xxx_calc，正例 xxx_kinematics）
② 领域术语优先：优先行业/数学标准术语，再考虑自造词
   （姿态域标准术语：quaternion/quat、kinematics、residual、innovation、
    observability、complementary filter、Mahony/Madgwick/EKF/ESKF、AHRS）
③ 一层一个后缀：接口层无后缀，实现层统一 _impl（或同类约定），禁止混搭
④ 中性化：名字不绑定具体单位/实现细节（rpm 等），单位进文档与注释
   （本项目：状态统一叫 quaternion 不叫 angle4；输出统一 euler 不叫 attitude_angle）
⑤ 成员命名（F12）：**统一尾下划线 `_` 后缀，无条件、无例外**（用户定案 2026-08-30；`x_`/`q0_`/`e_int_`）——**struct / class 不作区分**（C++ 中只差默认访问权限）；core / host / platforms 全层一致；**参数与局部变量不带 `_`**（否则约定失去区分意义）
⑥ 函数/变量 snake_case（用户定案 2026-08-30）：`inv_sqrt`/`deg_to_rad`/`from_euler`，禁止 camelCase 混入（用户敲出 camelCase 时提醒改回）
```

## 4. 工程规范

- C++17；core 零依赖（仅标准库头文件），无 STL/异常/RTTI/动态内存；host 允许 STL
- namespace 分层 `foucault::math / model / measure / solver`（F10）；代码注释引用决策编号 F#（F11）
- 声明/定义分离（.hpp/.cpp；数学内核可 header-only 例外）；编译告警全开 `-Wall -Wextra -Werror`；MCU 侧 `-fno-exceptions -fno-rtti -fno-threadsafe-statics`
- 库 / test / example 三个独立 CMake target（`foucault_core` / `foucault_tests` / `foucault_examples`）；构建与测试命令写入 tests/README
- 纯函数不持有状态；有状态组件显式标注；常量/参数聚合进 Config 结构
- 防御：接口层做输入校验（NaN/模长门限/时间倒退），返回错误语义明确（枚举/状态码），不静默吞错

## 5. 测试守则

- **锚点测试测行为，不测实现数值**：断言行为不变式（收敛、有界、四元数单位模长守恒、静止时姿态不漂），不是魔法数
- 退出码 = 结果（0 = 全过）；浮点断言必须用容差，禁止 `==`
- 模板/未实例化代码 = 未真正编译：2D 与 3D 实例都必须用 test 实际调用验证
- 测试覆盖关键路径 + 边界 + 失败路径（NaN 输入、dt=0、量测失效降级，不只 happy path）
- **改完必回归**：任何改动（含用户敲的代码、注释级改动）必须重新编译 + 全量测试回归，验证通过才算完成
- 数据集回归（NAV1/NAV2/BROAD 回放 + 三算法对比 + RMSE 报告）是持续验证手段，非一次性

## 6. 沟通规范

- 全程中文；**先讲原理**（比喻/对照表）再动手
- **决策点前置**：需要用户决断的事项提前列出决策点清单（含推荐与理由），不做事后才问
- **面向用户的展示内容统一置于回答末尾（2026-09-12 用户定案）**：AI 的处理过程（探查、验证、工具调用叙述、探索性推导）与**用户需要看到的内容**（结论、发现的缺陷、待用户决断的事项、改动清单）必须**分区**；后者**集中在回答最后**统一呈现，**不得穿插在处理过程之中**。目的：防止"需要用户处理的问题"被过程叙述淹没
- 总结**只要真相**，不要思辨包装；汇报简洁，文件路径写清楚
- 用户说"等等/停下/不需要"时立即停止，不继续展开
- **外部输入先验证**：评审/教程/网文/其他 agent 意见一律先验证（读代码/实测）再采纳或反驳，逐条给结论，不盲信也不护短

## 7. 文档规范

- **文档 = 已决断内容的归档，不是讨论载体**：日常问答/学习/思路讨论直接在对话中完成，**不因单次答疑而随手建档**；只有结论升级为决策（选型、设计定案、规则确立）时才归档进对应大类文档。**例外（2026-09-11 用户定案）**：允许**系统性概念问答录**（`docs/答疑.md`）作为独立大类文档存在——它归档的是"理解路径"而非"结论"
- `docs/` 按角色分工：**DESIGN.md（设计权威）** / RESEARCH.md（调研） / DEV.md（开发流程） / CODE.md（施工） / **答疑.md（概念问答录）** / 复盘.md（错误清单） / 待办.md（悬挂待办） / HANDOFF.md（交接）
- **内容按大类文档集中更新，不随意新增小文档**；新增主题先判断归属大类，塞进对应文档的合适小节
- **单一真相源（去重规则，2026-09-11）**：**结论进 DESIGN，过程/直觉/推导进答疑**。`答疑.md` 不得复制决策表、接口清单、参数表——只写"怎么走到那个结论的"，结论一律引用 `DESIGN §x` / `待办 P#`。判定标准：**该内容改一次要不要改两处？要 → 重复，删答疑这侧（DESIGN/待办为准）**
- **笔记（Obsidian vault）属于工作手册范围（2026-09-13 用户定案）**：vault 在 `~/Develop/Notes/Projects/Foucault/`，**AI 只读**；发现笔误/笔漏/代码镜像过时可以**指出**，改不改由用户定。
  **AI 生成的图/文档一律另起新文件**（如 `ArchitectureMap_v2.canvas`），**严禁覆盖用户已有笔记**——即使用户要求"完善"，也只提交建议或新建，不动原文件
- 设计决策变动时同步更新 DESIGN.md §5 决策点状态表；**文档与代码不一致 = 缺陷**，须指出
- **声明即承诺**：API/枚举/接口声明必须实现；未实现的声明 = 技术债，实现或删除并标注去向

## 8. 项目具体内容

### 8.1 组件/模块清单

| 组件 | 职责 | 依赖 |
|---|---|---|
| `core/math/` | vec3/quat/mat（定长模板 3×3/6×6/3×6）/scalar_ops | 无 |
| `core/model/` | 状态表示（q+bias / ψ）、运动学积分（2D/2.5D/3D，if constexpr 裁剪） | math |
| `core/measure/` | 加速度计/磁力计/外部 yaw 观测 + 静止检测/干扰检测/仲裁 | math, model |
| `core/solver/` | gain_solver 接口 + mahony/madgwick/ekf/kf_1d | math, model, measure |
| `core/estimator.h` | 门面 `template<typename Scalar, int DoF>` + 状态机（UNINIT→CONVERGING→CONVERGED→DEGRADED） | 以上全部 |
| `core/config.h` | Config 结构 + profile 预设（car_2d / gimbal_6axis / gimbal_9axis / infantry_6axis） | — |
| `core/output.h` | 输出快照（Quat/Euler/DCM/ω̂/biaŝ）+ 品质标志（收敛/相对or绝对航向/innovation） | math |
| `host/` | 回放+真值对比、Allan 方差、校准、可视化 | STL 允许 |
| `platforms/` | arm_cmsis（H7 标量钩子）/ desktop 适配示例 | core |

### 8.2 当前决策点与状态

全部决策见 DESIGN.md §5 决策点状态表（F1~F13，2026-08-30 全部已定）：
- 目标平台：**STM32H7**（带 FPU）+ 桌面验证；语言 C++17；核心边界=纯估算核心；
- 验证策略：NAV1/NAV2/BROAD 数据集 + 三算法回归；异步多速率量测：支持（predict 跟 IMU 速率，observe 事件驱动）；
- 2.5D = 3D 状态 + roll/pitch 量测降权；许可 = MIT（若发布）；品质输出 = 收敛标志 + 相对/绝对航向标志；
- F10 namespace 分层（foucault::math/…）；F11 决策编号贯穿代码注释；F12 成员命名（**统一尾下划线**，2026-08-30 更新）；F13 估算器门面（模板策略，2026-08-30 批次 3）；代码风格 snake_case（用户定案）。

### 8.3 悬挂待办

见 `docs/待办.md`（当前：开工步骤 1 core/math 未开始；根目录待 git init；Solà 论文待读）。

### 8.4 硬件事项（嵌入式适用）

- 目标板：达妙 DM-MC-Board02（STM32H723VGT6，@550MHz，FPU），参考库 CtrBoard-H7_IMU_Altitude 同款板
- 传感器：BMI088（参考库驱动可参考）或 ICM20602（用户小车经验）
- 工具链：Keil AC6（参考库 AC6_Code 工程）或 arm-none-eabi-gcc + CMake（待定）
- 烧录调试：SWD（ST-Link/J-Link），USB-CDC 虚拟串口 + VOFA 上位机（参考库工作流）
- 系统级操作（装工具链等）遵循 SysBuild 规范（sudo 走 zenity 授权、操作留痕到 /home/wilson/SysBuild/操作日志.md）

### 8.5 术语表

| 术语 | 含义（本项目语境） |
|---|---|
| attitude / 姿态 | 载体相对参考系（通常 NED/水平系）的朝向，用四元数表示 |
| AHRS | Attitude and Heading Reference System，航姿参考系统 |
| 姿态解算 | 用 IMU（+磁/外部参考）估计姿态的过程；本项目核心业务 |
| gyro / accel / mag | 陀螺仪 / 加速度计 / 磁力计 |
| yaw/pitch/roll | 偏航/俯仰/横滚（Z/Y/X 轴欧拉角，输出层概念） |
| 四元数 | 姿态表示 q=(q0,q1,q2,q3)，单位模长，无万向锁 |
| 互补滤波 / Mahony / Madgwick | 固定增益观测器家族 |
| EKF / ESKF | 扩展卡尔曼 / 误差状态卡尔曼（协方差自适应增益） |
| 残差 / innovation | z − h(q̂)，量测与预测之差（K 的输入） |
| 可观测性 | 某状态能否被量测确定（yaw 在 6 轴下不可观） |
| 卡方检验 | 残差统计检验，超阈值拒收量测（抗运动加速度污染） |
| 渐消因子 λ | 防零偏协方差过收敛的记忆衰减系数 |
| 显式降维 | 核心主动裁剪状态模型到真实自由度（2D/3D 模式） |
| 2.5D 模式 | 3D 状态 + roll/pitch 量测降权（地形小幅变化场景） |
| profile | 场景配置档位（算法+维度+量测源+参数表） |
| trust | 量测可信度（方差 R 或权重），observe 接口的输入 |

## 9. git 与发布规范

- **commit 由用户主导（用户定案 2026-08-30）**：AI 默认**不主动执行 git commit**；用户明确要求时才做。message 不用 AI 自造术语，用**用户看得懂的中文要点**（改了什么、为什么）；本次里程碑提交由用户授权 AI 执行
- 任何 git 操作（checkout/merge/reset/commit）前：先查 `git status`，工作区必须干净
- 未提交/untracked 文件先 `cp` 备份到 /tmp 并告知路径
- commit 前核对暂存列表：message 写到的文件必须真实在列（曾发生漏勾导致 message 与内容不符）
- 日常流程：dev 分支开发 → 阶段完成回 main 快进合并；分支 = 贴纸，历史保持单线
- 文件移动用 `git mv`（保留历史追踪）
- 独立开源库发布：LICENSE（MIT）+ README + CI 三件套齐备；头文件用前缀目录防撞名（`foc/estimator.hpp`）
- ✅ 已 git init（2026-08-30，批次 1-3 里程碑提交）；.gitignore 排除 build/、.idea/、reference/、课程工程
