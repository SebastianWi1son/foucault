---
class: log
generated: false
---
# trash/ — 待裁决区

> **这里不是"已删除"。** 这里放的是**判断为"非文档"或"过期/矛盾"的内容**，
> 原信息**一字未删**，等解冻后由 agent 逐条判断，再决定整目录删除。
> 建立时间：2026-09-13（第一次文档卫生整理，用户授权，源码一行未改）。

---

## 1. 里面的东西是什么

| 文件 | 它到底是什么 | 为什么在这里 | 解冻后怎么判 |
|---|---|---|---|
| `CODE.md` | **施工单**，不是文档。2297 行，逐批把代码又抄了一遍（代码本体 2225 行） | 代码是唯一源，施工单验收后即失效 | ① 直接删（历史在 git 里）；或② 压缩成"每批 3~5 行改动记录"存进 `docs/log/` |
| `AGENTS.md.original` | **压缩前的完整 AGENTS.md**（163 行） | 压缩成 ≤ 一屏的新版，原文保底留档 | 确认新版没有漏掉规则 → 删 |

---

## 2. 本次整理做了什么（全部可 `git mv` 回退）

| 动作 | 详情 |
|---|---|
| 建 `docs/log/` | A 类日志集中地 |
| 移动 | `docs/HANDOFF.md` → `docs/log/HANDOFF.md` |
| 移动 | `docs/RESEARCH.md` → `docs/log/RESEARCH.md` |
| 移动 | `docs/答疑.md` → `docs/log/答疑.md` |
| 移动 | `docs/复盘.md` → `docs/log/复盘.md` |
| 移动 | `docs/CODE.md` → `trash/CODE.md` |
| 新建 | `docs/README.md`（文档体系说明 = 模板） |
| 新建 | `docs/STATUS.md`（当前状态唯一来源） |
| 新建 | `trash/README.md`（本文件） |
| 压缩 | `AGENTS.md`（163 → ≤ 一屏），原文存 `trash/AGENTS.md.original` |
| 加头 | 7 个文档加上 `class:` front-matter + 一句话类规则（**只加，未删**） |

**没有删除任何信息，没有修改任何源码。**

---

## 3. 解冻后待裁决清单（矛盾 / 过期 / 结构问题）

按优先级排。每条都附了证据，方便直接判定。

### P1 — 明确的矛盾（已实证）

| # | 问题 | 证据 | 建议 |
|---|---|---|---|
| 1 | `README.md` 首段宣称"可插拔（Mahony/Madgwick/EKF/1D-KF）× 可裁剪维度（2D/2.5D/3D）× 可注入量测源（6/9 轴）"，三项全为占位 | `estimator.hpp` 写死 config；`config.hpp` 有 `(void)dim`；重力硬编码 | 首段改成"当前实现：Mahony + 3D + 6 轴"，其余标"路线图" |
| 2 | `README.md` 的"当前状态表"与 `STATUS.md` / `待办.md` 三处重复 | 三处都列批次进度 | 删表格，改链接到 `STATUS.md` |
| 3 | `DESIGN.md` 有两个同编号小节 `### 3.8.5`（"落地节奏" 与 "入口守门人"） | 全文搜索 `3.8.5` 命中两次 | 重编号 |
| 4 | `DESIGN.md` 通篇用 `mahony.cpp:25` / `quat.hpp:29` 一类行号，已全部错位 | 行 25 现在是 `reset()` 里的赋值；真正门禁在第 42 行 | 全文替换为符号名 |
| 5 | `DESIGN.md` §4.3 把**计划**当**已存在**：`core/model/`、`core/output.h`、`platforms/`、`host/allan|calib|plot` 均不存在 | `git ls-files` 无这些路径 | 每项标注 ✅已存在 / ⬜计划中（批次 N） |
| 6 | `DESIGN.md` §4.4 代码块用 camelCase（`observeAccel`）且写 `virtual GainSolver`，与 F13 定案（模板策略、snake_case）冲突 | 与 §5 决策表 F13 对不上 | 整体重写或标注"早期草案" |
| 7 | `AGENTS.md.original` §2 说"根目录尚未 git init"，§9 说"已 git init（2026-08-30）" | 同文件自相矛盾 | 新版已删（原文留档） |
| 8 | `AGENTS.md.original` §4 要求"构建与测试命令写入 `tests/README`"，该文件不存在 | `git ls-files` 无 `tests/README*` | 补建或删要求 |
| 9 | `AGENTS.md.original` §8.2 称"F1~F13 全部已定"，§8.3 称"开工步骤 1 core/math 未开始" | F14 是提案；core/math 早已冻结 | 新版已删（原文留档） |

### P2 — 内容重复（同一条事实多个副本）

| # | 重复的事实 | 出现在 | 唯一归属应为 |
|---|---|---|---|
| 10 | 批次进度（4a-1 ~ 4a-5 已完成） | `DESIGN.md` §3.9、`HANDOFF.md` §3、`待办.md`、`README.md` 状态表 | `待办.md` |
| 11 | 验收数字（2.152 / 2.645 / 0.085 / 0.159 拦截） | `HANDOFF.md` §3、`待办.md`、`DESIGN.md` §3.7.7、`README.md` | `STATUS.md` |
| 12 | P0-1 ~ P0-5 缺陷描述 | `DESIGN.md` §3.6.4、`待办.md`"审阅归档" | `待办.md`（DESIGN 只留"为什么这么设计"） |
| 13 | 冻结说明 / 复工清单 | `HANDOFF.md` §6/§7、`DESIGN.md` §3.9 路线图、`待办.md` | `待办.md`（HANDOFF 作为历史快照不改） |

### P3 — 结构 / 命名

| # | 问题 | 建议 |
|---|---|---|
| 14 | 命名语言不统一：`答疑.md` / `复盘.md`（中文）vs `RESEARCH.md` / `HANDOFF.md`（英文） | 统一为一种（建议中文）；**注意会打断现有引用** |
| 15 | 根 `README.md` 与 `docs/README.md` 同名，含义不同 | 可接受，但根 README 顶部加一行指向 `docs/README.md` |
| 16 | `docs/Quaternion kinematics ... .pdf`（23987 行）是外部论文，与本仓文档同级 | 移到 `docs/ref/` |
| 17 | `examples/` 空目录，但 `CMakeLists.txt` 注释与 `AGENTS.md` 都提到它 | 补最小示例，或删目录 + 清注释 |

### P4 — 待建机制（不是矛盾，是缺口）

| # | 缺口 | 对应本仓规则 |
|---|---|---|
| 18 | 缺 `scripts/gen_status.sh` + "重跑无 diff" CI 门禁 | C 类"状态不许手写" |
| 19 | 缺"文档路径 / 符号存在性"CI 检查 | 防 P1-4 / P1-5 类问题复发 |
| 20 | 三个 skill 未建：批次验收 / 文档对账 / 尺子审计 | `docs/README.md` §7 |
| 21 | CI 只锁 `RMSE < 5°`，不锁基线具体数字 | "逐位相同"目前靠人工 |

---

## 4. 删除本目录的前置条件

全部满足后，`trash/` 可整目录删除：

- [ ] §1 两个文件已判定（删 / 压缩后并入 `docs/log/`）
- [ ] §3 的 P1 九条全部关闭
- [ ] §3 的 P2 十三条重复已收敛到唯一归属
- [ ] §3 的 P4 四个机制已建（至少 18、19）
