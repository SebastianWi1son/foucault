---
class: log
generated: false
---
# REVIEW.md — 审阅验证记录

> **类：A 日志** —— 只增不改；每条带日期；不声称"当前状态"。
> 待办 / 进度见 [`../待办.md`](../待办.md)；设计权威见 [`../DESIGN.md`](../DESIGN.md)。

外部与内部审阅提的负面意见在这里**逐条对照实测**，给出判定与证据。
**新的审阅往下追加，不修改旧条目**（旧条目说法与新条目不同不是矛盾，是理解演化）。

---

## 2026-09-13 · 第三方 agent 全仓审阅 — 逐条实测验证

> 来源：用户让另一个 agent 做了全仓审阅，粘贴了其中的负面部分（正面部分未粘贴）。
> 本条由 AI **只读验证**（项目处于冻结，源码一行未改），逐条给判定 + 实测证据。
> 判定图例：✅ 真实 ｜ ⚠️ 半真（事实成立但措辞需修正）｜ ❌ 不成立 ｜ 🕓 现在无法验证

### 验证结果总表

#### A. "宣称"与"实现"的落差

| 编号 | 审阅声称 | 判定 | 实测证据 |
|---|---|---|---|
| **A1** | 可插拔求解器是假的，`Estimator<EKF>` 编不过 | ✅ **真实** | 实测 `Estimator<ekf::EKF>`（EKF 构造参数为 `EKFConfig`）→ **编译失败**：`no matching function for call to 'ekf::EKF::EKF(foucault::solver::MahonyConfig)'`。根因两条：① `estimator.hpp:15` 写死 `solver_(make_mahony_config(dim))` —— 求解器的**构造参数类型**被钉死为 `MahonyConfig`；② `config.hpp:3` **反向** `#include "solver/mahony.hpp"` —— config（上层）依赖 solver（下层），依赖方向颠倒 |
| **A2** | 维度模式是占位 | ✅ **真实** | `config.hpp:16` 就是 `(void)dim;`。实测（identity 起步、acc=(0.1,0.2,0.98)、gyro=(1,2,3)、200 周期）：d2 / d25 / d3 的四元数**逐位相同** = `(-0.922424376, -0.182870314, -0.076640442, -0.331374913)` ×3 |
| **A3** | 量测源只接得进 acc + heading | ✅ **真实** | `mahony.cpp:9` `k_gravity_world(0,0,1)` 硬编码；**无** `observe_vector(vec, ref)` 一类通用向量入口 → 磁力计无处可接 |
| **A4** | EKF / Madgwick / 1D-KF 不存在 | ✅ **真实** | `core/solver/` 只有 `mahony.hpp` / `mahony.cpp` |
| **A5** | 2D / 2.5D 不存在 | ✅ **真实** | 同上（枚举有值、行为为零）|
| **A6** | README 首段承诺与实现落差大、应写明"当前仅 Mahony + 3D" | ⚠️ **半真（结论对、措辞偏重）** | README 第 3 行确以宣称语气列出"4 求解器 × 3 维度 × 3 量测源" ✓ **但**同一文件 10 行后即有"当前状态"表 + "当前唯一实现的求解器是 Mahony（6 轴 acc+gyro）" + "yaw 不可观测，漂移是预期行为" —— **不是"藏在 900 行文档深处"**。改法本身成立（见 N1）|

#### B. 审阅独立发现的问题

| 编号 | 审阅声称 | 判定 | 实测证据 |
|---|---|---|---|
| **B1** | `test_estimator.cpp` 第 4 条是恒真断言（从单位元 / acc=(0,0,1) / 零陀螺起步，任何维度 `euler().norm()` 都 ≈ 0），没有判别力 | ✅ **真实**（"恒真"措辞需下调） | **实验①**：把三个 `Estimator` 的维度参数**全部改成 d3**（`g(d3), c(d3), i(d3)`）→ 测试**照样 ALL PASS** ⇒ **该断言完全无法检测"Dimension 被忽略"** ✓ **实验②**：把小节结果改成 `(9,9,9)` → FAIL ⇒ 它不是**字面**恒真（`norm()<1e-3` 确实是有效范围检查），而是**与维度无关的弱冒烟测试**。三条断言各自独立、互不比较，也不对照任何维度语义 |
| **B2** | CI 锁定不了"逐位回归"——ctest 里的 replay 只断言 RMSE < 5° | ✅ **真实** | `replay_nav2.cpp:231` 唯一断言 = `rms_r < 5.0 && rms_p < 5.0`；`CMakeLists.txt:57` 的 `add_test(replay_nav2 ...)` 无 `-y`、无 golden。⇒ 0.1° 级数值漂移（调参/重构/优化档变化）CI 全绿放行，"五项回放逐位相同"**靠人肉读打印数字**，不是门禁 |
| **B3①** | `AGENT.md §2` 写"项目根目录尚未 `git init`"，§9 写"✅ 已 git init（2026-08-30）"，**自相矛盾** | ✅ **真实** | `AGENT.md:33` = "（注意：项目根目录尚未 `git init`，删除/移动前一律先 `cp` 备份到 /tmp。）" vs `AGENT.md:163` = "✅ 已 git init（2026-08-30，批次 1-3 里程碑提交）"。另 `AGENT.md:123` 又写"根目录待 git init" |
| **B3②** | `AGENT.md §4` 要求"构建与测试命令写入 tests/README"，仓库里没有这个文件 | ✅ **真实** | `AGENT.md:59` 有此要求；`ls tests/` 只有 `tools/` 与 `unit/`，**无 README** |
| **B3③** | `AGENT.md §8.2` 说"F1~F13 全部已定"，但 F14 是"⏳ 提案" | ⚠️ **半真** | 严格说不构成逻辑矛盾（"F1~F13" 的字面范围不含 F14）。真问题是**范围声明过时**：`DESIGN.md §5` 决策表已延伸到 **F14**，且 F14 明确标 **⏳ 提案（2026-09-11，待用户确认）** ⇒ "全部已定"会误导读者 |
| **B4** | 文档用行号锚定代码，已经腐化（`DESIGN §3.6.5` 引 `mahony.cpp:25`/`:51`，`quat.hpp:29` 也漂了） | ✅ **真实，且比审阅说的更严重** | 全仓扫描出 **38 处** `file.ext:数字` 锚定。抽验（打印被引用的那一行）：<br>· `DESIGN.md:323` 说 `mahony.cpp:25` 是"**门禁**" → 实际该行是 `acc_age_ = k_never_measured_;`（**在 `reset()` 里**）；真正的门禁在 `mahony.cpp:42`（dt）与 `:59/:61`（acc）<br>· `DESIGN.md:323` 说 `mahony.cpp:51` 是"**闩锁置位**" → 实际是 `const math::Vec3f omega = gyro_ok`<br>· `DESIGN.md:441` 引 `mahony.cpp:56-59` 为"时序证据" → 实际是 `predict()` 的尾部三行 + `}`<br>· `quat.hpp:29`（`DESIGN.md:351`）→ 实际该行是零向量分支 `else if (n2 == T(0))`；**真守卫 `if (n2 > T(0))` 在 26 行**<br>· 更极端的：`mahony.cpp:38` 被多处引用，而该行现在只是一个 `}`<br>（另有仍正确的：`config.hpp:3/16/18`、`estimator.hpp:15/17`、`mahony.cpp:9/43`）|
| **B5** | `inv_sqrt` 位魔法在 Cortex-M7 上很可能是负优化；P1-2 的取舍前提缺目标硬件支撑 | 🕓 **现在无法验证（质疑成立，需上车实测）** | 审阅的技术论证可信：M7 有单精度 FPU + `VSQRT.F32`，位魔法要 reinterpret + 移位 + 两轮牛顿迭代（每轮 4 次乘加），量级相当、可能更慢。**项目的 P1-2 结论确实是在"没有目标硬件"的前提下做的** ⇒ 应降级为"待实测复核"而非"已定案"（见 N12）|
| **B6** | `correction_heading` 用 `to_euler` 多算且脆弱（为取 psi 顺带算 roll/pitch） | ✅ **真实**（但"改直算能治 gimbal lock"是误解） | `correction_heading` 里 `q_.to_euler(roll, pitch, psi); // 只取psi` —— roll/pitch 算了就丢 ✓ 直算 `psi = atan2(2(q0q3+q1q2), 1−2(q2²+q3²))` 可省**一次 asin**、且 yaw 公式与 pitch 无关 ✓ **但**审阅自己也指出 pitch→±90° 时"yaw 定义本身退化"——那是**旋转本身的奇异（gimbal lock）**，换公式**治不了**，只是省算力/少一次发散路径 ⇒ 收益要按"省一次超越函数"计，不能按"修 bug"计 |
| **B7** | 文档量是代码的 2.2 倍，单一真相源在退化 | ✅ **真实** | 实测 文档 **4912** 行 / 代码 **2129** 行 = **2.31×**。且 `DESIGN.md` 确在重复 `待办.md`/`CODE.md`/`HANDOFF.md` 的批次状态（如 §3.9 路线图）；`待办.md` 里"## 已关闭（近期）"标题**出现两次**（66 行 / 71 行）；`README.md` 状态表仍写"数学内核 ✅ **21 PASS**"（实际 **22** ）|

#### C. 工程细节

| 编号 | 审阅声称 | 判定 | 实测证据 |
|---|---|---|---|
| **C1** | sanitizer 选项在 CMakeLists 重复 4 次；replay_nav2 没有 sanitizer | ✅ **真实** | `-fsanitize=address,undefined` 在 `CMakeLists.txt` 出现 **8 次**（4 个测试 target × compile+link，行 24/25/35/36/42/43/49/50）。`replay_nav2`（行 56）只有 `-Wall -Wextra -Werror`，**无 sanitizer** —— 而它恰是唯一跑真实数据全路径的 ✓ |
| **C2** | CI 只有单一 Ubuntu/Debug/gcc；无 -O2、无 clang、无严格档 | ✅ **真实** | `.github/workflows/ci.yml` 全文 23 行：单 job、`runs-on: ubuntu-latest`、无 `matrix`、`-DCMAKE_BUILD_TYPE=Debug`、无 `-O2`、无 clang、**无项目自称的严格档**（`-Wconversion -Wshadow -pedantic`）。⇒ UBSan 在 `-O0` 可能漏掉"仅优化档暴露"的 UB |
| **C3** | 门面只转发 `rejected_count()`，未转发 `is_acc_valid()`/`is_heading_valid()` | ✅ **真实** | `estimator.hpp:27` 只转发 `rejected_count()`（P2-* 已记，本条与其合并）|
| **C4** | `predict(IMUSample,dt)` / `observe(IMUSample)` 各自只用一半字段，载荷耦合是 API 坏味道 | ⚠️ **半真（事实对、优先级应下调）** | 事实：`estimator.hpp:17` 只用 `s.gyro_`、`:18` 只用 `s.acc_` ✓ **但**调用方每帧手里本就只有一个 `IMUSample`，整传是**便利**不是错误。真正的问题是**命名**：`observe(s)` 读起来像"观察这个样本"，实际只吃掉 acc ⇒ 建议改名为 `observe_accel(s.acc_)` 一类（低优先，冻结期不改）|
| **C5** | `examples/` 是空目录，`example_ahrs` 被注释掉，而 `AGENT.md §4` 要求三个 target | ✅ **真实** | `examples/` 存在但**完全空**；`CMakeLists.txt:61-62` 的 `add_executable(example_ahrs ...)` 两行均被注释；`AGENT.md:59` 明确要求 `foucault_core` / `foucault_tests` / `foucault_examples` 三个 target |
| **C6** | 无性能基准（bench）；引用的 1.3µs 是参考库数字 | ✅ **真实** | 全仓无 `*bench*` 文件；对"1kHz 每帧周期数"这一核心卖点，**项目自己没有测量** |

**汇总（逐项计 21 条）：✅ 真实 17 条 ｜ ⚠️ 半真 3 条（**A6** 措辞偏重 / **B3③** 非严格矛盾 / **C4** 优先级偏低）｜ 🕓 待实测 1 条（**B5**）｜ ❌ 不成立 0 条。**

> 分档计数：**A** 6 条（5 ✅ + A6 ⚠️）｜ **B** 9 条（7 ✅ + B3③ ⚠️ + B5 🕓；B3① B3② 各自计 ✅）｜ **C** 6 条（5 ✅ + C4 ⚠️）。**B1/B4/B6/B7 计 ✅**（B1 的"恒真"措辞在证据栏已下调为"与维度无关"）。
> 整体评价：这份外部审阅**命中率极高**，且 B1/B2/B5/B6 四条是项目自查清单里**没有**的新发现，含金量高。
> 唯一需要修正的是措辞分寸（A6 把"README 坦白位置"说重了；B1 的"恒真"应说成"与维度无关"）。
