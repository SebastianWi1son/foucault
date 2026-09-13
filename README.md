# foucault

通用可移植**姿态解算（attitude estimation）算法核心** —— 一个内核框架 + 可插拔增益求解器（Mahony / Madgwick / EKF / 1D-KF）× 可裁剪维度模式（2D / 2.5D / 3D）× 可注入量测源（6 轴 / 9 轴 / 外部 yaw）。

面向云台、地面小车、立体步兵三类场景。C++17，core **零依赖**（仅标准库头文件）。

> 名字取自 Léon Foucault —— 傅科摆首次在实验室尺度上证明了地球自转，即"用局部量测（摆的振动面）反推全局姿态"的那类问题。

## 当前状态

| 批次 | 内容 | 状态 |
|---|---|---|
| 1 | 数学内核（`vec3` / `quat` / `scalar_ops`） | ✅ 21 PASS |
| 2 | Mahony 求解器（异步多速率 predict / observe） | ✅ 通过 |
| 3 | Estimator 门面 + NAV2 数据集回放（RMSE 对照真值） | ✅ 通过 |
| 4 | EKF（卡方检验 / 渐消因子 / 限幅）+ Allan 方差定 Q/R | ⏳ 计划中 |
| 5 | 维度模式 2D / 2.5D | ⏳ 计划中 |

当前唯一实现的求解器是 Mahony（6 轴 acc + gyro）。**yaw 在 6 轴下不可观测，漂移是预期行为，不是缺陷。**

## 快速开始

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
cd build && ctest --output-on-failure     # 或：ctest --test-dir build --output-on-failure
```

要求：CMake ≥ 3.16、支持 C++17 的编译器。测试目标自带 `-fsanitize=address,undefined`。

单独跑数据集回放（可选导出 CSV 供画图）：

```bash
./build/replay_nav2 data/NAV2_data.bin /tmp/out.csv
```

## 目录结构

```
core/          算法核心（无 STL / 异常 / RTTI / 动态内存）
  math/          vec3 / quat / scalar_ops（header-only）
  measure/       量测输入（IMUSample）
  solver/        Mahony（后续：Madgwick / EKF / 1D-KF）
  estimator.hpp  Estimator 门面（模板策略，一行切换求解器）
  config.hpp     配置聚合 + 维度档位
host/          主机工具（桌面，允许 STL）
  replay/        数据集回放 + 真值对比
tests/unit/    锚点测试（测行为不变式，不测实现数值）
data/          数据集（文本 12 列，50Hz）
docs/          文档体系（见下）
reference/     三个参考库（只读，不入 git）
```

## 代码风格

本项目采用明确、可机械检查的风格，**无条件应用，不按类型或层级开例外**：

| 规则 | 说明 |
|---|---|
| `snake_case` | 函数、变量、类型成员统一小写下划线；禁止 camelCase |
| **成员统一尾下划线 `_`** | `x_` / `q0_` / `e_int_` / `acc_` / `gt_`。**`struct` 与 `class` 不作区分** —— C++ 中二者只差默认访问权限，不把编译器的默认值编码进命名。core / host 全层一致 |
| **参数与局部变量不带 `_`** | 尾下划线的唯一职责是区分**成员**与**参数/局部**。若参数也带 `_`，约定即失去意义 |
| `is_` 前缀 | bool 成员 = `is_` 前缀 + 尾下划线，如 `is_meas_` |

设计决策编号（F1~F13）见 [`docs/DESIGN.md`](docs/DESIGN.md) §5，代码注释中的 `F#` 即指向对应决策。

## 文档

| 文件 | 内容 |
|---|---|
| [`docs/DESIGN.md`](docs/DESIGN.md) | **设计权威**：本质公式 / 场景覆盖 / 设计决策 / 决策点状态表 |
| [`docs/DEV.md`](docs/DEV.md) | 开发流程：体量 / 理论路径 / 工程规范 / 开工顺序 |
| [`docs/RESEARCH.md`](docs/RESEARCH.md) | 调研：算法全景 / 开源库 / 参考库分析 |
| [`docs/CODE.md`](docs/CODE.md) | 代码施工文档（逐批次真实代码） |
| [`docs/HANDOFF.md`](docs/HANDOFF.md) | 交接速查 |
| [`docs/答疑.md`](docs/答疑.md) | **概念问答录**：理解过程中的问题与解答（直觉 / 推导 / 代码索引，含错答修正痕迹） |

## 参考实现

`reference/` 收录三个参考库（只读，不入 git）：`CtrBoard-H7_IMU_Altitude`、`IMU_Attitude_Estimator`（gaochq）、`MahonyAHRS`（PaulStoffregen）。

## 许可

[MIT](LICENSE)
