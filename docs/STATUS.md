---
class: status
generated: false   # 目标：改为 true —— 由 scripts/gen_status.sh 跑 ctest + replay 生成
---
# STATUS.md — foucault 当前状态

> **这是全项目"当前状态"的唯一来源。** 其他文档里的同类数字都是历史快照。
> **不要手改。** 目标：由脚本 `cmake --build` + `ctest` + `replay_nav2` 自动生成。
> 进度 / 待办见 [`待办.md`](待办.md)；设计与理由见 [`DESIGN.md`](DESIGN.md)。

---

## 1. 实现进度

| 能力 | 状态 | 证据 |
|---|---|---|
| 数学内核 `vec3` / `quat` / `scalar_ops` | ✅ **已冻结** | `test_math_audit`（125 断言，独立金标 + 自写参考实现） |
| Mahony 求解器（6 轴 + 外部航向 + 入口守门人） | ✅ 已验收 | `test_mahony` 24 断言 |
| `Estimator` 门面 | ⚠️ **可插拔是假的** | `estimator.hpp` 写死 `make_mahony_config`；`config.hpp` 反向依赖 solver |
| 维度模式 2D / 2.5D | ❌ 未实现 | `config.hpp` 里 `(void)dim`，三档同参 |
| 可注量测源（磁力计 / 9 轴） | ❌ 未实现 | 重力方向硬编码在 `mahony.cpp` |
| EKF / Madgwick / 1D-KF | ❌ 未实现 | 仅 Mahony |
| 平台层 / 硬件实测 | ❌ 未开始 | 等硬件（H743 + ICM-20602 + 4 路编码器） |

---

## 2. 验收基线

数据集 `data/NAV2_data.bin`（15999 行，320.0 s @50 Hz，6 轴 acc+gyro）。

| 场景 | 命令 | roll RMSE | pitch RMSE | yaw |
|---|---|---|---|---|
| 6 轴基线 | `replay_nav2 data/NAV2_data.bin` | 2.152° | 2.645° | 漂移 −14.450°（预期行为） |
| 注入真值 yaw @50Hz | `-y gt` | 2.152° | 2.645° | **RMSE 0.085°**，MAX 0.701° |
| 注入真值 yaw @10Hz | `-y gt_slow` | 2.152° | 2.645° | RMSE 0.095° |
| 加噪 yaw（σ≈2°） | `-y gt_noisy` | 2.152° | 2.645° | RMSE 1.266°，MAX 2.051° |
| yaw 断线 30 s | `-y gt_drop` | 2.152° | 2.645° | RMSE 0.385° |

**roll / pitch 在注入 yaw 后逐位不变** = 航向通道与 roll/pitch 正交的最强证据。

### 故障注入（每 100 帧一帧脏数据，共 159 帧）

| 故障 | 拦截 | RMSE |
|---|---|---|
| `acc_nan` | 159 / 159 | 2.152°（不退化） |
| `acc_zero` | 159 / 159 | 2.152° |
| `acc_sat` | 159 / 159 | 2.152° |
| `gyro_nan` | 159 / 159 | 2.152° |

拆掉守门人时 `acc_nan` / `gyro_nan` 的 RMSE = **nan**（永久损坏）；`acc_zero` 则**指标完全看不出来**（静默失效）。

---

## 3. 测试

| target | 断言数 | 说明 |
|---|---|---|
| `test_math` | 21 | 数学内核锚点 |
| `test_math_audit` | 125 | 独立审计（自写 4×4 矩阵 / Rodrigues + Python 金标） |
| `test_mahony` | 24 | Mahony 行为锚点 + 航向注入 + 守门人 |
| `test_estimator` | 6 | 门面转发 |
| `replay_nav2` | — | 数据集回归（RMSE < 5° 判据） |

```
ctest 结果：5/5 PASS（ASan + UBSan，本机复验 2026-09-13）
```

> ⚠️ **CI 只锁 `RMSE < 5°` 阈值，不锁上表的具体数字。**
> 文档宣称的"逐位相同"目前靠人工比对 → 待办：把基线写进断言（见 `待办.md`）。

---

## 4. 生成方式（待实现）

```bash
# 目标：⬜ scripts/gen_status.sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/replay_nav2 data/NAV2_data.bin            # 6 轴
./build/replay_nav2 data/NAV2_data.bin -y gt      # 注入 yaw
# …把输出写进本文件
```

配套 CI 门禁：**重跑生成脚本 → 与已提交的 `STATUS.md` 比较 → 有 diff 即失败**（证明它真的是生成的）。
