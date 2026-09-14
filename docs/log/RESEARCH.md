---
class: log
generated: false
---
> **类：A 日志（append-only）** —— 调研记录。只增不改，**不代表当前状态或当前决策**。
> 决策权威见 [`../DESIGN.md`](../DESIGN.md)。

# RESEARCH.md — 调研：算法全景、开源库、参考库分析

> 分类文档（调研类）。本文件是算法与市场调研的**唯一权威**，更新集中在此。
> 创建：2026-08-28（合并原 01/04/05/06 调研内容）

---

## 1. 主流姿态解算算法全景

### 1.1 所有算法共享同一骨架

```
陀螺仪 ω ──► 积分（预测）──► 姿态估计 q̂ ──► 输出
                 ▲                │
                 │                ▼
            增益 K ◄── 残差 err ◄── 向量观测 z（加速度计/磁力计/视觉）
```

陀螺短时精确但会漂移，向量观测绝对但含噪声。**全部算法的区别只有一点：增益 K 怎么算。**

| 算法 | 增益 K 的求法 | 本质归类 |
|---|---|---|
| 经典互补滤波 | 常数 α / 一阶低高通 | 固定增益观测器 |
| Mahony | 常数 Kp（比例）+ Ki（积分消零偏） | 固定增益观测器（PI 控制器） |
| Madgwick | 梯度下降步长 β 决定 | 固定增益观测器（优化视角） |
| 线性 Kalman | 协方差递推 P·Hᵀ/(H·P·Hᵀ+R) | 贝叶斯最优（线性假设） |
| EKF | 同上，但 F/H 用雅可比线性化 | 贝叶斯最优（一阶线性化） |
| ESKF / IEKF | 同上，但状态是误差量 | 贝叶斯最优（数值稳定性更好） |
| UKF | sigma 点传播，无雅可比 | 贝叶斯最优（高阶近似） |
| VQF | 特殊设计 + 磁干扰检测 | 混合（xio 公司，2021） |
| SO(3) 非线性观测器 | 李群上的李雅普诺夫设计 | 固定增益（理论最漂亮） |

### 1.2 逐个拆解

**经典互补滤波**：一维形式 `θ̂ = α·(θ̂+ω·dt) + (1−α)·θ_acc`，高通陀螺 + 低通加速度计的频率域拼接。几行代码、极快，但多轴耦合难调、无零偏估计。适用：玩具级、低功耗穿戴。

**Mahony**（显式互补滤波，Mahony/Hamel/Trumpf IEEE TAC 2008）：
- 加速度计实测重力方向与姿态预测重力方向**叉积**得误差角，PI 控制器拉回：
```
err = a_meas × a_pred
q̂̇ = ½·q̂⊗ω + Kp·err + Ki·∫err   （Ki 项在线估计并消除陀螺零偏）
```
- 优点：计算量极小（H723 实测 1.33µs）、理论有稳定性证明、调参仅 Kp≈0.5~2.0 / Ki≈0~0.1。
- 缺点：固定增益 —— 静态时噪声渗入、动态时跟不上；Ki 在剧烈运动时误学零偏。
- 适用：**MCU 实时控制场景的默认首选**。

**Madgwick**（2011）：把"姿态对齐量测"建模成最优化问题，沿梯度下降方向修正，步长 β 唯一主参数。与 Mahony 差异很小（叉积是小角度近似下的梯度方向）。优点：单参数、收敛快。缺点：β 固定，静态噪声与动态滞后矛盾依旧。

**线性 Kalman**：只对线性模型+高斯噪声最优。姿态运动学非线性，线性 KF 只能用于单轴小角度近似（一维倾角 Kalman）或把 yaw 单独拿出来做一维 Kalman —— **这是地面小车 gyro-z 方案的天然升级路径**。

**EKF**（参考库采用）：状态 `[q0 q1 q2 q3 b_x b_y b_z]`（四元数+陀螺零偏），对 F/H 做一阶泰勒展开，协方差递推出时变最优增益。
- 优点：增益自适应（静止信量测、动态信陀螺）；显式建模零偏；可挂**卡方检验**剔除异常量测；可随时加传感器不改框架。
- 缺点：计算量大（H723 实测 29.4µs vs Mahony 1.33µs，差 20 倍）；调参维度高（Q/R/初始 P）；线性化误差在剧烈运动时仍存在。
- 适用：RoboMaster 步兵/无人机、抗冲击与多传感器融合场景。

**ESKF/IEKF**（PX4 选择）：不直接估四元数，估**误差状态**（小量可安全线性化），四元数误差只用 3 参数 → 无奇异、数值稳定。PX4 EKF2 = 24 状态 ESKF（四元数4+速度3+位置3+陀螺零偏3+加速度计偏置3+磁场6+风2）。

**VQF**：xio 2021，磁干扰在线检测与剔除、对动态加速度鲁棒。代价：每次更新 4400~10000 cycles（Mahony/Madgwick 的 5~10 倍），Flash 28~32KB。小 MCU 慎用。

### 1.3 实验数据：没有绝对赢家

| 研究 | 结论 |
|---|---|
| 四旋翼飞行数据 | Mahony RMSE 最小且最快，EKF 最慢且精度未必最好 |
| KUKA 机械臂基准 | EKF 精度最好，代价算力 |
| 足绑 MIMU 数据 | Madgwick 航向最好，略慢于 Mahony |
| 外部加速度干扰 | Madgwick 抑制最好；Kalman/Mahony 抗振动更好 |
| MPU6050+Arduino 实测 | Kalman 精度优先推荐；Mahony 最快（950µs vs KF 1376µs） |

**结论：精度取决于传感器质量、运动类型与调参水平，不取决于算法品牌。EKF 调不好可以比 Mahony 更差。**

### 1.4 选型建议（通用核心的算法层）

| 需求档位 | 推荐算法 | 理由 |
|---|---|---|
| 纯 yaw / 单轴（地面小车） | 一维 KF 或带高通的陀螺积分 | 状态小、可解释、可调性最好 |
| 6 轴 3D，算力紧张（云台内环） | Mahony / Madgwick | 1~2µs 级，1kHz 环轻松跑 |
| 6 轴 3D，恶劣工况（步兵颠簸） | EKF（卡方检验+静止检测） | 参考库已验证，抗冲击刚需 |
| 多传感器融合（磁/视觉/里程计） | ESKF 架构 | PX4 同款，扩展性最强 |

**通用核心的正确姿态：不是"只写一个算法"，而是"一个内核框架 + 可插拔增益求解器"。**

---

## 2. 开源库调研

### 2.1 全景（按可参考价值分层）

**第一层 · 必读经典（算法源头，200~400 行，建议逐行读）**
- x-io / MadgwickAHRS：Madgwick 官方实现，C 零依赖，被移植到全世界所有平台
- x-io / MahonyAHRS：Mahony 官方实现，含磁力计支持
- 参考库 MahonyAHRS.c：就是上者的移植 + `Mahony_invSqrt`（0x5f3759df 快速倒数开方）

**第二层 · 生产级飞控（功能最全、验证最充分、体量最大）**
- PX4 / ECL：EKF2（24 状态 ESKF），融合 IMU/GPS/视觉/气压计/磁力计，BSD-3
- ArduPilot / EKF3 + DCM：24 状态 EKF3 多 lane 冗余 + 互补滤波 DCM 兜底，**GPLv3（注意传染性）**

**第三层 · 商业闭源/半开源（市场对标）**
- ST MotionFX（闭源，仅 ST MCU）、TDK InvenSense（闭源）、NXP Sensor Fusion（半开源）、Bosch BSX（闭源）

**第四层 · 社区高质量库（最接近本项目目标）**
- **xioTechnologies/Fusion**（imufusion）：C99 零 malloc，MIT，句柄式 API 教科书
- **martinbudden/Library-SensorFusion**：C++，Complementary+Mahony+Madgwick+VQF 同框架 —— "可插拔增益"的现成演示
- **liuskywalkerjskd/6-Axis-AHRS**：Madgwick + Tactical EKF，为 RoboMaster 写（冲击检测/运动加速度补偿/零偏修正/heave）—— 与步兵场景直接对口
- **WangHongxi2001/RoboMaster-C-Board-INS-Example**：RM 圈最知名惯导项目（参考库源头），卡方+渐消
- **imufusion-c**：一个头文件四算法（Complementary/Madgwick/Mahony/EKF）
- **jettify/uf-ahrs**：Rust no_std，BROAD 数据集评估 —— "如何用公开数据验证算法"的范式
- **richcreations/imud**：IMU 版 gpsd，时间戳管理/校准/MAVLink

**第五层 · Python 验证工具**
- **ahrs**（PyPI）：19 种算法统一接口，调参对比神器
- **imufusion**（PyPI）：xio Fusion 的 Python 版

### 2.2 重点拆解

**xioTechnologies/Fusion**：句柄式 `FusionAhrs`（无全局变量 —— 可移植性第一要义，对比参考库全局单例）；支持 gain 设置、倾斜校准、磁力计外部校准、陀螺 bias 在线估计。

**6-Axis-AHRS / Tactical EKF**：说明 6 轴场景下"量测仲裁"（信不信加速度计）比滤波算法本身更值钱。

**PX4 ECL EKF2**：不直接移植，但其"哪些状态该建模、哪些量测何时可用"的清单是 profile 设计的教科书。

### 2.3 结论与选型建议

1. **没有现成的"通用可移植核心"可直接抄**；最接近的是 imufusion-c 与 Library-SensorFusion，但 EKF 深度与场景技巧不如参考库和 6-Axis-AHRS；
2. **推荐组合借鉴**：API 设计 ← xio Fusion；多算法插拔 ← Library-SensorFusion / imufusion-c；EKF 工程加固 ← WangHongxi INS；场景技巧 ← 6-Axis-AHRS；验证方法 ← uf-ahrs / ahrs；
3. **许可注意**：ArduPilot GPLv3（传染），参考库未标许可；发布时避免直接复制 GPL 代码，BSD/MIT 优先。

---

## 3. 参考库深度分析（CtrBoard-H7_IMU_Altitude）

> 来源：`reference/CtrBoard-H7_IMU_Altitude`（cmjang，达妙 H723 板，BMI088）
> EKF 移植自 WangHongxi2001/RoboMaster-C-Board-INS-Example，Mahony 为魔改版，另含恒温控制。

### 3.1 项目全景

```
AC5_Code / AC6_Code        # Keil AC5/AC6 双版本
├── App/imu_temp_ctrl.c    # 陀螺仪恒温控制（40°C，PID）
├── Device/Algorithm/      # QuaternionEKF(531行) + kalman_filter(548行) + algorithm(VOFA)
├── Device/Mahony/         # MahonyAHRS.c(321行)
├── Device/BMI088/         # 驱动
└── (HAL/FreeRTOS/USB-CDC 框架)
```

实测（H723 @550MHz）：**Mahony 含欧拉角 1.33µs；EKF 含欧拉角 29.4µs**。

### 3.2 EKF 设计逐项拆解

**状态与量测**：`x ∈ R⁶ : [q0 q1 q2 q3 b_x b_y]`（四元数 + x/y 零偏），量测 = 归一化加速度（一阶低通后）。刻意不估 b_z（"z轴通天，yaw 漂移不可观"），且量测更新中 q3 修正强制置零。

**预测步**：F 矩阵手工装配（左上 4×4 四元数运动学 `½Ω(ω−b)·dt`、右上 4×2 零偏耦合、右下 2×2 单位阵），每次预测前 memcpy 覆盖。Q = diag(Q1·dt×4, Q2·dt×2) —— 随 dt 缩放（正确做法）。

**量测更新（精华）**：
- (a) 静止检测：`gyro_norm<0.3 && accl_norm∈[9.3,10.3]` → StableFlag；
- (b) 卡方检验：`ChiSquare = (z−h)ᵀS⁻¹(z−h)` 超阈值 → 跳过量测更新（只预测），防运动加速度污染；连续异常计数 >50 判发散恢复更新；
- (c) 自适应增益：残差在 [0.1×thr, 1×thr] 区间增益线性缩水，残差越大越不信。

**零偏防呆全家桶**：渐消因子 λ（P[28]/P[35] ×= 1/0.9996，防过收敛）、协方差限幅 >10000 截断、修正限幅 |Δb|≤0.01·dt、方向余弦加权（弱观测方向少修）、双状态初始化。

**输出层**：四元数→欧拉角标准 atan2/asin 公式；`YawTotalAngle` 360° 展开（小陀螺场景）。

### 3.3 Mahony（MahonyAHRS.c）

标准 Madgwick 版 Mahony：叉积误差 + PI 补偿律；接口保留磁力计参数但调用处传 0（6 轴模式）；`Mahony_invSqrt` 快速倒数开方全平台可用。

### 3.4 可移植性评估（对本项目的启示）

| 依赖 | 位置 | 移植成本 |
|---|---|---|
| ARM DSP 矩阵库 | kalman_filter.h 8 个宏 | **最大障碍**，需自实现定长矩阵 |
| arm_atan2_f32 / __sqrtf | EKF 与 Mahony | 换标准库即可 |
| user_malloc | FreeRTOS pvPortMalloc | 应改零动态内存 |
| 全局单例 QEKF_INS | QuaternionEKF.c | 多实例不兼容 |
| 硬编码阈值 | 0.3 rad/s、±0.5g、0.01 | 应参数化 |
| 板级绑定 | BMI088/恒温/VOFA/USB | demo 层，与算法无关 |

### 3.5 优缺点总结

**值得继承**：① 卡方+静止检测+自适应增益的"量测仲裁"（比算法本身值钱）；② 零偏估计防呆全家桶；③ 明确的 yaw 不可观处理；④ 双算法对照框架 + VOFA 工作流；⑤ 恒温控制（yaw 十分钟漂移 ~20°→~10°）。

**局限（差异化空间）**：强绑定 ARM DSP 与单板；全局单例/硬编码阈值；磁力计接口空置；无 2D 降级模式；yaw 防漂靠"抹掉小角速度"cheat。

---

## 4. 新入库审阅（gaochq / PaulStoffregen，~200★）

### 4.1 gaochq/IMU_Attitude_Estimator —— 桌面研究框架

- **定位**：Linux 桌面算法研究/对比框架，非嵌入式库。C++ + Eigen3 + glog + matplotlib-cpp，ORB-SLAM 风格线程封装。
- **算法**：Mahony + EKF（12 状态）+ ESKF（6 状态误差）三合一，统一 `Run(Vector_9)` 接口。
  - EKF：12 状态 = 角速度3 + 角加速度3 + 加速度状态3 + 磁状态3 —— 老 Pixhawk attitude_estimator_ekf 的方向矢量参数化，**已被 PX4 淘汰，参考价值有限**；
  - ESKF：现代误差状态（Solà 2017 同款）：标称四元数+陀螺零偏，误差 θ(3)+b(3)，6×6 协方差，量测 9 维（陀螺+加速度+磁）—— **全项目最值得抄的部分**。
- **工具链**：`Allan_Analysis.py`（艾伦方差→标定 Q/R）、`DataSets.py`、自带 NAV1/NAV2 真实数据集+真值、三算法同图对比。
- **可移植性差，但"算法数学 + 验证工作流"参考极佳**。

### 4.2 PaulStoffregen/MahonyAHRS —— Arduino 生态标杆

- **定位**：Teensy 作者维护的 Arduino 库，Mahony 的 C++ 封装。单 .cpp/.h，仅依赖 `<math.h>`，零 STL 零 malloc —— **可移植性满分**。
- **API**：`Mahony` 类：begin(freq) / update(9参含磁) / updateIMU(6参) / getRoll/getPitch/getYaw（惰性计算 anglesComputed 标志）。
- **亮点**：惰性欧拉角换算；积分项防 windup（Ki=0 时清零积分器）；getYaw() 自动 +180° 对准航向惯例；static invSqrt；默认 Kp=0.5/Ki=0。
- **局限**：单算法；无零偏显式状态（靠 Ki）；无 2D 模式；无验证工具。

### 4.3 三方对比总表

| 维度 | CtrBoard-H7 | gaochq | PaulStoffregen |
|---|---|---|---|
| 语言 | C + ARM DSP 库 | C++ + Eigen/glog | C++ 纯标准库 |
| 目标平台 | STM32H723（嵌入式） | Linux 桌面（研究） | Arduino/Teensy（嵌入式） |
| 算法 | Mahony + EKF(6状态,仅加速度) | Mahony + EKF(12状态,旧式) + ESKF | 仅 Mahony |
| 磁力计 | 接口空置 | ESKF 完整接入 | 完整接入 |
| 零偏估计 | EKF 估 x/y 轴 | ESKF 估 3 轴 | 无显式状态（Ki 积分） |
| 工程加固 | 卡方/静止检测/渐消/限幅/恒温 | Allan 方差/数据集真值 | 惰性角度/防 windup |
| 验证手段 | VOFA 对比曲线 | 数据集+真值+三算法同图 | 无 |
| 可移植性 | 差 | 差 | **好（最接近目标形态）** |
| 可抄什么 | 生产级鲁棒技巧 | ESKF 数学+验证工作流 | API 设计+零依赖纪律 |

### 4.4 综合结论

三个库互不重叠：**CtrBoard 教"恶劣工况下活下来"（量测仲裁）；gaochq 教"算法数学与怎么证明它有效"（ESKF+数据集验证）；PaulStoffregen 教"库怎么做得干净可移植"（API+零依赖）。**
foucault = **Paul 的形态 × gaochq 的算法谱系 × CtrBoard 的鲁棒技巧** + C++ 模板化与 2D/3D 显式降维 —— 此组合目前无现成开源库占位。
