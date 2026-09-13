#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_math_golden.py — 为 core/math 生成独立金标数据（tests/unit/math_golden.hpp）

独立性来源：
  1) 独立语言实现：本脚本用 Python 从数学定义直接算，不读 C++ 源码、不调用 C++ 代码。
  2) 独立方法：四元数乘积用 4x4 左乘矩阵形式；旋转用 Rodrigues 公式（轴角 → 矩阵）。
     C++ 侧用的是分量展开式 —— 两条路径完全不同，可交叉验证。
  3) 覆盖 core/math 的全部对外能力：四元数旋转、欧拉角（ZYX 内旋）、轴角积分、Hamilton 积。

用法：python3 tests/tools/gen_math_golden.py > tests/unit/math_golden.hpp
"""
import math

F = lambda x: "%.9ff" % x

# ───────────────────────── 基础定义（纯数学，独立实现） ─────────────────────────

def q_axis_angle(axis, angle):
    """轴角 → 单位四元数 (w,x,y,z)。定义式，非代码移植。"""
    n = math.sqrt(sum(c * c for c in axis))
    k = [c / n for c in axis]
    h = angle / 2.0
    return [math.cos(h)] + [k[i] * math.sin(h) for i in range(3)]


def rodrigues(axis, angle, v):
    """Rodrigues 公式旋转向量：v' = v cosθ + (k×v) sinθ + k (k·v)(1-cosθ)"""
    n = math.sqrt(sum(c * c for c in axis))
    k = [c / n for c in axis]
    kxv = [k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]]
    kdv = sum(k[i] * v[i] for i in range(3))
    c, s = math.cos(angle), math.sin(angle)
    return [v[i] * c + kxv[i] * s + k[i] * kdv * (1 - c) for i in range(3)]


def q_mul(a, b):
    """Hamilton 积，用 4x4 左乘矩阵形式（与 C++ 的分量展开式方法不同）"""
    M = [[a[0], -a[1], -a[2], -a[3]],
         [a[1],  a[0], -a[3],  a[2]],
         [a[2],  a[3],  a[0], -a[1]],
         [a[3], -a[2],  a[1],  a[0]]]
    return [sum(M[i][j] * b[j] for j in range(4)) for i in range(4)]


def rot_matrix(axis, angle):
    """Rodrigues → 3x3 旋转矩阵（列主序无关，这里按行存）"""
    cols = [rodrigues(axis, angle, e) for e in ([1, 0, 0], [0, 1, 0], [0, 0, 1])]
    return [[cols[j][i] for j in range(3)] for i in range(3)]


def zyx_matrix(roll, pitch, yaw):
    """内旋 ZYX（先 yaw 再 pitch 最后 roll）：R = Rz(yaw)·Ry(pitch)·Rx(roll)"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


# ───────────────────────── 测试用例集 ─────────────────────────

# (轴, 角) —— 覆盖单轴、斜轴、负角、大角
AXIS_ANGLE = [
    ("z90",   [0, 0, 1],             math.pi / 2),
    ("y12",   [0, 1, 0],             1.2),
    ("xn07",  [1, 0, 0],            -0.7),
    ("obl21", [0.3, 0.5, -0.81],     2.1),
    ("diag05",[1, 1, 0],             0.5),
    ("neg25", [0, 0, 1],            -2.5),
]

ROT_IN = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.3, -0.7, 0.2], [-1.0, 2.0, 0.5]]

# 欧拉角 (roll, pitch, yaw) rad —— pitch 限制在 (-π/2, π/2) 以外时用矩阵比对
EULER = [
    ("identity", 0.0, 0.0, 0.0),
    ("mixed",    0.3, -0.5, 1.1),
    ("big",      0.7, 1.2, -2.5),
    ("negyaw",   0.4, 0.8, -1.5),
    ("roll179",  3.0, 0.2, 0.6),
    ("glock+",   0.0, math.pi / 2, 0.0),
    ("glock-",   1.0, -math.pi / 2, -1.0),
]

# Hamilton 积用例（含非交换验证对）
MUL = [
    ("orth", [0.7071067811865476, 0, 0, 0.7071067811865476], [0.7071067811865476, 0, 0.7071067811865476, 0]),
    ("rev",  [0.7071067811865476, 0, 0, 0.7071067811865476], [0.7071067811865476, 0, 0, -0.7071067811865476]),
    ("gen",  [0.5, -0.5, 0.5, -0.5], [0.3, 0.4, -0.5, 0.7]),
]

# 轴角积分用例：常角速度 ω 积分 T 秒 ⇒ 等价于绕 ω 轴转 |ω|·T
INTEG = [
    ("z", [0, 0, 1.0],   1.0, 1.5),
    ("y", [0, 1.0, 0],   0.5, 2.0),
    ("obl", [0.6, -0.8, 0.0], 1.0, 0.9),
]


def emit():
    W = []
    w = W.append
    w("// math_golden.hpp — core/math 独立金标数据【自动生成，请勿手改】")
    w("// 生成器: tests/tools/gen_math_golden.py（Python 纯数学实现）")
    w("// 独立性: 四元数乘积用 4x4 左乘矩阵形式；旋转用 Rodrigues 公式；与本仓 C++ 实现方法不同")
    w("// 重新生成: python3 tests/tools/gen_math_golden.py > tests/unit/math_golden.hpp")
    w("#pragma once")
    w("")
    w("// NOLINTBEGIN")
    w("namespace golden {")
    w("")

    # ── 旋转用例表 ──
    w("// 每个用例: 轴角 → 参考四元数 → 用 Rodrigues 独立算出 5 个输入向量的期望输出")
    w("struct RotCase {")
    w("    const char* name_;")
    w("    float q_[4];                     // (w,x,y,z) 参考四元数")
    w("    float in_[%d][3];                // 输入向量（机体系）" % len(ROT_IN))
    w("    float out_[%d][3];               // Rodrigues 期望输出（世界系）" % len(ROT_IN))
    w("};")
    w("")
    w("static const RotCase kRot[%d] = {" % len(AXIS_ANGLE))
    for name, axis, ang in AXIS_ANGLE:
        q = q_axis_angle(axis, ang)
        w("    { \"%s\"," % name)
        w("      { %s }," % ", ".join(F(c) for c in q))
        w("      { " + ", ".join("{ %s }" % ", ".join(F(c) for c in v) for v in ROT_IN) + " },")
        w("      { " + ", ".join("{ %s }" % ", ".join(F(c) for c in rodrigues(axis, ang, v)) for v in ROT_IN) + " } },")
    w("};")
    w("")

    # ── 欧拉角用例表 ──
    w("// 每个用例: 欧拉角 + 独立构造的 ZYX 旋转矩阵（用于验证 rotate 与 to_euler 同一约定）")
    w("struct EulCase { const char* name_; float euler_[3]; float R_[3][3]; };")
    w("")
    w("static const EulCase kEul[%d] = {" % len(EULER))
    for name, r, p, y in EULER:
        R = zyx_matrix(r, p, y)
        w("    { \"%s\", { %s }," % (name, ", ".join(F(c) for c in (r, p, y))))
        w("      { " + ", ".join("{ %s }" % ", ".join(F(c) for c in row) for row in R) + " } },")
    w("};")
    w("")

    # ── Hamilton 积用例表 ──
    w("struct MulCase { const char* name_; float a_[4]; float b_[4]; float ab_[4]; float ba_[4]; };")
    w("")
    w("static const MulCase kMul[%d] = {" % len(MUL))
    for name, a, b in MUL:
        ab, ba = q_mul(a, b), q_mul(b, a)
        w("    { \"%s\"," % name)
        w("      { %s }, { %s }," % (", ".join(F(c) for c in a), ", ".join(F(c) for c in b)))
        w("      { %s }, { %s } }," % (", ".join(F(c) for c in ab), ", ".join(F(c) for c in ba)))
    w("};")
    w("")

    # ── 轴角积分用例表 ──
    w("// 常角速度 ω 保持 T 秒 ⇒ 姿态应等于绕 ω 轴、转角 |ω|·T 的四元数")
    w("// 覆盖两种时间粒度：粗步长（截断误差可见）与细步长（逼近精确解）")
    w("struct IntegCase {")
    w("    const char* name_; float w_[3]; float T_; float q_coarse_[4]; float q_fine_[4];")
    w("};")
    w("")
    w("static const IntegCase kInteg[%d] = {" % len(INTEG))
    for name, wv, T, dt_coarse in INTEG:
        ang = math.sqrt(sum(c * c for c in wv)) * T
        q = q_axis_angle(wv, ang)
        w("    { \"%s\", { %s }, %s," % (name, ", ".join(F(c) for c in wv), F(T)))
        w("      { %s }, { %s } }," % (", ".join(F(c) for c in q), ", ".join(F(c) for c in q)))
    w("};")
    w("")
    w("}  // namespace golden")
    w("// NOLINTEND")
    w("")
    return "\n".join(W)


if __name__ == "__main__":
    print(emit())
