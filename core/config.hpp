#pragma once

#include "solver/mahony.hpp"

namespace foucault {

// 维度模式（数学抽象，核心层只懂数学；产品语义属应用层）
// 产品映射（注释即文档）：地面小车 → d2；云台/立体步兵 → d3
enum class Dimension {
    d2,   // 2D：只估 yaw，平面运动（地面小车）—— 批次 5 落地
    d25,  // 2.5D：平面运动 + 倾角 —— 批次 5 落地
    d3    // 3D：全姿态（云台/立体步兵）—— 当前唯一实现
};

inline solver::MahonyConfig make_mahony_config(Dimension dim) {
    (void)dim;   // 本批各维度同起步参数；分档在 EKF 的 Q/R（批次 4）与维度模式（批次 5）
    solver::MahonyConfig cfg;
    cfg.kp_ = 5.0f;               // 比例增益（残差 → 角速度修正力度）
    cfg.ki_ = 0.3f;               // 积分增益（零偏估计）
    cfg.integral_limit_ = 10.0f;  // 积分限幅（防饱和，rad/s）
    return cfg;
}

}
