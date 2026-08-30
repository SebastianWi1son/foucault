#pragma once

#include "solver/mahony.hpp"

namespace foucault {


enum class Scene {
    gimbal,
    car,
    infantry
};

inline solver::MahonyConfig make_mahony_config(Scene scene) {
    (void)scene;
    solver::MahonyConfig cfg;
    cfg.kp_ = 5.0f;
    cfg.ki_ = 0.3f;
    cfg.integral_limit_ = 10.0f;
    return cfg;
}




}