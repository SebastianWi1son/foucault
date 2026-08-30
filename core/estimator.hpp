#pragma once

#include "config.hpp"
#include "math/quat.hpp"
#include "math/vec3.hpp"
#include "measure/measure.hpp"
#include "solver/mahony.hpp"

namespace foucault {


template <typename Solver = solver::Mahony>
class Estimator {
public:
    explicit Estimator(Scene scene = Scene::gimbal) : solver_(make_mahony_config(scene)) {}

    void predict(const measure::IMUSample& s, float dt) { solver_.predict(s.gyro_, dt); }
    void observe(const measure::IMUSample& s, float dt) { solver_.observe(s.acc_, dt); }

    void observe_yaw(float /*yaw_rad*/, float /*dt*/) {}

    void reset() { solver_.reset(); }
    void reset(const math::Quatf& q) { solver_.reset(q); }

    const math::Quatf& quaternion() const { return solver_.quaternion(); }
    math::Vec3f euler() const { return solver_.euler(); }
private:
    Solver solver_;
};




}