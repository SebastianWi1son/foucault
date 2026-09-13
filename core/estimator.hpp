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
    explicit Estimator(Dimension dim = Dimension::d3) : solver_(make_mahony_config(dim)) {}

    void predict(const measure::IMUSample& s, float dt) { solver_.predict(s.gyro_, dt); }
    void observe(const measure::IMUSample& s) { solver_.observe(s.acc_); }
    void observe_heading(float heading_rad, float trust = 1.0f) { solver_.observe_heading(heading_rad, trust); }

    void reset() { solver_.reset(); }
    void reset(const math::Quatf& q) { solver_.reset(q); }

    const math::Quatf& quaternion() const { return solver_.quaternion(); }
    math::Vec3f euler() const { return solver_.euler(); }

    unsigned rejected_count() const { return solver_.rejected_count(); }
private:
    Solver solver_;
};




}