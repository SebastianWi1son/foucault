#pragma once

#include "../math/vec3.hpp"

namespace foucault::measure {


struct IMUSample {
    math::Vec3f acc_;
    math::Vec3f gyro_;
};




}