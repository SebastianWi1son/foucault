#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace foucault::math {


// quick result of '1/sqrt(x)'
// float: 经典 位魔法？ + 两轮牛顿迭代
// --- float sqrt calc with optimization ----
template <typename T>
inline T inv_sqrt(T x) {
    if constexpr (std::is_same_v<T, float>) {
        float y = x;
        std::int32_t i;
        std::memcpy(&i, &y, sizeof(i));
        i =  0x5f3759df - (i >> 1);
        std::memcpy(&y, &i, sizeof(i));
        y = y * (1.5f - (0.5f * x * y * y));
        y = y * (1.5f - (0.5f * x * y * y));
        return y;
    }
    else { return (T(1) / std::sqrt(x)); }
}

template <typename T>
inline T clamp(T v, T lo, T hi) { return (v < lo) ? lo : ((v > hi) ? hi : v);   }

template <typename T>
inline T deg_to_rad(T deg) { return deg * T(0.017453292519943295); }   // π/180

template <typename T>
inline T rad_to_deg(T rad) { return rad * T(57.29577951308232); }      // 180/π




}
