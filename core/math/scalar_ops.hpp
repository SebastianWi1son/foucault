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

// 角度归一化到 [−π, π]：角度的【差分】必须 wrap，否则 179° 与 −179° 会算出 358° 造成猛转
// 快路径：atan2 类角度的差分绝大多数已在范围内 → 零开销直接返回
template <typename T>
inline T wrap_pi(T a) {
    const T pi = T(3.14159265358979323846);
    if (a > pi || a < -pi) {
        const T two_pi = T(2) * pi;
        a = std::fmod(a + pi, two_pi);
        if (a < T(0)) { a += two_pi; }
        a -= pi;
    }
    return a;
}

template <typename T>
inline T clamp(T v, T lo, T hi) { return (v < lo) ? lo : ((v > hi) ? hi : v);   }

template <typename T>
inline T deg_to_rad(T deg) { return deg * T(0.017453292519943295); }   // π/180

template <typename T>
inline T rad_to_deg(T rad) { return rad * T(57.29577951308232); }      // 180/π




}
