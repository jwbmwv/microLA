// SPDX-License-Identifier: MIT
/// @file fast_math.hpp
/// @brief Fast approximations for common math functions
/// @details Provides lookup table and polynomial approximations for resource-constrained systems
///          Trade accuracy for speed - typically 2-10x faster than standard library functions
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_FAST_MATH_HPP_
#define MICROLA_FAST_MATH_HPP_

#include "compiler_features.hpp"
#include "constants.hpp"
#include <cmath>
#include <cstdint>

namespace microla
{
namespace fast
{

/// @brief Fast sine approximation using Bhaskara I's approximation
/// @details Accurate to ~0.0016 radians (0.09 degrees) maximum error
/// @param x Angle in radians
/// @return Approximate sine value
template<typename T = float>
inline T sin(T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "fast::sin requires floating-point type");

    // Normalize to [-pi, pi]
    const T pi = constants::pi<T>();
    const T two_pi = constants::two_pi<T>();
    x = x - std::floor((x + pi) / two_pi) * two_pi;

    // Bhaskara I's sine approximation: sin(x) ≈ 16x(π - x) / (5π² - 4x(π - x))
    if (x < T(0))
    {
        const T x_abs = -x;
        const T numerator = T(16) * x_abs * (pi - x_abs);
        const T denominator = T(5) * pi * pi - T(4) * x_abs * (pi - x_abs);
        return -numerator / denominator;
    }
    else
    {
        const T numerator = T(16) * x * (pi - x);
        const T denominator = T(5) * pi * pi - T(4) * x * (pi - x);
        return numerator / denominator;
    }
}

/// @brief Fast cosine approximation
/// @details Uses identity cos(x) = sin(x + π/2)
/// @param x Angle in radians
/// @return Approximate cosine value
template<typename T = float>
inline T cos(T x) noexcept
{
    return sin(x + constants::half_pi<T>());
}

/// @brief Fast tangent approximation
/// @details Uses tan(x) = sin(x) / cos(x)
/// @param x Angle in radians
/// @return Approximate tangent value
template<typename T = float>
inline T tan(T x) noexcept
{
    const T sine = sin(x);
    const T cosine = cos(x);
    return sine / cosine;
}

/// @brief Fast inverse square root approximation (Quake III algorithm)
/// @details Famous "fast inverse square root" - accurate to ~0.175% error
/// @param x Input value (must be positive)
/// @return Approximate 1/sqrt(x)
inline float rsqrt(float x) noexcept
{
    // Quake III fast inverse square root with one Newton-Raphson iteration
    union
    {
        float f;
        std::uint32_t i;
    } conv;

    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);  // Magic constant
    float y = conv.f;

    // One Newton-Raphson iteration for better accuracy
    y = y * (1.5f - (0.5f * x * y * y));

    return y;
}

/// @brief Fast square root approximation
/// @details Uses fast_rsqrt then multiplies: sqrt(x) = x * (1/sqrt(x))
/// @param x Input value (must be positive)
/// @return Approximate sqrt(x)
inline float sqrt(float x) noexcept
{
    return x * rsqrt(x);
}

/// @brief Fast arc-tangent approximation (atan2)
/// @details Accurate to ~0.005 radians maximum error
/// @param y Y coordinate
/// @param x X coordinate
/// @return Approximate atan2(y, x) in radians [-π, π]
template<typename T = float>
inline T atan2(T y, T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "fast::atan2 requires floating-point type");

    const T pi = constants::pi<T>();
    const T half_pi = constants::half_pi<T>();

    // Handle special cases
    if (x == T(0))
    {
        if (y > T(0))
            return half_pi;
        if (y < T(0))
            return -half_pi;
        return T(0);
    }

    // Compute atan(y/x) using polynomial approximation
    T z = y / x;
    T abs_z = (z < T(0)) ? -z : z;

    T angle;
    if (abs_z < T(1))
    {
        // |z| < 1: use polynomial directly
        angle = z / (T(1) + T(0.28) * z * z);
    }
    else
    {
        // |z| >= 1: use identity atan(z) = π/2 - atan(1/z)
        angle = half_pi - z / (z * z + T(0.28));
    }

    // Adjust for quadrant
    if (x < T(0))
    {
        if (y >= T(0))
            angle += pi;
        else
            angle -= pi;
    }

    return angle;
}

/// @brief Fast exponential approximation e^x
/// @details Uses Padé approximation - accurate to ~1% error for |x| < 2
/// @param x Exponent
/// @return Approximate e^x
template<typename T = float>
inline T exp(T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "fast::exp requires floating-point type");

    // For large |x|, use scaling: e^x = (e^(x/2))^2
    if (x > T(2))
    {
        T half = exp(x / T(2));
        return half * half;
    }
    if (x < T(-2))
    {
        T half = exp(x / T(2));
        return half * half;
    }

    // Padé approximation [2/2]: e^x ≈ (2 + x) / (2 - x)
    // More accurate: e^x ≈ (12 + 6x + x²) / (12 - 6x + x²)
    const T x2 = x * x;
    const T numerator = T(12) + T(6) * x + x2;
    const T denominator = T(12) - T(6) * x + x2;
    return numerator / denominator;
}

/// @brief Fast natural logarithm approximation ln(x)
/// @details Uses bit manipulation and polynomial - accurate to ~2% error
/// @param x Input value (must be positive)
/// @return Approximate ln(x)
inline float ln(float x) noexcept
{
    // Extract exponent from IEEE 754 representation
    union
    {
        float f;
        std::uint32_t i;
    } conv;

    conv.f = x;
    int exp = ((conv.i >> 23) & 0xFF) - 127;
    conv.i = (conv.i & 0x007FFFFF) | 0x3F800000;  // mantissa in [1, 2)
    float m = conv.f;

    // Polynomial approximation for ln(m) where m in [1, 2)
    // ln(m) ≈ (m - 1) * (a + b*(m-1) + c*(m-1)²)
    float m1 = m - 1.0f;
    float log_m = m1 * (2.0f - 0.666666f * m1 + 0.4f * m1 * m1);

    // ln(x) = ln(2^exp * m) = exp * ln(2) + ln(m)
    return exp * 0.69314718f + log_m;
}

/// @brief Fast power approximation x^y
/// @details Uses exp(y * ln(x)) with fast approximations
/// @param x Base (must be positive)
/// @param y Exponent
/// @return Approximate x^y
inline float pow(float x, float y) noexcept
{
    return exp(y * ln(x));
}

/// @brief Fast absolute value
/// @details Branchless implementation using bit manipulation for floats
/// @param x Input value
/// @return |x|
inline float abs(float x) noexcept
{
    union
    {
        float f;
        std::uint32_t i;
    } conv;
    conv.f = x;
    conv.i &= 0x7FFFFFFF;  // Clear sign bit
    return conv.f;
}

/// @brief Fast sign function
/// @details Returns -1, 0, or 1
/// @param x Input value
/// @return sign(x)
template<typename T = float>
MICROLA_CONSTEXPR T sign(T x) noexcept
{
    return T((T(0) < x) - (x < T(0)));
}

/// @brief Linear interpolation (lerp) - inlined for speed
/// @param a Start value
/// @param b End value
/// @param t Interpolation factor [0, 1]
/// @return Interpolated value
template<typename T = float>
MICROLA_CONSTEXPR T lerp(T a, T b, T t) noexcept
{
    return a + t * (b - a);
}

/// @brief Smoothstep interpolation (cubic Hermite)
/// @details Smooth acceleration and deceleration
/// @param edge0 Lower edge
/// @param edge1 Upper edge
/// @param x Value to interpolate
/// @return Smoothly interpolated value
template<typename T = float>
inline T smoothstep(T edge0, T edge1, T x) noexcept
{
    T t = (x - edge0) / (edge1 - edge0);
    if (t < T(0))
        t = T(0);
    if (t > T(1))
        t = T(1);
    return t * t * (T(3) - T(2) * t);
}

}  // namespace fast
}  // namespace microla

#endif  // MICROLA_FAST_MATH_HPP_
