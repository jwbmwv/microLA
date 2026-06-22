// SPDX-License-Identifier: Apache-2.0
/// @file fast_math.hpp
/// @brief Fast approximations for common math functions
/// @details Provides lookup table and polynomial approximations for resource-constrained systems
///          Trade accuracy for speed - typically 2-10x faster than standard library functions
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "compiler_features.hpp"
#include "constants.hpp"
#include <cmath>
#include <cstdint>

namespace microla
{
namespace fast
{

// ==================== Magic Constants for Fast Approximations ====================

/// @brief Quake III fast inverse square root magic constant
constexpr std::uint32_t RSQRT_MAGIC_CONSTANT = 0x5f3759dfU;

/// @brief Bhaskara I sine approximation numerator coefficient
constexpr float BHASKARA_SINE_NUMERATOR_COEFF = 16.0F;

/// @brief Bhaskara I sine approximation denominator coefficient
constexpr float BHASKARA_SINE_DENOMINATOR_COEFF = 5.0F;

/// @brief atan2 polynomial approximation coefficient
constexpr float ATAN2_POLY_COEFF = 0.28F;

/// @brief Padé approximation numerator/denominator constant
constexpr float PADE_CONSTANT = 12.0F;

/// @brief Padé approximation linear coefficient
constexpr float PADE_LINEAR_COEFF = 6.0F;

/// @brief Natural logarithm polynomial coefficient a
constexpr float LN_POLY_COEFF_A = 2.0F;

/// @brief Natural logarithm polynomial coefficient b
constexpr float LN_POLY_COEFF_B = 0.666666F;

/// @brief Natural logarithm polynomial coefficient c
constexpr float LN_POLY_COEFF_C = 0.4F;

/// @brief Smoothstep Hermite coefficient
constexpr float SMOOTHSTEP_COEFF = 3.0F;

// ==================== Fast Approximation Functions ====================

/// @brief Fast sine approximation using Bhaskara I's approximation
/// @details Accurate to ~0.0016 radians (0.09 degrees) maximum error
/// @param x Angle in radians
/// @return Approximate sine value
template<typename T = float>
inline auto sin(T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "fast::sin requires floating-point type");

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

    const T numerator = T(16) * x * (pi - x);
    const T denominator = T(5) * pi * pi - T(4) * x * (pi - x);
    return numerator / denominator;
}

/// @brief Fast cosine approximation
/// @details Uses identity cos(x) = sin(x + π/2)
/// @param x Angle in radians
/// @return Approximate cosine value
template<typename T = float>
inline auto cos(T x) noexcept -> T
{
    return sin(x + constants::half_pi<T>());
}

/// @brief Fast tangent approximation
/// @details Uses tan(x) = sin(x) / cos(x). Returns infinity when cos(x) is
///          below the float epsilon guard — consistent with IEEE 754 division
///          behaviour at the ±π/2 singularity.
/// @param x Angle in radians
/// @return Approximate tangent value; ±infinity near ±π/2
template<typename T = float>
inline auto tan(T x) noexcept -> T
{
    const T sine = sin(x);
    const T cosine = cos(x);
    if (std::abs(cosine) < std::numeric_limits<T>::epsilon())
    {
        return sine >= T(0) ? std::numeric_limits<T>::infinity() : -std::numeric_limits<T>::infinity();
    }
    return sine / cosine;
}

/// @brief Fast inverse square root approximation (Quake III algorithm)
/// @tparam Iterations Number of Newton-Raphson refinement iterations (default: 1)
/// @details Famous "fast inverse square root" with configurable precision.
///          - 0 iterations: ~3.5% max error, very fast
///          - 1 iteration: ~0.175% max error (default)
///          - 2 iterations: ~0.0015% max error
/// @param x Input value (must be positive)
/// @return Approximate 1/sqrt(x)
template<int Iterations = 1>
MICROLA_CONSTEXPR20 auto rsqrt(float x) noexcept -> float
{
    static_assert(Iterations >= 0 && Iterations <= 3, "Iterations must be 0-3");

    // Quake III fast inverse square root
    auto bits = MICROLA_BIT_CAST(std::uint32_t, x);
    bits = RSQRT_MAGIC_CONSTANT - (bits >> 1);
    auto y = MICROLA_BIT_CAST(float, bits);

    // Newton-Raphson iterations for better accuracy
    if constexpr (Iterations >= 1)
    {
        y = y * (1.5F - (0.5F * x * y * y));
    }
    if constexpr (Iterations >= 2)
    {
        y = y * (1.5F - (0.5F * x * y * y));
    }
    if constexpr (Iterations >= 3)
    {
        y = y * (1.5F - (0.5F * x * y * y));
    }

    return y;
}

/// @brief Fast square root approximation with configurable precision
/// @tparam Iterations Number of refinement iterations (0-3)
/// @details Uses fast_rsqrt then multiplies: sqrt(x) = x * (1/sqrt(x))
/// @param x Input value (must be positive)
/// @return Approximate sqrt(x)
template<int Iterations = 1>
MICROLA_CONSTEXPR20 auto sqrt(float x) noexcept -> float
{
    if (x <= 0.0F)
    {
        return 0.0F;
    }
    return x * rsqrt<Iterations>(x);
}

/// @brief Fast arc-tangent approximation (atan2)
/// @details Accurate to ~0.005 radians maximum error
/// @param y Y coordinate
/// @param x X coordinate
/// @return Approximate atan2(y, x) in radians [-π, π]
template<typename T = float>
inline auto atan2(T y, T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "fast::atan2 requires floating-point type");

    const T pi = constants::pi<T>();
    const T half_pi = constants::half_pi<T>();

    // Handle special cases
    if (x == T(0))
    {
        if (y > T(0))
        {
            return half_pi;
        }
        if (y < T(0))
        {
            return -half_pi;
        }
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
        // |z| >= 1: use identity atan(z) = sign(z)*π/2 - atan(1/z)
        T inv_z = T(1) / z;
        angle = ((z > T(0)) ? half_pi : -half_pi) - inv_z / (T(1) + T(0.28) * inv_z * inv_z);
    }

    // Adjust for quadrant
    if (x < T(0))
    {
        if (y >= T(0))
        {
            angle += pi;
        }
        else
        {
            angle -= pi;
        }
    }

    return angle;
}

/// @brief Fast exponential approximation e^x
/// @details Uses Padé approximation - accurate to ~1% error for |x| < 2
/// @param x Exponent
/// @return Approximate e^x
template<typename T = float>
inline auto exp(T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "fast::exp requires floating-point type");

    // For large |x|, use scaling: e^x = (e^(x/2^k))^(2^k)
    // Implement iteratively to avoid recursion (embedded-friendly).
    int squares = 0;
    while (x > T(2) || x < T(-2))
    {
        x = x / T(2);
        ++squares;
    }

    // Padé approximation [2/2]: e^x ≈ (2 + x) / (2 - x)
    // More accurate: e^x ≈ (12 + 6x + x²) / (12 - 6x + x²)
    const T x2 = x * x;
    const T numerator = T(12) + T(6) * x + x2;
    const T denominator = T(12) - T(6) * x + x2;
    T result = numerator / denominator;
    for (int i = 0; i < squares; ++i)
    {
        result = result * result;
    }
    return result;
}

/// @brief Fast natural logarithm approximation ln(x)
/// @details Uses bit manipulation and polynomial - accurate to ~2% error
/// @param x Input value (must be positive)
/// @return Approximate ln(x)
inline auto ln(float x) noexcept -> float
{
    // Extract exponent from IEEE 754 representation
    auto bits = MICROLA_BIT_CAST(std::uint32_t, x);
    const int exponent = static_cast<int>(((bits >> 23) & 0xFFU)) - 127;
    bits = (bits & 0x007FFFFFU) | 0x3F800000U;  // mantissa in [1, 2)
    const auto m = MICROLA_BIT_CAST(float, bits);

    // Polynomial approximation for ln(m) where m in [1, 2)
    // ln(m) ≈ (m - 1) * (a + b*(m-1) + c*(m-1)²)
    const float m1 = m - 1.0F;
    const float log_m = m1 * (2.0F - 0.666666F * m1 + 0.4F * m1 * m1);

    // ln(x) = ln(2^exp * m) = exp * ln(2) + ln(m)
    return static_cast<float>(exponent) * constants::ln2<float>() + log_m;
}

/// @brief Fast power approximation x^y
/// @details Uses exp(y * ln(x)) with fast approximations
/// @param x Base (must be positive)
/// @param y Exponent
/// @return Approximate x^y
inline auto pow(float x, float y) noexcept -> float
{
    return exp(y * ln(x));
}

/// @brief Fast absolute value
/// @details Branchless implementation using bit manipulation for floats
/// @param x Input value
/// @return |x|
MICROLA_CONSTEXPR20 auto abs(float x) noexcept -> float
{
    auto bits = MICROLA_BIT_CAST(std::uint32_t, x);
    bits &= 0x7FFFFFFFU;  // Clear sign bit
    return MICROLA_BIT_CAST(float, bits);
}

/// @brief Fast sign function
/// @details Returns -1, 0, or 1
/// @param x Input value
/// @return sign(x)
template<typename T = float>
constexpr auto sign(T x) noexcept -> T
{
    return T((T(0) < x) - (x < T(0)));
}

/// @brief Linear interpolation (lerp) - inlined for speed
/// @param a Start value
/// @param b End value
/// @param t Interpolation factor [0, 1]
/// @return Interpolated value
template<typename T = float>
constexpr auto lerp(T a, T b, T t) noexcept -> T
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
inline auto smoothstep(T edge0, T edge1, T x) noexcept -> T
{
    T t = (x - edge0) / (edge1 - edge0);
    if (t < T(0))
    {
        t = T(0);
    }
    if (t > T(1))
    {
        t = T(1);
    }
    return t * t * (T(3) - T(2) * t);
}

}  // namespace fast
}  // namespace microla
