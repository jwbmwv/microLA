// SPDX-License-Identifier: MIT
/// @file safe_math.hpp
/// @brief Safe math operations with optional bounds checking for safety-critical systems
/// @details Provides runtime validation, overflow detection, and NaN checking when enabled
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_SAFE_MATH_HPP_
#define MICROLA_SAFE_MATH_HPP_

#include "compiler_features.hpp"
#include <cmath>
#include <limits>
#include <type_traits>

namespace microla
{
namespace safe
{

/// @brief Check if a floating-point value is finite (not NaN or infinity)
/// @tparam T Floating-point type
/// @param value Value to check
/// @return true if finite, false otherwise
template<typename T = float>
MICROLA_CONSTEXPR bool is_finite(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "is_finite requires floating-point type");
    return (value == value) && (value != std::numeric_limits<T>::infinity()) &&
           (value != -std::numeric_limits<T>::infinity());
}

/// @brief Check if a floating-point value is NaN
/// @tparam T Floating-point type
/// @param value Value to check
/// @return true if NaN, false otherwise
template<typename T = float>
MICROLA_CONSTEXPR bool is_nan(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "is_nan requires floating-point type");
    return value != value;
}

/// @brief Safe division with zero-check
/// @tparam T Arithmetic type
/// @param numerator Numerator
/// @param denominator Denominator
/// @param default_value Value to return if denominator is zero (default: 0)
/// @return numerator / denominator, or default_value if denominator is zero
template<typename T>
MICROLA_CONSTEXPR T safe_divide(T numerator, T denominator, T default_value = T(0)) noexcept
{
    static_assert(std::is_arithmetic<T>::value, "safe_divide requires arithmetic type");
    MICROLA_IF_CONSTEXPR(std::is_floating_point<T>::value)
    {
        if (std::abs(denominator) < std::numeric_limits<T>::epsilon())
        {
            return default_value;
        }
    }
    else
    {
        if (denominator == T(0))
        {
            return default_value;
        }
    }
    return numerator / denominator;
}

/// @brief Safe reciprocal (1/x) with zero-check
/// @tparam T Floating-point type
/// @param value Input value
/// @param default_value Value to return if input is zero (default: 0)
/// @return 1/value, or default_value if value is near zero
template<typename T>
MICROLA_CONSTEXPR T safe_reciprocal(T value, T default_value = T(0)) noexcept
{
    static_assert(std::is_floating_point<T>::value, "safe_reciprocal requires floating-point type");
    return safe_divide(T(1), value, default_value);
}

/// @brief Safe square root with negative check
/// @tparam T Floating-point type
/// @param value Input value
/// @return sqrt(value) if value >= 0, otherwise 0
template<typename T>
inline T safe_sqrt(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "safe_sqrt requires floating-point type");
    return (value >= T(0)) ? std::sqrt(value) : T(0);
}

/// @brief Safe arc-cosine with clamping to valid range [-1, 1]
/// @tparam T Floating-point type
/// @param value Input value
/// @return acos(clamp(value, -1, 1))
template<typename T>
inline T safe_acos(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "safe_acos requires floating-point type");
    if (value <= T(-1))
        return T(3.14159265358979323846);  // pi
    if (value >= T(1))
        return T(0);
    return std::acos(value);
}

/// @brief Safe arc-sine with clamping to valid range [-1, 1]
/// @tparam T Floating-point type
/// @param value Input value
/// @return asin(clamp(value, -1, 1))
template<typename T>
inline T safe_asin(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "safe_asin requires floating-point type");
    if (value <= T(-1))
        return T(-1.57079632679489661923);  // -pi/2
    if (value >= T(1))
        return T(1.57079632679489661923);  // pi/2
    return std::asin(value);
}

#ifdef MICROLA_SAFE_MODE

/// @brief Check if array index is within bounds (only in safe mode)
/// @param index Index to check
/// @param size Array size
/// @return true if valid, false otherwise
inline bool check_bounds(std::uint32_t index, std::uint32_t size) noexcept
{
    return index < size;
}

/// @brief Assert that value is finite (only in safe mode)
/// @tparam T Floating-point type
/// @param value Value to check
/// @return The value if finite, 0 otherwise
template<typename T>
inline T assert_finite(T value) noexcept
{
    if (!is_finite(value))
    {
        return T(0);
    }
    return value;
}

/// @brief Validate matrix condition number before inversion (only in safe mode)
/// @tparam T Floating-point type
/// @param condition_number Condition number of matrix
/// @param threshold Maximum acceptable condition number (default: 1e6)
/// @return true if matrix is well-conditioned
template<typename T>
bool is_well_conditioned(T condition_number, T threshold = T(1e6)) noexcept
{
    return condition_number < threshold && is_finite(condition_number);
}

#else

// No-op implementations when not in safe mode
inline constexpr bool check_bounds(std::uint32_t, std::uint32_t) noexcept
{
    return true;
}

template<typename T>
MICROLA_CONSTEXPR T assert_finite(T value) noexcept
{
    return value;
}

template<typename T>
constexpr bool is_well_conditioned(T, T = T(1e6)) noexcept
{
    return true;
}

#endif  // MICROLA_SAFE_MODE

/// @brief Saturating addition (clamps to type limits instead of wrapping)
/// @tparam T Integral type
/// @param a First operand
/// @param b Second operand
/// @return Saturated sum clamped to [min, max] of type T
template<typename T>
MICROLA_CONSTEXPR T saturating_add(T a, T b) noexcept
{
    static_assert(std::is_integral<T>::value, "saturating_add requires integral type");
    MICROLA_IF_CONSTEXPR(std::is_signed<T>::value)
    {
        if (b > 0 && a > std::numeric_limits<T>::max() - b)
            return std::numeric_limits<T>::max();
        if (b < 0 && a < std::numeric_limits<T>::min() - b)
            return std::numeric_limits<T>::min();
    }
    else
    {
        if (a > std::numeric_limits<T>::max() - b)
            return std::numeric_limits<T>::max();
    }
    return a + b;
}

/// @brief Saturating subtraction (clamps to type limits instead of wrapping)
/// @tparam T Integral type
/// @param a First operand
/// @param b Second operand
/// @return Saturated difference clamped to [min, max] of type T
template<typename T>
MICROLA_CONSTEXPR T saturating_sub(T a, T b) noexcept
{
    static_assert(std::is_integral<T>::value, "saturating_sub requires integral type");
    MICROLA_IF_CONSTEXPR(std::is_signed<T>::value)
    {
        if (b < 0 && a > std::numeric_limits<T>::max() + b)
            return std::numeric_limits<T>::max();
        if (b > 0 && a < std::numeric_limits<T>::min() + b)
            return std::numeric_limits<T>::min();
    }
    else
    {
        if (a < b)
            return T(0);
    }
    return a - b;
}

/// @brief Saturating multiplication (clamps to type limits instead of wrapping)
/// @tparam T Integral type
/// @param a First operand
/// @param b Second operand
/// @return Saturated product clamped to [min, max] of type T
template<typename T>
MICROLA_CONSTEXPR T saturating_mul(T a, T b) noexcept
{
    static_assert(std::is_integral<T>::value, "saturating_mul requires integral type");
    if (a == 0 || b == 0)
        return 0;

    MICROLA_IF_CONSTEXPR(std::is_signed<T>::value)
    {
        if (a > 0)
        {
            if (b > 0 && a > std::numeric_limits<T>::max() / b)
                return std::numeric_limits<T>::max();
            if (b < 0 && b < std::numeric_limits<T>::min() / a)
                return std::numeric_limits<T>::min();
        }
        else
        {
            if (b > 0 && a < std::numeric_limits<T>::min() / b)
                return std::numeric_limits<T>::min();
            if (b < 0 && a < std::numeric_limits<T>::max() / b)
                return std::numeric_limits<T>::max();
        }
    }
    else
    {
        if (a > std::numeric_limits<T>::max() / b)
            return std::numeric_limits<T>::max();
    }
    return a * b;
}

}  // namespace safe
}  // namespace microla

#endif  // MICROLA_SAFE_MATH_HPP_
