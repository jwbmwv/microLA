// SPDX-License-Identifier: Apache-2.0
/// @file safe_math.hpp
/// @brief Safe math operations with optional bounds checking for safety-critical systems
/// @details Provides runtime validation, overflow detection, and NaN checking when enabled
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "compiler_features.hpp"
#include <cmath>
#include <limits>
#include <type_traits>
#include <cstdint>
#include <cstring>

namespace microla
{
namespace safe
{

/// @brief Check if a floating-point value is finite (not NaN or infinity)
/// @tparam T Floating-point type
/// @param value Value to check
/// @return true if finite, false otherwise
template<typename T = float>
inline auto is_finite(T value) noexcept -> bool
{
    static_assert(std::is_floating_point_v<T>, "is_finite requires floating-point type");
    static_assert(sizeof(T) == 4 || sizeof(T) == 8, "is_finite supports 32-bit and 64-bit floating-point types");
    using Bits = typename std::conditional<sizeof(T) == 4, std::uint32_t, std::uint64_t>::type;
    Bits bits = 0;
    std::memcpy(&bits, &value, sizeof(Bits));
    if constexpr (sizeof(T) == 4)
    {
        constexpr std::uint32_t exp_mask32 = 0x7F800000U;
        return (bits & static_cast<Bits>(exp_mask32)) != static_cast<Bits>(exp_mask32);
    }
    else if constexpr (sizeof(T) == 8)
    {
        constexpr std::uint64_t exp_mask64 = 0x7FF0000000000000ULL;
        return (bits & static_cast<Bits>(exp_mask64)) != static_cast<Bits>(exp_mask64);
    }

    MICROLA_UNREACHABLE();
    return false;
}

/// @brief Check if a floating-point value is NaN
/// @tparam T Floating-point type
/// @param value Value to check
/// @return true if NaN, false otherwise
template<typename T = float>
inline auto is_nan(T value) noexcept -> bool
{
    static_assert(std::is_floating_point_v<T>, "is_nan requires floating-point type");
    static_assert(sizeof(T) == 4 || sizeof(T) == 8, "is_nan supports 32-bit and 64-bit floating-point types");
    using Bits = typename std::conditional<sizeof(T) == 4, std::uint32_t, std::uint64_t>::type;
    Bits bits = 0;
    std::memcpy(&bits, &value, sizeof(Bits));
    if constexpr (sizeof(T) == 4)
    {
        constexpr std::uint32_t exp_mask32 = 0x7F800000U;
        constexpr std::uint32_t frac_mask32 = 0x007FFFFFU;
        return (bits & static_cast<Bits>(exp_mask32)) == static_cast<Bits>(exp_mask32) &&
               (bits & static_cast<Bits>(frac_mask32)) != static_cast<Bits>(0U);
    }
    else if constexpr (sizeof(T) == 8)
    {
        constexpr std::uint64_t exp_mask64 = 0x7FF0000000000000ULL;
        constexpr std::uint64_t frac_mask64 = 0x000FFFFFFFFFFFFFULL;
        return (bits & static_cast<Bits>(exp_mask64)) == static_cast<Bits>(exp_mask64) &&
               (bits & static_cast<Bits>(frac_mask64)) != static_cast<Bits>(0ULL);
    }

    MICROLA_UNREACHABLE();
    return false;
}

/// @brief Safe division with zero-check
/// @tparam T Arithmetic type
/// @param numerator Numerator
/// @param denominator Denominator
/// @param default_value Value to return if denominator is zero (default: 0)
/// @return numerator / denominator, or default_value if denominator is zero
template<typename T>
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4723)  // potential divide by 0 - false positives for template branches
#endif
constexpr auto safe_divide(T numerator, T denominator, T default_value = T(0)) noexcept -> T
{
    static_assert(std::is_arithmetic_v<T>, "safe_divide requires arithmetic type");
    if constexpr (std::is_floating_point_v<T>)
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
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

/// @brief Safe reciprocal (1/x) with zero-check
/// @tparam T Floating-point type
/// @param value Input value
/// @param default_value Value to return if input is zero (default: 0)
/// @return 1/value, or default_value if value is near zero
template<typename T>
constexpr auto safe_reciprocal(T value, T default_value = T(0)) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "safe_reciprocal requires floating-point type");
    return safe_divide(T(1), value, default_value);
}

/// @brief Safe square root with negative check
/// @tparam T Floating-point type
/// @param value Input value
/// @return sqrt(value) if value >= 0, otherwise 0
template<typename T>
inline auto safe_sqrt(T value) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "safe_sqrt requires floating-point type");
    // Normalize to a non-negative value before calling std::sqrt so static analyzers
    // can clearly see we never pass a negative value to std::sqrt.
    T v = (value < T(0)) ? T(0) : value;
    return std::sqrt(v);
}

/// @brief Safe arc-cosine with clamping to valid range [-1, 1]
/// @tparam T Floating-point type
/// @param value Input value
/// @return acos(clamp(value, -1, 1))
template<typename T>
inline auto safe_acos(T value) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "safe_acos requires floating-point type");
    if (value <= T(-1))
    {
        return T(3.14159265358979323846);  // pi
    }
    if (value >= T(1))
    {
        return T(0);
    }
    return std::acos(value);
}

/// @brief Safe arc-sine with clamping to valid range [-1, 1]
/// @tparam T Floating-point type
/// @param value Input value
/// @return asin(clamp(value, -1, 1))
template<typename T>
inline auto safe_asin(T value) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "safe_asin requires floating-point type");
    if (value <= T(-1))
    {
        return T(-1.57079632679489661923);  // -pi/2
    }
    if (value >= T(1))
    {
        return T(1.57079632679489661923);  // pi/2
    }
    return std::asin(value);
}

#ifdef MICROLA_SAFE_MODE

/// @brief Check if array index is within bounds (only in safe mode)
/// @param index Index to check
/// @param size Array size
/// @return true if valid, false otherwise
inline bool check_bounds(std::size_t index, std::size_t size) noexcept
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
constexpr auto check_bounds(std::size_t /*unused*/, std::size_t /*unused*/) noexcept -> bool
{
    return true;
}

template<typename T>
constexpr auto assert_finite(T value) noexcept -> T
{
    return value;
}

template<typename T>
constexpr auto is_well_conditioned(T /*unused*/, T /*unused*/ = T(1e6)) noexcept -> bool
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
constexpr auto saturating_add(T a, T b) noexcept -> T
{
    static_assert(std::is_integral_v<T>, "saturating_add requires integral type");
    if constexpr (std::is_signed_v<T>)
    {
        if (b > 0 && a > std::numeric_limits<T>::max() - b)
        {
            return std::numeric_limits<T>::max();
        }
        if (b < 0 && a < std::numeric_limits<T>::min() - b)
        {
            return std::numeric_limits<T>::min();
        }
    }
    else
    {
        if (a > std::numeric_limits<T>::max() - b)
        {
            return std::numeric_limits<T>::max();
        }
    }
    return a + b;
}

/// @brief Saturating subtraction (clamps to type limits instead of wrapping)
/// @tparam T Integral type
/// @param a First operand
/// @param b Second operand
/// @return Saturated difference clamped to [min, max] of type T
template<typename T>
constexpr auto saturating_sub(T a, T b) noexcept -> T
{
    static_assert(std::is_integral_v<T>, "saturating_sub requires integral type");
    if constexpr (std::is_signed_v<T>)
    {
        if (b < 0 && a > std::numeric_limits<T>::max() + b)
        {
            return std::numeric_limits<T>::max();
        }
        if (b > 0 && a < std::numeric_limits<T>::min() + b)
        {
            return std::numeric_limits<T>::min();
        }
    }
    else
    {
        if (a < b)
        {
            return T(0);
        }
    }
    return a - b;
}

/// @brief Saturating multiplication (clamps to type limits instead of wrapping)
/// @tparam T Integral type
/// @param a First operand
/// @param b Second operand
/// @return Saturated product clamped to [min, max] of type T
template<typename T>
constexpr auto saturating_mul(T a, T b) noexcept -> T
{
    static_assert(std::is_integral_v<T>, "saturating_mul requires integral type");
    if (a == 0 || b == 0)
    {
        return 0;
    }

    if constexpr (std::is_signed_v<T>)
    {
        if (a > 0)
        {
            if (b > 0 && a > std::numeric_limits<T>::max() / b)
            {
                return std::numeric_limits<T>::max();
            }
            if (b < 0 && b < std::numeric_limits<T>::min() / a)
            {
                return std::numeric_limits<T>::min();
            }
        }
        else
        {
            if (b > 0 && a < std::numeric_limits<T>::min() / b)
            {
                return std::numeric_limits<T>::min();
            }
            if (b < 0 && a < std::numeric_limits<T>::max() / b)
            {
                return std::numeric_limits<T>::max();
            }
        }
    }
    else
    {
        if (a > std::numeric_limits<T>::max() / b)
        {
            return std::numeric_limits<T>::max();
        }
    }
    return a * b;
}

}  // namespace safe
}  // namespace microla
