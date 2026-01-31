// SPDX-License-Identifier: MIT
/// @file constants.hpp
/// @brief Mathematical constants with C++11-C++26 optimizations
/// @details Provides compile-time mathematical constants for various numeric types.
///          This header is independent and can be used by any component.
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef _MICROLA_CONSTANTS_HPP_
#define _MICROLA_CONSTANTS_HPP_

#include <cstdint>
#include <limits>
#include <type_traits>

#include "compiler_features.hpp"

namespace microla
{

/// @namespace constants
/// @brief Mathematical constants for common calculations
/// @details In C++11, constants are accessed as functions: constants::pi<T>().
///          In C++14+, variable-template compatibility is provided so
///          constants::pi<T> also works (and remains callable via operator()).
namespace constants
{
namespace detail
{
template<typename T>
MICROLA_CONSTEXPR T pi_value() noexcept
{
    return T(3.14159265358979323846);
}

template<typename T>
MICROLA_CONSTEXPR T two_pi_value() noexcept
{
    return T(6.28318530717958647692);
}

template<typename T>
MICROLA_CONSTEXPR T half_pi_value() noexcept
{
    return T(1.57079632679489661923);
}

template<typename T>
MICROLA_CONSTEXPR T quarter_pi_value() noexcept
{
    return T(0.78539816339744830962);
}

template<typename T>
MICROLA_CONSTEXPR T e_value() noexcept
{
    return T(2.71828182845904523536);
}

template<typename T>
MICROLA_CONSTEXPR T golden_ratio_value() noexcept
{
    return T(1.61803398874989484820);
}

template<typename T>
MICROLA_CONSTEXPR T sqrt2_value() noexcept
{
    return T(1.41421356237309504880);
}

template<typename T>
MICROLA_CONSTEXPR T sqrt3_value() noexcept
{
    return T(1.73205080756887729352);
}

template<typename T>
MICROLA_CONSTEXPR T ln2_value() noexcept
{
    return T(0.69314718055994530942);
}

template<typename T>
MICROLA_CONSTEXPR T ln10_value() noexcept
{
    return T(2.30258509299404568402);
}

inline MICROLA_CONSTEXPR float epsilon_f_value() noexcept
{
    return std::numeric_limits<float>::epsilon() * 8.0f;  // ≈ 9.5e-7
}

inline MICROLA_CONSTEXPR double epsilon_d_value() noexcept
{
    return std::numeric_limits<double>::epsilon() * 4.5e6;  // ≈ 1e-9 (relaxed from 1e-12)
}

template<typename T>
MICROLA_CONSTEXPR T epsilon_value() noexcept
{
    return std::numeric_limits<T>::epsilon() * T(100);
}

template<typename T>
MICROLA_CONSTEXPR T deg_to_rad_value() noexcept
{
    return T(0.01745329251994329577);
}

template<typename T>
MICROLA_CONSTEXPR T rad_to_deg_value() noexcept
{
    return T(57.29577951308232087680);
}

template<typename T>
MICROLA_CONSTEXPR T gravity_value() noexcept
{
    return T(9.80665);
}

template<typename T>
MICROLA_CONSTEXPR T speed_of_light_value() noexcept
{
    return T(299792458.0);
}

#if __cplusplus >= 201402L
template<typename T, T (*Fn)()>
struct constant_proxy
{
    /// @brief Invoke the constant as a function.
    /// @return Constant value.
    MICROLA_CONSTEXPR T operator()() const noexcept { return Fn(); }
    /// @brief Implicit conversion to the constant value.
    MICROLA_CONSTEXPR operator T() const noexcept { return Fn(); }
};
#endif
}  // namespace detail

#if __cplusplus >= 201402L
/// @brief Pi (π) - Ratio of circle's circumference to diameter
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::pi_value<T>> pi{};

/// @brief Two times Pi (2π)
/// @tparam T Numeric type (default: double)
/// @details Used for full circle calculations (360 degrees = 2π radians)
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::two_pi_value<T>> two_pi{};

/// @brief Half Pi (π/2)
/// @tparam T Numeric type (default: double)
/// @details 90 degrees in radians
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::half_pi_value<T>> half_pi{};

/// @brief Quarter Pi (π/4)
/// @tparam T Numeric type (default: double)
/// @details 45 degrees in radians
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::quarter_pi_value<T>> quarter_pi{};

/// @brief Euler's number (e) - Base of natural logarithms
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::e_value<T>> e{};

/// @brief Golden ratio (φ) - (1 + √5) / 2
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::golden_ratio_value<T>> golden_ratio{};

/// @brief Square root of 2 (√2)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::sqrt2_value<T>> sqrt2{};

/// @brief Square root of 3 (√3)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::sqrt3_value<T>> sqrt3{};

/// @brief Natural logarithm of 2 (ln(2))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::ln2_value<T>> ln2{};

/// @brief Natural logarithm of 10 (ln(10))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::ln10_value<T>> ln10{};

/// @brief Default epsilon for floating-point comparisons (float)
/// @details Scaled machine epsilon (~8x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_f()
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<float, detail::epsilon_f_value> epsilon_f{};

/// @brief Default epsilon for floating-point comparisons (double)
/// @details Scaled machine epsilon (~4.5e6x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_d()
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<double, detail::epsilon_d_value> epsilon_d{};

/// @brief Default epsilon for floating-point comparisons (generic)
/// @tparam T Numeric type (default: double)
/// @details Scaled machine epsilon for practical comparison tolerance.
///          For float: ~8x machine epsilon, for double: ~100x machine epsilon
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::epsilon_value<T>> epsilon{};

// ==================== Conversion Factors ====================

/// @brief Degrees to radians multiplier (π/180)
/// @tparam T Numeric type (default: double)
/// @details Multiply degrees by this to get radians
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::deg_to_rad_value<T>> deg_to_rad{};

/// @brief Radians to degrees multiplier (180/π)
/// @tparam T Numeric type (default: double)
/// @details Multiply radians by this to get degrees
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::rad_to_deg_value<T>> rad_to_deg{};

// ==================== Physical Constants (Optional - Commonly Used in Embedded) ====================

/// @brief Gravity acceleration at Earth's surface (m/s²)
/// @tparam T Numeric type (default: double)
/// @details Standard gravity: 9.80665 m/s²
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::gravity_value<T>> gravity{};

/// @brief Speed of light in vacuum (m/s)
/// @tparam T Numeric type (default: double)
/// @details Exact value: 299,792,458 m/s
template<typename T = double>
MICROLA_INLINE_VAR MICROLA_CONSTEXPR detail::constant_proxy<T, detail::speed_of_light_value<T>> speed_of_light{};

#else
/// @brief Pi (π) - Ratio of circle's circumference to diameter
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T pi() noexcept
{
    return detail::pi_value<T>();
}

/// @brief Two times Pi (2π)
/// @tparam T Numeric type (default: double)
/// @details Used for full circle calculations (360 degrees = 2π radians)
template<typename T = double>
MICROLA_CONSTEXPR T two_pi() noexcept
{
    return detail::two_pi_value<T>();
}

/// @brief Half Pi (π/2)
/// @tparam T Numeric type (default: double)
/// @details 90 degrees in radians
template<typename T = double>
MICROLA_CONSTEXPR T half_pi() noexcept
{
    return detail::half_pi_value<T>();
}

/// @brief Quarter Pi (π/4)
/// @tparam T Numeric type (default: double)
/// @details 45 degrees in radians
template<typename T = double>
MICROLA_CONSTEXPR T quarter_pi() noexcept
{
    return detail::quarter_pi_value<T>();
}

/// @brief Euler's number (e) - Base of natural logarithms
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T e() noexcept
{
    return detail::e_value<T>();
}

/// @brief Golden ratio (φ) - (1 + √5) / 2
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T golden_ratio() noexcept
{
    return detail::golden_ratio_value<T>();
}

/// @brief Square root of 2 (√2)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T sqrt2() noexcept
{
    return detail::sqrt2_value<T>();
}

/// @brief Square root of 3 (√3)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T sqrt3() noexcept
{
    return detail::sqrt3_value<T>();
}

/// @brief Natural logarithm of 2 (ln(2))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T ln2() noexcept
{
    return detail::ln2_value<T>();
}

/// @brief Natural logarithm of 10 (ln(10))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
MICROLA_CONSTEXPR T ln10() noexcept
{
    return detail::ln10_value<T>();
}

/// @brief Default epsilon for floating-point comparisons (float)
/// @details Scaled machine epsilon (~8x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_f()
inline MICROLA_CONSTEXPR float epsilon_f() noexcept
{
    return detail::epsilon_f_value();
}

/// @brief Default epsilon for floating-point comparisons (double)
/// @details Scaled machine epsilon (~4.5e6x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_d()
inline MICROLA_CONSTEXPR double epsilon_d() noexcept
{
    return detail::epsilon_d_value();
}

/// @brief Default epsilon for floating-point comparisons (generic)
/// @tparam T Numeric type (default: double)
/// @details Scaled machine epsilon for practical comparison tolerance.
///          For float: ~8x machine epsilon, for double: ~100x machine epsilon
template<typename T = double>
MICROLA_CONSTEXPR T epsilon() noexcept
{
    return detail::epsilon_value<T>();
}

// ==================== Conversion Factors ====================

/// @brief Degrees to radians multiplier (π/180)
/// @tparam T Numeric type (default: double)
/// @details Multiply degrees by this to get radians
template<typename T = double>
MICROLA_CONSTEXPR T deg_to_rad() noexcept
{
    return detail::deg_to_rad_value<T>();
}

/// @brief Radians to degrees multiplier (180/π)
/// @tparam T Numeric type (default: double)
/// @details Multiply radians by this to get degrees
template<typename T = double>
MICROLA_CONSTEXPR T rad_to_deg() noexcept
{
    return detail::rad_to_deg_value<T>();
}

// ==================== Physical Constants (Optional - Commonly Used in Embedded) ====================

/// @brief Gravity acceleration at Earth's surface (m/s²)
/// @tparam T Numeric type (default: double)
/// @details Standard gravity: 9.80665 m/s²
template<typename T = double>
MICROLA_CONSTEXPR T gravity() noexcept
{
    return detail::gravity_value<T>();
}

/// @brief Speed of light in vacuum (m/s)
/// @tparam T Numeric type (default: double)
/// @details Exact value: 299,792,458 m/s
template<typename T = double>
MICROLA_CONSTEXPR T speed_of_light() noexcept
{
    return detail::speed_of_light_value<T>();
}
#endif

}  // namespace constants

}  // namespace microla

#endif  // _MICROLA_CONSTANTS_HPP_
