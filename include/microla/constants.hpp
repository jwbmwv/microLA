// SPDX-License-Identifier: Apache-2.0
/// @file constants.hpp
/// @brief Mathematical constants with C++20-C++26 optimizations
/// @details Provides compile-time mathematical constants for various numeric types.
///          This header is independent and can be used by any component.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include <cstdint>
#include <limits>
#include <numbers>
#include <type_traits>

#include "compiler_features.hpp"

/// @namespace constants
/// @brief Mathematical constants for common calculations
/// @details Constants are exposed as inline variable templates and remain
///          callable via proxy operator(), so both constants::pi<T> and
///          constants::pi<T>() are supported.
namespace microla
{
namespace constants
{
namespace detail
{
template<typename T>
constexpr auto pi_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::pi_v<T>;
    }
    return static_cast<T>(std::numbers::pi_v<long double>);
}

template<typename T>
constexpr auto two_pi_value() noexcept -> T
{
    return T(6.28318530717958647692);
}

template<typename T>
constexpr auto half_pi_value() noexcept -> T
{
    return T(1.57079632679489661923);
}

template<typename T>
constexpr auto quarter_pi_value() noexcept -> T
{
    return T(0.78539816339744830962);
}

template<typename T>
constexpr auto e_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::e_v<T>;
    }
    return static_cast<T>(std::numbers::e_v<long double>);
}

template<typename T>
constexpr auto golden_ratio_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::phi_v<T>;
    }
    return static_cast<T>(std::numbers::phi_v<long double>);
}

template<typename T>
constexpr auto sqrt2_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::sqrt2_v<T>;
    }
    return static_cast<T>(std::numbers::sqrt2_v<long double>);
}

template<typename T>
constexpr auto sqrt3_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::sqrt3_v<T>;
    }
    return static_cast<T>(std::numbers::sqrt3_v<long double>);
}

template<typename T>
constexpr auto ln2_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::ln2_v<T>;
    }
    return static_cast<T>(std::numbers::ln2_v<long double>);
}

template<typename T>
constexpr auto ln10_value() noexcept -> T
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::numbers::ln10_v<T>;
    }
    return static_cast<T>(std::numbers::ln10_v<long double>);
}

constexpr auto epsilon_f_value() noexcept -> float
{
    return std::numeric_limits<float>::epsilon() * 8.0F;  // ≈ 9.5e-7
}

constexpr auto epsilon_d_value() noexcept -> double
{
    return std::numeric_limits<double>::epsilon() * 4.5e6;  // ≈ 1e-9 (relaxed from 1e-12)
}

template<typename T>
constexpr auto epsilon_value() noexcept -> T
{
    return std::numeric_limits<T>::epsilon() * T(100);
}

template<typename T>
constexpr auto deg_to_rad_value() noexcept -> T
{
    return T(0.01745329251994329577);
}

template<typename T>
constexpr auto rad_to_deg_value() noexcept -> T
{
    return T(57.29577951308232087680);
}

template<typename T>
constexpr auto gravity_value() noexcept -> T
{
    return T(9.80665);
}

template<typename T>
constexpr auto speed_of_light_value() noexcept -> T
{
    return T(299792458.0);
}

template<typename T, T (*Fn)()>
struct ConstantProxy
{
    /// @brief Invoke the constant as a function.
    /// @return Constant value.
    constexpr auto operator()() const noexcept -> T { return Fn(); }
    /// @brief Implicit conversion to the constant value.
    constexpr operator T() const noexcept { return Fn(); }  // NOLINT(hicpp-explicit-conversions)
};
}  // namespace detail

/// @brief Pi (π) - Ratio of circle's circumference to diameter
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::pi_value<T>> pi{};

/// @brief Two times Pi (2π)
/// @tparam T Numeric type (default: double)
/// @details Used for full circle calculations (360 degrees = 2π radians)
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::two_pi_value<T>> two_pi{};

/// @brief Half Pi (π/2)
/// @tparam T Numeric type (default: double)
/// @details 90 degrees in radians
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::half_pi_value<T>> half_pi{};

/// @brief Quarter Pi (π/4)
/// @tparam T Numeric type (default: double)
/// @details 45 degrees in radians
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::quarter_pi_value<T>> quarter_pi{};

/// @brief Euler's number (e) - Base of natural logarithms
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::e_value<T>> e{};

/// @brief Golden ratio (φ) - (1 + √5) / 2
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::golden_ratio_value<T>> golden_ratio{};

/// @brief Square root of 2 (√2)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::sqrt2_value<T>> sqrt2{};

/// @brief Square root of 3 (√3)
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::sqrt3_value<T>> sqrt3{};

/// @brief Natural logarithm of 2 (ln(2))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::ln2_value<T>> ln2{};

/// @brief Natural logarithm of 10 (ln(10))
/// @tparam T Numeric type (default: double)
/// @details Precision: 20 decimal places
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::ln10_value<T>> ln10{};

/// @brief Default epsilon for floating-point comparisons (float)
/// @details Scaled machine epsilon (~8x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_f()
inline constexpr detail::ConstantProxy<float, detail::epsilon_f_value> epsilon_f{};

/// @brief Default epsilon for floating-point comparisons (double)
/// @details Scaled machine epsilon (~4.5e6x) for practical comparison tolerance.
///          Use for approximate equality: abs(a - b) < epsilon_d()
inline constexpr detail::ConstantProxy<double, detail::epsilon_d_value> epsilon_d{};

/// @brief Default epsilon for floating-point comparisons (generic)
/// @tparam T Numeric type (default: double)
/// @details Scaled machine epsilon for practical comparison tolerance.
///          For float: ~8x machine epsilon, for double: ~100x machine epsilon
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::epsilon_value<T>> epsilon{};

// ==================== Conversion Factors ====================

/// @brief Degrees to radians multiplier (π/180)
/// @tparam T Numeric type (default: double)
/// @details Multiply degrees by this to get radians
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::deg_to_rad_value<T>> deg_to_rad{};

/// @brief Radians to degrees multiplier (180/π)
/// @tparam T Numeric type (default: double)
/// @details Multiply radians by this to get degrees
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::rad_to_deg_value<T>> rad_to_deg{};

// ==================== Physical Constants (Optional - Commonly Used in Embedded) ====================

/// @brief Gravity acceleration at Earth's surface (m/s²)
/// @tparam T Numeric type (default: double)
/// @details Standard gravity: 9.80665 m/s²
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::gravity_value<T>> gravity{};

/// @brief Speed of light in vacuum (m/s)
/// @tparam T Numeric type (default: double)
/// @details Exact value: 299,792,458 m/s
template<typename T = double>
inline constexpr detail::ConstantProxy<T, detail::speed_of_light_value<T>> speed_of_light{};

}  // namespace constants
}  // namespace microla
