// SPDX-License-Identifier: MIT
/// @file microla.hpp
/// @brief Main header for MicroLA - header-only linear algebra library
/// @details This library provides template-based vector and matrix classes with comprehensive
///          operator support, SIMD optimizations (CMSIS-DSP, NEON, MVE), and specialized
///          functionality for embedded systems and real-time applications.
///          This header includes all vector and matrix components.
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_HPP_
#define MICROLA_HPP_

#include "compiler_features.hpp"

#include <cstdint>
#include <type_traits>
#include <cmath>
#include <algorithm>

// Optional NEON/MVE/CMSIS blocks (disabled by default)
// Enable NEON optimizations (ARM Cortex-A, Apple Silicon, ARM64):
//   #define CONFIG_MICROLA_NEON
// Enable MVE optimizations (ARM Cortex-M with Helium):
//   #define CONFIG_MICROLA_MVE
// Enable CMSIS-DSP optimizations (ARM Cortex-M):
//   #define CONFIG_MICROLA_CMSIS

#ifdef CONFIG_MICROLA_NEON
#include <arm_neon.h>
#endif

#ifdef CONFIG_MICROLA_CMSIS
#include <arm_math.h>
#endif

#ifdef CONFIG_MICROLA_AVX
#include <immintrin.h>
#endif

// Include all component headers
#include "microla/compiler_features.hpp"
#include "microla/constants.hpp"
#include "microla/vector.hpp"
#include "microla/matrix.hpp"
#include "microla/quaternion.hpp"
#include "microla/safe_math.hpp"
#include "microla/fast_math.hpp"
#include "microla/numerical_stability.hpp"
#include "microla/resource_checks.hpp"
#include "microla/kalman.hpp"
#include "microla/extended_kalman.hpp"

namespace microla
{

// C++20 concepts for better type safety and error messages
#if __cplusplus >= 202002L
template<typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template<typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

template<typename T>
concept Integral = std::is_integral_v<T>;
#endif

/// \brief Convert degrees to radians
/// \param degrees Angle in degrees
/// \return Angle in radians
template<typename T>
MICROLA_CONSTEXPR T deg_to_rad(T degrees) noexcept
{
    return degrees * constants::deg_to_rad<T>();
}

/// \brief Convert radians to degrees
/// \param radians Angle in radians
/// \return Angle in degrees
template<typename T>
MICROLA_CONSTEXPR T rad_to_deg(T radians) noexcept
{
    return radians * constants::rad_to_deg<T>();
}

/// \brief Wrap angle to range [-pi, pi]
/// \param angle Angle in radians
/// \return Angle wrapped to [-pi, pi]
template<typename T>
MICROLA_CONSTEXPR14 T wrap_pi(T angle) noexcept
{
    // Normalize to [-pi, pi]
    while (angle > constants::pi<T>())
        angle -= constants::two_pi<T>();
    while (angle < -constants::pi<T>())
        angle += constants::two_pi<T>();
    return angle;
}

/// \brief Wrap angle to range [0, 2*pi]
/// \param angle Angle in radians
/// \return Angle wrapped to [0, 2*pi]
template<typename T>
MICROLA_CONSTEXPR14 T wrap_two_pi(T angle) noexcept
{
    // Normalize to [0, 2*pi]
    while (angle < T(0))
        angle += constants::two_pi<T>();
    while (angle >= constants::two_pi<T>())
        angle -= constants::two_pi<T>();
    return angle;
}

/// \brief Calculate shortest angular distance between two angles
/// \param from Starting angle in radians
/// \param to Ending angle in radians
/// \return Shortest angular distance in range [-pi, pi]
template<typename T>
MICROLA_CONSTEXPR14 T angle_distance(T from, T to) noexcept
{
    T diff = to - from;
    return wrap_pi(diff);
}

/// \brief Clamp value to range [min, max]
/// \param value Value to clamp
/// \param min Minimum value
/// \param max Maximum value
/// \return Clamped value
template<typename T>
MICROLA_CONSTEXPR T clamp(T value, T min, T max) noexcept
{
    return value < min ? min : (value > max ? max : value);
}

/// \brief Saturate value to range [0, 1]
/// \param value Value to saturate
/// \return Value clamped to [0, 1]
template<typename T>
MICROLA_CONSTEXPR T saturate(T value) noexcept
{
    return clamp(value, T(0), T(1));
}

// Optional NEON/MVE blocks (placeholders, disabled by default)
#ifdef CONFIG_MICROLA_NEON
// NEON implementations here
#endif

#ifdef CONFIG_MICROLA_MVE
// MVE implementations here
#endif

#ifdef CONFIG_MICROLA_CMSIS
// CMSIS-DSP implementations here

// Note: Optimizations are integrated into Vec and SquareMat methods for float types
#endif

// Note: Static asserts for trivial copyability have been removed.
// Reason: User-defined constexpr constructors (even if trivial) prevent std::is_trivially_copyable
// from being true in C++17. The classes are still efficiently copyable and have standard layout,
// but C++17's strict definition requires = default constructors for trivial copyability.
// These classes are safe for memcpy, DMA, and binary serialization despite not being formally
// "trivially copyable" according to the C++17 standard.
// See: https://en.cppreference.com/w/cpp/types/is_trivially_copyable

/// @namespace memory_info
/// @brief Compile-time memory footprint information for embedded systems
/// @details Provides constexpr functions to query memory usage of matrix/vector types.
///          Useful for validating stack allocation sizes and estimating memory budgets.
namespace memory_info
{

/// @brief Get the size in bytes of a matrix type
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @return Size in bytes including alignment padding
template<typename T, std::uint32_t R, std::uint32_t C>
MICROLA_CONSTEXPR std::size_t matrix_size_bytes() noexcept
{
    return sizeof(Mat<T, R, C>);
}

/// @brief Get the size in bytes of a vector type
/// @tparam T Element type
/// @tparam N Number of dimensions
/// @return Size in bytes including alignment padding
template<typename T, std::uint32_t N>
MICROLA_CONSTEXPR std::size_t vector_size_bytes() noexcept
{
    return sizeof(Vec<T, N>);
}

/// @brief Get the size in bytes of a quaternion type
/// @tparam T Element type
/// @return Size in bytes including alignment padding
template<typename T>
MICROLA_CONSTEXPR std::size_t quaternion_size_bytes() noexcept
{
    return sizeof(Quaternion<T>);
}

/// @brief Check if a matrix type exceeds the configured stack size limit
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @return True if matrix size exceeds MICROLA_STACK_SIZE_LIMIT
template<typename T, std::uint32_t R, std::uint32_t C>
MICROLA_CONSTEXPR bool matrix_exceeds_stack_limit() noexcept
{
    return sizeof(Mat<T, R, C>) > MICROLA_STACK_SIZE_LIMIT;
}

/// @brief Get the alignment requirement for a matrix type
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @return Alignment requirement in bytes
template<typename T, std::uint32_t R, std::uint32_t C>
MICROLA_CONSTEXPR std::size_t matrix_alignment() noexcept
{
    return alignof(Mat<T, R, C>);
}

/// @brief Compile-time assertion helper for stack size validation
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @details Use this in static_assert to ensure matrices fit within stack limits.
///          Example: static_assert(validate_matrix_stack_size<float, 4, 4>(), "Matrix too large");
template<typename T, std::uint32_t R, std::uint32_t C>
MICROLA_CONSTEXPR bool validate_matrix_stack_size() noexcept
{
    return sizeof(Mat<T, R, C>) <= MICROLA_STACK_SIZE_LIMIT;
}

}  // namespace memory_info

}  // namespace microla

#endif  // MICROLA_HPP_
