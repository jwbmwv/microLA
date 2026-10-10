// SPDX-License-Identifier: Apache-2.0
/// @file microla.hpp
/// @brief Main header for MicroLA - header-only linear algebra library
/// @details This library provides template-based vector and matrix classes with comprehensive
///          operator support, SIMD optimizations (CMSIS-DSP, NEON, MVE), and specialized
///          functionality for embedded systems and real-time applications.
///          This header includes all vector and matrix components.
///
/// @section standards Standards and Conventions
/// - **Matrix Storage**: Row-major order (C-style: data[row * cols + col])
/// - **Quaternion Convention**: Hamilton (q = w + xi + yj + zk), memory layout [x,y,z,w]
/// - **Quaternion Multiplication**: Right-to-left (q1 * q2 applies q2 then q1)
/// - **Euler Angles**: Intrinsic rotations, order XYZ (roll-pitch-yaw)
/// - **Rotation Direction**: Right-hand rule (positive = counter-clockwise)
/// - **Coordinate System**: Right-handed (x=right, y=up, z=forward)
/// - **Homogeneous Coords**: [x, y, z, w] where w=1 for points, w=0 for vectors
/// - **Thread Safety**: Classes are thread-safe for read-only operations;
///                      concurrent writes require external synchronization
///
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

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
#include "microla/constants.hpp"
#include "microla/vector.hpp"
#include "microla/matrix.hpp"
#include "microla/matrix_view.hpp"
#include "microla/vector_view.hpp"
#include "microla/quaternion.hpp"
#include "microla/safe_math.hpp"
#include "microla/fast_math.hpp"
#include "microla/numerical_stability.hpp"
#include "microla/resource_checks.hpp"
#include "microla/kalman.hpp"
#include "microla/extended_kalman.hpp"
#include "microla/sensor_fusion.hpp"

namespace microla
{

// C++20 concepts for better type safety and error messages
/// @brief Accept built-in arithmetic types.
/// @tparam T Type tested for arithmetic classification.
template<typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

/// @brief Accept built-in floating-point types.
/// @tparam T Type tested for floating-point classification.
template<typename T>
concept FloatingPoint = std::is_floating_point_v<T>;

/// @brief Accept built-in integral types.
/// @tparam T Type tested for integral classification.
template<typename T>
concept Integral = std::is_integral_v<T>;

/// \brief Convert degrees to radians
/// @tparam T Numeric type of the angle and result.
/// \param degrees Angle in degrees
/// \return Angle in radians
template<typename T>
[[nodiscard]] constexpr auto deg_to_rad(T degrees) noexcept -> T
{
    return degrees * constants::deg_to_rad<T>();
}

/// \brief Convert radians to degrees
/// @tparam T Numeric type of the angle and result.
/// \param radians Angle in radians
/// \return Angle in degrees
template<typename T>
[[nodiscard]] constexpr auto rad_to_deg(T radians) noexcept -> T
{
    return radians * constants::rad_to_deg<T>();
}

namespace detail
{
/// @brief Reduce an angle modulo one revolution without an integral quotient cast.
/// @tparam T Numeric type of the angle and result.
/// @param angle Angle in radians.
/// @return Signed remainder modulo two pi; non-finite floating-point inputs produce NaN.
template<typename T>
constexpr auto angle_remainder(T angle) noexcept -> T
{
    if constexpr (std::is_integral_v<T>)
    {
        return angle % constants::two_pi<T>();
    }
    if (!(angle <= std::numeric_limits<T>::max() && angle >= -std::numeric_limits<T>::max()))
    {
        return std::numeric_limits<T>::quiet_NaN();
    }
    if (!std::is_constant_evaluated())
    {
        return std::fmod(angle, constants::two_pi<T>());
    }
    const bool negative = angle < T(0);
    T remainder = negative ? -angle : angle;
    T divisor = constants::two_pi<T>();
    for (int step = 0; step < std::numeric_limits<T>::max_exponent && divisor <= remainder / T(2); ++step)
    {
        divisor *= T(2);
    }
    for (int step = 0; step <= std::numeric_limits<T>::max_exponent && divisor >= constants::two_pi<T>(); ++step)
    {
        if (remainder >= divisor)
        {
            remainder -= divisor;
        }
        divisor /= T(2);
    }
    return negative ? -remainder : remainder;
}
}  // namespace detail

/// \brief Wrap angle to range [-pi, pi]
/// @tparam T Numeric type of the angle and result.
/// \param angle Angle in radians
/// \return Angle wrapped to [-pi, pi]
template<typename T>
[[nodiscard]] constexpr auto wrap_pi(T angle) noexcept -> T
{
    angle = detail::angle_remainder(angle);
    if (angle > constants::pi<T>())
    {
        angle -= constants::two_pi<T>();
    }
    if (angle < -constants::pi<T>())
    {
        angle += constants::two_pi<T>();
    }
    return angle;
}

/// \brief Wrap angle to range [0, 2*pi]
/// @tparam T Numeric type of the angle and result.
/// \param angle Angle in radians
/// \return Angle wrapped to [0, 2*pi]
template<typename T>
[[nodiscard]] constexpr auto wrap_two_pi(T angle) noexcept -> T
{
    angle = detail::angle_remainder(angle);
    if (angle < T(0))
    {
        angle += constants::two_pi<T>();
    }
    if (angle >= constants::two_pi<T>())
    {
        return T(0);
    }
    return angle;
}

/// \brief Calculate shortest angular distance between two angles
/// @tparam T Numeric type of the angles and result.
/// \param from Starting angle in radians
/// \param to Ending angle in radians
/// \return Shortest angular distance in range [-pi, pi]
template<typename T>
[[nodiscard]] constexpr auto angle_distance(T from, T to) noexcept -> T
{
    T diff = to - from;
    return wrap_pi(diff);
}

/// \brief Clamp value to range [min, max]
/// @tparam T Ordered numeric type of the value and bounds.
/// \param value Value to clamp
/// \param min_value Minimum value
/// \param max_value Maximum value
/// \return Clamped value
template<typename T>
[[nodiscard]] constexpr auto clamp(T value, T min_value, T max_value) noexcept -> T
{
    return std::clamp(value, min_value, max_value);
}

/// \brief Saturate value to range [0, 1]
/// @tparam T Numeric type of the value and result.
/// \param value Value to saturate
/// \return Value clamped to [0, 1]
template<typename T>
[[nodiscard]] constexpr auto saturate(T value) noexcept -> T
{
    return clamp(value, T(0), T(1));
}

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
template<typename T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr auto matrix_size_bytes() noexcept -> std::size_t
{
    return sizeof(Mat<T, R, C>);
}

/// @brief Get the size in bytes of a vector type
/// @tparam T Element type
/// @tparam N Number of dimensions
/// @return Size in bytes including alignment padding
template<typename T, std::size_t N>
[[nodiscard]] constexpr auto vector_size_bytes() noexcept -> std::size_t
{
    return sizeof(Vec<T, N>);
}

/// @brief Get the size in bytes of a quaternion type
/// @tparam T Element type
/// @return Size in bytes including alignment padding
template<typename T>
[[nodiscard]] constexpr auto quaternion_size_bytes() noexcept -> std::size_t
{
    return sizeof(Quaternion<T>);
}

/// @brief Check if a matrix type exceeds the configured stack size limit
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @return True if matrix size exceeds MICROLA_STACK_SIZE_LIMIT
/// @retval true The complete matrix object exceeds the configured limit.
/// @retval false The complete matrix object fits within the configured limit.
template<typename T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr auto matrix_exceeds_stack_limit() noexcept -> bool
{
    return sizeof(Mat<T, R, C>) > MICROLA_STACK_SIZE_LIMIT;
}

/// @brief Get the alignment requirement for a matrix type
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @return Alignment requirement in bytes
template<typename T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr auto matrix_alignment() noexcept -> std::size_t
{
    return alignof(Mat<T, R, C>);
}

/// @brief Compile-time assertion helper for stack size validation
/// @tparam T Element type
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @details Use this in static_assert to ensure matrices fit within stack limits.
///          Example: static_assert(validate_matrix_stack_size<float, 4, 4>(), "Matrix too large");
/// @return Whether the complete matrix object fits within the configured limit.
/// @retval true The object size does not exceed MICROLA_STACK_SIZE_LIMIT.
/// @retval false The object size exceeds MICROLA_STACK_SIZE_LIMIT.
template<typename T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr auto validate_matrix_stack_size() noexcept -> bool
{
    return sizeof(Mat<T, R, C>) <= MICROLA_STACK_SIZE_LIMIT;
}

}  // namespace memory_info

}  // namespace microla
