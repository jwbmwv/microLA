// SPDX-License-Identifier: Apache-2.0
/// @file resource_checks.hpp
/// @brief Compile-time resource usage validation for embedded systems
/// @details Provides static assertions for memory layout and size constraints
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "compiler_features.hpp"
#include "vector.hpp"
#include "matrix.hpp"
#include "quaternion.hpp"
#include <type_traits>
#include <cstddef>

#ifndef MICROLA_RESOURCE_CHECKS_NAMESPACE
namespace microla
{
namespace resource_checks
{
#endif

// ==================== Memory Layout Validation ====================

/// @brief Validate Vec<T,N> memory layout for DMA compatibility
/// @details Ensures Vec is standard layout and properly aligned
/// @tparam T Vector element type.
/// @tparam N Number of vector components.
template<typename T, std::size_t N>
struct VecLayoutCheck
{
    using VecType = Vec<T, N>;

    // Vec must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout_v<VecType>, "Vec must be standard layout for DMA compatibility");

    // Vec should have minimal padding (size is either exact or rounded up to the active data alignment)
    static_assert(sizeof(VecType) == sizeof(T) * N ||
                      sizeof(VecType) == ((sizeof(T) * N + MICROLA_DATA_ALIGNMENT - 1) &
                                          ~static_cast<std::size_t>(MICROLA_DATA_ALIGNMENT - 1)),
                  "Vec has unexpected padding - check alignment settings");

#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(VecType) <= MICROLA_STACK_SIZE_LIMIT,
                  "Vec size exceeds MICROLA_STACK_SIZE_LIMIT - consider reducing dimension");
#endif
};

/// @brief Validate Mat<T,R,C> memory layout for DMA compatibility
/// @details Ensures Mat is standard layout and properly aligned
/// @tparam T Matrix element type.
/// @tparam R Number of matrix rows.
/// @tparam C Number of matrix columns.
template<typename T, std::size_t R, std::size_t C>
struct MatLayoutCheck
{
    using MatType = Mat<T, R, C>;

    static_assert(R > 0 && C > 0, "Matrix dimensions must be greater than 0");

    // Mat must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout_v<MatType>, "Mat must be standard layout for DMA compatibility");

#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(T) * R * C <= MICROLA_STACK_SIZE_LIMIT,
                  "Matrix size exceeds MICROLA_STACK_SIZE_LIMIT - consider reducing dimensions");
#endif

    // Verify contiguous row-major storage
    static_assert(sizeof(MatType) >= sizeof(T) * R * C, "Matrix storage is too small for elements");
};

/// @brief Validate Quaternion<T> memory layout for DMA compatibility
/// @details Ensures Quaternion is standard layout and properly aligned
/// @tparam T Floating-point quaternion component type.
template<typename T>
struct QuaternionLayoutCheck
{
    using QuatType = Quaternion<T>;

    static_assert(std::is_floating_point_v<T>, "Quaternion requires floating-point type");

    // Quaternion must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout_v<QuatType>, "Quaternion must be standard layout for DMA compatibility");

#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(QuatType) <= MICROLA_STACK_SIZE_LIMIT, "Quaternion size exceeds MICROLA_STACK_SIZE_LIMIT");
#endif

    // Verify expected size (4 components + alignment padding up to MICROLA_DATA_ALIGNMENT)
    static_assert(sizeof(QuatType) == sizeof(T) * 4 ||
                      sizeof(QuatType) == ((sizeof(T) * 4 + MICROLA_DATA_ALIGNMENT - 1) &
                                           ~static_cast<std::size_t>(MICROLA_DATA_ALIGNMENT - 1)),
                  "Quaternion has unexpected size");
};

// ==================== Common Type Validation ====================

// Validate common vector types
using Vec2fCheck = VecLayoutCheck<float, 2>;
using Vec3fCheck = VecLayoutCheck<float, 3>;
using Vec4fCheck = VecLayoutCheck<float, 4>;

// Validate common matrix types
using Mat2fCheck = MatLayoutCheck<float, 2, 2>;
using Mat3fCheck = MatLayoutCheck<float, 3, 3>;
using Mat4fCheck = MatLayoutCheck<float, 4, 4>;

// Validate quaternion
using QuatfCheck = QuaternionLayoutCheck<float>;

// ==================== Size Reporting (can be used at compile-time) ====================

/// @brief Report Vec size at compile time
/// @tparam T Vector element type.
/// @tparam N Number of vector components.
template<typename T, std::size_t N>
struct VecSizeInfo
{
    /// @brief Query the size of one component.
    /// @return sizeof(T), in bytes.
    static constexpr auto element_size() -> std::size_t { return sizeof(T); }
    /// @brief Query the vector dimension.
    /// @return The number of vector components N.
    static constexpr auto dimension() -> std::size_t { return N; }
    /// @brief Query the payload size without object padding.
    /// @return N times sizeof(T), in bytes.
    static constexpr auto theoretical_size() -> std::size_t { return sizeof(T) * N; }
    /// @brief Query the complete owning object size.
    /// @return sizeof(Vec<T,N>), including alignment padding, in bytes.
    static constexpr auto actual_size() -> std::size_t { return sizeof(Vec<T, N>); }
    /// @brief Query the owning type's alignment requirement.
    /// @return alignof(Vec<T,N>), in bytes.
    static constexpr auto alignment() -> std::size_t { return alignof(Vec<T, N>); }
    /// @brief Query the storage overhead beyond the component payload.
    /// @return The difference between complete object and payload sizes, in bytes.
    static constexpr auto padding() -> std::size_t { return actual_size() - theoretical_size(); }
};

/// @brief Report Mat size at compile time
/// @tparam T Matrix element type.
/// @tparam R Number of matrix rows.
/// @tparam C Number of matrix columns.
template<typename T, std::size_t R, std::size_t C>
struct MatSizeInfo
{
    /// @brief Query the size of one matrix entry.
    /// @return sizeof(T), in bytes.
    static constexpr auto element_size() -> std::size_t { return sizeof(T); }
    /// @brief Query the row count.
    /// @return The number of matrix rows R.
    static constexpr auto rows() -> std::size_t { return R; }
    /// @brief Query the column count.
    /// @return The number of matrix columns C.
    static constexpr auto cols() -> std::size_t { return C; }
    /// @brief Query the row-major payload size without object padding.
    /// @return R times C times sizeof(T), in bytes.
    static constexpr auto theoretical_size() -> std::size_t { return sizeof(T) * R * C; }
    /// @brief Query the complete owning matrix size.
    /// @return sizeof(Mat<T,R,C>), including alignment padding, in bytes.
    static constexpr auto actual_size() -> std::size_t { return sizeof(Mat<T, R, C>); }
    /// @brief Query the owning matrix's alignment requirement.
    /// @return alignof(Mat<T,R,C>), in bytes.
    static constexpr auto alignment() -> std::size_t { return alignof(Mat<T, R, C>); }
    /// @brief Query the storage overhead beyond the entry payload.
    /// @return The difference between complete object and payload sizes, in bytes.
    static constexpr auto padding() -> std::size_t { return actual_size() - theoretical_size(); }
};

/// @brief Report Quaternion size at compile time
/// @tparam T Floating-point quaternion component type.
template<typename T>
struct QuaternionSizeInfo
{
    /// @brief Query the size of one quaternion component.
    /// @return sizeof(T), in bytes.
    static constexpr auto element_size() -> std::size_t { return sizeof(T); }
    /// @brief Query the four-component payload size.
    /// @return Four times sizeof(T), in bytes.
    static constexpr auto theoretical_size() -> std::size_t { return sizeof(T) * 4; }
    /// @brief Query the complete quaternion object size.
    /// @return sizeof(Quaternion<T>), including alignment padding, in bytes.
    static constexpr auto actual_size() -> std::size_t { return sizeof(Quaternion<T>); }
    /// @brief Query the quaternion's alignment requirement.
    /// @return alignof(Quaternion<T>), in bytes.
    static constexpr auto alignment() -> std::size_t { return alignof(Quaternion<T>); }
    /// @brief Query the storage overhead beyond the four components.
    /// @return The difference between complete object and payload sizes, in bytes.
    static constexpr auto padding() -> std::size_t { return actual_size() - theoretical_size(); }
};

// ==================== DMA Compatibility Checks ====================

/// @brief Validate that Vec data is properly aligned for DMA
/// @details Cannot use offsetof with templates, but we can check alignment
/// @tparam T Vector element type.
/// @tparam N Number of vector components.
template<typename T, std::size_t N>
struct VecDMACheck
{
    // Verify Vec is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable_v<Vec<T, N>>, "Vec must be trivially copyable for DMA compatibility");

    // Verify proper alignment for DMA (typically 16 bytes for SIMD)
    static_assert(alignof(Vec<T, N>) >= alignof(T), "Vec alignment must be at least as strict as element type");
};

/// @brief Check trivial copyability and element-compatible matrix alignment.
/// @tparam T Matrix element type.
/// @tparam R Number of matrix rows.
/// @tparam C Number of matrix columns.
/// @note Device-specific DMA alignment and coherency requirements remain the caller's responsibility.
template<typename T, std::size_t R, std::size_t C>
struct MatDMACheck
{
    // Verify Mat is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable_v<Mat<T, R, C>>, "Mat must be trivially copyable for DMA compatibility");

    // Verify proper alignment for DMA
    static_assert(alignof(Mat<T, R, C>) >= alignof(T), "Mat alignment must be at least as strict as element type");
};

/// @brief Check trivial copyability and element-compatible quaternion alignment.
/// @tparam T Floating-point quaternion component type.
/// @note These checks do not certify device-specific DMA behavior.
template<typename T>
struct QuaternionDMACheck
{
    // Verify Quaternion is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable_v<Quaternion<T>>,
                  "Quaternion must be trivially copyable for DMA compatibility");

    // Verify proper alignment for DMA
    static_assert(alignof(Quaternion<T>) >= alignof(T),
                  "Quaternion alignment must be at least as strict as element type");
};

// Instantiate DMA checks for common types
using Vec3fDMACheck = VecDMACheck<float, 3>;
using Mat4fDMACheck = MatDMACheck<float, 4, 4>;
using QuatfDMACheck = QuaternionDMACheck<float>;

#ifndef MICROLA_RESOURCE_CHECKS_NAMESPACE
}  // namespace resource_checks
}  // namespace microla
#endif
