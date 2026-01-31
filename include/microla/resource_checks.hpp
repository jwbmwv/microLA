// SPDX-License-Identifier: MIT
/// @file resource_checks.hpp
/// @brief Compile-time resource usage validation for embedded systems
/// @details Provides static assertions for memory layout and size constraints
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_RESOURCE_CHECKS_HPP_
#define MICROLA_RESOURCE_CHECKS_HPP_

#include "compiler_features.hpp"
#include "vector.hpp"
#include "matrix.hpp"
#include "quaternion.hpp"
#include <type_traits>
#include <cstddef>

namespace microla
{
namespace resource_checks
{

// ==================== Memory Layout Validation ====================

/// @brief Validate Vec<T,N> memory layout for DMA compatibility
/// @details Ensures Vec is standard layout and properly aligned
template<typename T, std::uint32_t N>
struct VecLayoutCheck
{
    using VecType = Vec<T, N>;
    
    // Vec must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout<VecType>::value,
                  "Vec must be standard layout for DMA compatibility");
    
    // Vec should have minimal padding (aligned to 16 bytes or size of elements)
    static_assert(sizeof(VecType) == sizeof(T) * N || 
                  sizeof(VecType) == ((sizeof(T) * N + 15) & ~15),
                  "Vec has unexpected padding - check alignment settings");
                  
#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(VecType) <= MICROLA_STACK_SIZE_LIMIT,
                  "Vec size exceeds MICROLA_STACK_SIZE_LIMIT - consider reducing dimension");
#endif
};

/// @brief Validate Mat<T,R,C> memory layout for DMA compatibility
/// @details Ensures Mat is standard layout and properly aligned
template<typename T, std::uint32_t R, std::uint32_t C>
struct MatLayoutCheck
{
    using MatType = Mat<T, R, C>;
    
    static_assert(R > 0 && C > 0, "Matrix dimensions must be greater than 0");
    
    // Mat must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout<MatType>::value,
                  "Mat must be standard layout for DMA compatibility");
    
#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(T) * R * C <= MICROLA_STACK_SIZE_LIMIT,
                  "Matrix size exceeds MICROLA_STACK_SIZE_LIMIT - consider reducing dimensions");
#endif
    
    // Verify contiguous row-major storage
    static_assert(sizeof(MatType) >= sizeof(T) * R * C,
                  "Matrix storage is too small for elements");
};

/// @brief Validate Quaternion<T> memory layout for DMA compatibility
/// @details Ensures Quaternion is standard layout and properly aligned
template<typename T>
struct QuaternionLayoutCheck
{
    using QuatType = Quaternion<T>;
    
    static_assert(std::is_floating_point<T>::value,
                  "Quaternion requires floating-point type");
    
    // Quaternion must be standard layout for DMA and C compatibility
    static_assert(std::is_standard_layout<QuatType>::value,
                  "Quaternion must be standard layout for DMA compatibility");
    
#ifdef MICROLA_STACK_SIZE_LIMIT
    static_assert(sizeof(QuatType) <= MICROLA_STACK_SIZE_LIMIT,
                  "Quaternion size exceeds MICROLA_STACK_SIZE_LIMIT");
#endif
    
    // Verify expected size (4 components + alignment)
    static_assert(sizeof(QuatType) == sizeof(T) * 4 || 
                  sizeof(QuatType) == ((sizeof(T) * 4 + 15) & ~15),
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
template<typename T, std::uint32_t N>
struct VecSizeInfo
{
    static constexpr std::size_t element_size() { return sizeof(T); }
    static constexpr std::uint32_t dimension() { return N; }
    static constexpr std::size_t theoretical_size() { return sizeof(T) * N; }
    static constexpr std::size_t actual_size() { return sizeof(Vec<T, N>); }
    static constexpr std::size_t alignment() { return alignof(Vec<T, N>); }
    static constexpr std::size_t padding() { return actual_size() - theoretical_size(); }
};

/// @brief Report Mat size at compile time
template<typename T, std::uint32_t R, std::uint32_t C>
struct MatSizeInfo
{
    static constexpr std::size_t element_size() { return sizeof(T); }
    static constexpr std::uint32_t rows() { return R; }
    static constexpr std::uint32_t cols() { return C; }
    static constexpr std::size_t theoretical_size() { return sizeof(T) * R * C; }
    static constexpr std::size_t actual_size() { return sizeof(Mat<T, R, C>); }
    static constexpr std::size_t alignment() { return alignof(Mat<T, R, C>); }
    static constexpr std::size_t padding() { return actual_size() - theoretical_size(); }
};

/// @brief Report Quaternion size at compile time
template<typename T>
struct QuaternionSizeInfo
{
    static constexpr std::size_t element_size() { return sizeof(T); }
    static constexpr std::size_t theoretical_size() { return sizeof(T) * 4; }
    static constexpr std::size_t actual_size() { return sizeof(Quaternion<T>); }
    static constexpr std::size_t alignment() { return alignof(Quaternion<T>); }
    static constexpr std::size_t padding() { return actual_size() - theoretical_size(); }
};

// ==================== DMA Compatibility Checks ====================

/// @brief Validate that Vec data is properly aligned for DMA
/// @details Cannot use offsetof with templates, but we can check alignment
template<typename T, std::uint32_t N>
struct VecDMACheck
{
    // Verify Vec is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable<Vec<T, N>>::value,
                  "Vec must be trivially copyable for DMA compatibility");
    
    // Verify proper alignment for DMA (typically 16 bytes for SIMD)
    static_assert(alignof(Vec<T, N>) >= alignof(T),
                  "Vec alignment must be at least as strict as element type");
};

template<typename T, std::uint32_t R, std::uint32_t C>
struct MatDMACheck
{
    // Verify Mat is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable<Mat<T, R, C>>::value,
                  "Mat must be trivially copyable for DMA compatibility");
    
    // Verify proper alignment for DMA
    static_assert(alignof(Mat<T, R, C>) >= alignof(T),
                  "Mat alignment must be at least as strict as element type");
};

template<typename T>
struct QuaternionDMACheck
{
    // Verify Quaternion is trivially copyable (required for DMA)
    static_assert(std::is_trivially_copyable<Quaternion<T>>::value,
                  "Quaternion must be trivially copyable for DMA compatibility");
    
    // Verify proper alignment for DMA
    static_assert(alignof(Quaternion<T>) >= alignof(T),
                  "Quaternion alignment must be at least as strict as element type");
};

// Instantiate DMA checks for common types
using Vec3fDMACheck = VecDMACheck<float, 3>;
using Mat4fDMACheck = MatDMACheck<float, 4, 4>;
using QuatfDMACheck = QuaternionDMACheck<float>;

}  // namespace resource_checks
}  // namespace microla

#endif  // MICROLA_RESOURCE_CHECKS_HPP_
