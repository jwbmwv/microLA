// SPDX-License-Identifier: MIT
/// @file compiler_features.hpp
/// @brief Compiler feature detection and cross-version C++ compatibility macros
/// @details Provides MICROLA_* macros that adapt to different C++ standard versions (C++11-C++26).
///          This header centralizes all compiler feature detection to avoid duplication.
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_COMPILER_FEATURES_HPP_
#define MICROLA_COMPILER_FEATURES_HPP_

// ==================== Library Configuration ====================
// MICROLA_AUTODETECT_SIMD: Enable automatic SIMD feature detection
// MICROLA_NO_EXCEPTIONS: Build without exceptions
// MICROLA_NO_RTTI: Build without RTTI
// MICROLA_EMBEDDED: Favor small-footprint embedded defaults
// CONFIG_MICROLA_NEON: Enable ARM NEON SIMD support
// CONFIG_MICROLA_MVE: Enable ARM MVE (Helium) SIMD support
// CONFIG_MICROLA_CMSIS: Enable CMSIS-DSP library support
// CONFIG_MICROLA_AVX: Enable x86 AVX/AVX2 SIMD support
// CONFIG_MICROLA_RISCV: Enable RISC-V SIMD support
// MICROLA_FIXED_POINT: Enable fixed-point arithmetic support (future)
// MICROLA_NO_DYNAMIC_ALLOC: Disable dynamic memory allocation
// MICROLA_STACK_SIZE_LIMIT: Maximum stack allocation size (default: 4096 bytes)

// ==================== Embedded System Defaults ====================
#if defined(MICROLA_EMBEDDED)
// For embedded systems, enable conservative defaults
#ifndef MICROLA_NO_EXCEPTIONS
#define MICROLA_NO_EXCEPTIONS
#endif
#ifndef MICROLA_NO_RTTI
#define MICROLA_NO_RTTI
#endif
#ifndef MICROLA_NO_DYNAMIC_ALLOC
#define MICROLA_NO_DYNAMIC_ALLOC
#endif
#ifndef MICROLA_STACK_SIZE_LIMIT
#define MICROLA_STACK_SIZE_LIMIT 2048  // Conservative 2KB limit for embedded
#endif
#endif

// ==================== Stack Size Limits ====================
#ifndef MICROLA_STACK_SIZE_LIMIT
#define MICROLA_STACK_SIZE_LIMIT 4096  // Default 4KB for non-embedded systems
#endif

#if defined(MICROLA_AUTODETECT_SIMD)
#if !defined(CONFIG_MICROLA_NEON) && defined(__ARM_NEON)
#define CONFIG_MICROLA_NEON
#endif
#if !defined(CONFIG_MICROLA_MVE) && defined(__ARM_FEATURE_MVE)
#define CONFIG_MICROLA_MVE
#endif
#if !defined(CONFIG_MICROLA_AVX) && (defined(__AVX__) || defined(__AVX2__))
#define CONFIG_MICROLA_AVX
#endif
#if !defined(CONFIG_MICROLA_RISCV) && defined(__RISCV__)
#define CONFIG_MICROLA_RISCV
#endif
#endif

// ==================== Exceptions / RTTI ====================
#if defined(MICROLA_NO_EXCEPTIONS) || !defined(__cpp_exceptions)
#define MICROLA_HAS_EXCEPTIONS 0
#else
#define MICROLA_HAS_EXCEPTIONS 1
#endif

#if defined(MICROLA_NO_RTTI)
#define MICROLA_HAS_RTTI 0
#elif defined(__cpp_rtti) || defined(__GXX_RTTI) || defined(_CPPRTTI)
#define MICROLA_HAS_RTTI 1
#else
#define MICROLA_HAS_RTTI 0
#endif

// ==================== C++14 Features ====================
#if __cplusplus >= 201402L
#define MICROLA_CONSTEXPR constexpr
#define MICROLA_CONSTEXPR14 constexpr
#else
#define MICROLA_CONSTEXPR
#define MICROLA_CONSTEXPR14
#endif

// ==================== C++17 Features ====================
// Note: MICROLA_NODISCARD must be placed BEFORE the return type in function declarations
// Correct: MICROLA_NODISCARD MICROLA_CONSTEXPR ReturnType func()
// Incorrect: MICROLA_CONSTEXPR MICROLA_NODISCARD ReturnType func()
#if __cplusplus >= 201703L
#define MICROLA_NODISCARD [[nodiscard]]
#define MICROLA_CONSTEXPR17 constexpr
#define MICROLA_INLINE_VAR inline
#define MICROLA_IF_CONSTEXPR if constexpr
#else
#define MICROLA_NODISCARD
#define MICROLA_CONSTEXPR17
#define MICROLA_INLINE_VAR static
#define MICROLA_IF_CONSTEXPR if
#endif

// ==================== C++20 Features ====================
#if __cplusplus >= 202002L
#define MICROLA_CONSTEXPR20 constexpr
#define MICROLA_CONSTEVAL consteval
#define MICROLA_CONSTINIT constinit
#else
#define MICROLA_CONSTEXPR20
#define MICROLA_CONSTEVAL constexpr
#define MICROLA_CONSTINIT
#endif

// ==================== C++23 Features ====================
#if __cplusplus >= 202302L
#define MICROLA_CONSTEXPR23 constexpr
#define MICROLA_IF_CONSTEVAL if consteval
#include <utility>
#define MICROLA_UNREACHABLE() std::unreachable()
#else
#define MICROLA_CONSTEXPR23
#define MICROLA_IF_CONSTEVAL if (false)
// Compiler-specific unreachable hints for older standards
#if defined(__GNUC__) || defined(__clang__)
#define MICROLA_UNREACHABLE() __builtin_unreachable()
#elif defined(_MSC_VER)
#define MICROLA_UNREACHABLE() __assume(0)
#else
#define MICROLA_UNREACHABLE() \
    do                       \
    {                        \
    } while (0)
#endif
#endif

// ==================== C++26 Features ====================
#if __cplusplus >= 202600L
#define MICROLA_CONSTEXPR26 constexpr
// C++26 makes std::sin, std::cos, std::sqrt, etc. constexpr
#define MICROLA_CONSTEXPR_TRIG constexpr
#else
#define MICROLA_CONSTEXPR26
#define MICROLA_CONSTEXPR_TRIG
#endif

// ==================== Type Conversion ====================
#if __cplusplus >= 202002L
#include <bit>
#define MICROLA_BIT_CAST(T, val) std::bit_cast<T>(val)
#else
// C++11 fallback: Use memcpy-based type conversion (safe, well-defined)
// This avoids strict aliasing violations that reinterpret_cast can cause
#include <cstring>
#include <type_traits>

namespace microla
{
namespace detail
{
template<typename To, typename From>
inline To bit_cast_memcpy(const From& src) noexcept
{
    static_assert(sizeof(To) == sizeof(From), "bit_cast requires same size types");
    static_assert(std::is_trivially_copyable<To>::value, "To must be trivially copyable");
    static_assert(std::is_trivially_copyable<From>::value, "From must be trivially copyable");
    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}
}  // namespace detail
}  // namespace microla

#define MICROLA_BIT_CAST(T, val) (::microla::detail::bit_cast_memcpy<T>(val))
#endif

// ==================== Compiler Hints & Optimizations ====================
// MICROLA_ASSUME: Optimization hint that condition is always true
#if defined(__clang__)
#define MICROLA_ASSUME(cond) __builtin_assume(cond)
#elif defined(_MSC_VER)
#define MICROLA_ASSUME(cond) __assume(cond)
#elif defined(__GNUC__) && __GNUC__ >= 13
#define MICROLA_ASSUME(cond) __attribute__((assume(cond)))
#else
#define MICROLA_ASSUME(cond)       \
    do                            \
    {                             \
        if (!(cond))              \
            MICROLA_UNREACHABLE(); \
    } while (0)
#endif

// MICROLA_FORCEINLINE: Strong inline hint for performance-critical code
#if defined(_MSC_VER)
#define MICROLA_FORCEINLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define MICROLA_FORCEINLINE __attribute__((always_inline)) inline
#else
#define MICROLA_FORCEINLINE inline
#endif

#endif  // MICROLA_COMPILER_FEATURES_HPP_
