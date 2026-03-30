// SPDX-License-Identifier: Apache-2.0
/// @file compiler_features.hpp
/// @brief Compiler feature detection and cross-version C++ feature macros
/// @details Provides MICROLA_* macros for the required C++20 baseline plus optional C++23/C++26
///          features and platform/compiler hints.
///          This header centralizes all compiler feature detection to avoid duplication.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include <type_traits>

#if defined(__has_include)
#if __has_include(<bit>)
#include <bit>
#endif
#if __has_include(<utility>)
#include <utility>
#endif
#endif

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

#if defined(MICROLA_NO_DYNAMIC_ALLOC)
#define MICROLA_HAS_DYNAMIC_ALLOC 0
#else
#define MICROLA_HAS_DYNAMIC_ALLOC 1
#endif

#if MICROLA_HAS_DYNAMIC_ALLOC
#define MICROLA_DYNAMIC_ALLOC_ONLY(...) __VA_ARGS__
#else
#define MICROLA_DYNAMIC_ALLOC_ONLY(...)
#endif

// ==================== C++20 Minimum ====================
#define MICROLA_CONSTEXPR20 constexpr

#if defined(__cpp_consteval) && (__cpp_consteval >= 201811L)
#define MICROLA_CONSTEVAL consteval
#else
#define MICROLA_CONSTEVAL constexpr
#endif

// ==================== C++23 Features ====================
#if defined(__cpp_if_consteval) && (__cpp_if_consteval >= 202106L)
#define MICROLA_CONSTEXPR23 constexpr
#define MICROLA_IF_CONSTEVAL if consteval
#else
#define MICROLA_CONSTEXPR23
#define MICROLA_IF_CONSTEVAL if (false)
#endif

#if defined(__cpp_lib_unreachable) && (__cpp_lib_unreachable >= 202202L)
#define MICROLA_UNREACHABLE() std::unreachable()
#else
// Compiler-specific unreachable hints when std::unreachable is unavailable.
#if defined(__GNUC__) || defined(__clang__)
#define MICROLA_UNREACHABLE() __builtin_unreachable()
#elif defined(_MSC_VER)
#define MICROLA_UNREACHABLE() __assume(0)
#else
#define MICROLA_UNREACHABLE() \
    do                        \
    {                         \
    } while (0)
#endif
#endif

// ==================== C++26 Features ====================
// C++26 brings constexpr <cmath> functions
#if __cplusplus >= 202600L
#define MICROLA_CONSTEXPR26 constexpr
// C++26 makes std::sin, std::cos, std::sqrt, etc. constexpr
#define MICROLA_CONSTEXPR_TRIG constexpr
#else
#define MICROLA_CONSTEXPR26
#define MICROLA_CONSTEXPR_TRIG
#endif

namespace microla
{
namespace detail
{
MICROLA_CONSTEVAL inline auto stack_size_limit_is_valid() noexcept -> bool
{
    return MICROLA_STACK_SIZE_LIMIT > 0;
}

static_assert(stack_size_limit_is_valid(), "MICROLA_STACK_SIZE_LIMIT must be greater than 0");
}  // namespace detail
}  // namespace microla

// ==================== Embedded System Extensions ====================

// ==================== ROM/Flash Placement ====================
// Place constant data in ROM/Flash for embedded systems. Use platform-
// appropriate section specifiers: ELF uses ".rodata"/".flash", Mach-O
// (macOS/iOS) requires a segment and section pair.
#if defined(__APPLE__) && defined(__MACH__)
// Mach-O targets (Apple) expect a segment,section specifier like
// "__TEXT,__const" for read-only data. For Flash placement there's no
// portable equivalent on macOS, so leave MICROLA_FLASH empty.
#define MICROLA_RODATA __attribute__((section("__TEXT,__const")))
#define MICROLA_FLASH
#elif defined(__GNUC__) || defined(__clang__)
#define MICROLA_RODATA __attribute__((section(".rodata")))
#define MICROLA_FLASH __attribute__((section(".flash")))
#elif defined(__ICCARM__) || defined(__ICCRX__)
#define MICROLA_RODATA _Pragma("location=\".rodata\"")
#define MICROLA_FLASH _Pragma("location=\".flash\"")
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define MICROLA_RODATA __attribute__((section(".rodata")))
#define MICROLA_FLASH __attribute__((section(".flash")))
#else
#define MICROLA_RODATA
#define MICROLA_FLASH
#endif

// ==================== Cache Line Alignment ====================
// Configure cache line size for optimal performance
#ifndef MICROLA_CACHE_LINE_SIZE
#if defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_8A__)
#define MICROLA_CACHE_LINE_SIZE 64  // Cortex-A series typically 64 bytes
#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define MICROLA_CACHE_LINE_SIZE 32  // Cortex-M series typically 32 bytes or no cache
#elif defined(__x86_64__) || defined(__i386__)
#define MICROLA_CACHE_LINE_SIZE 64  // x86/x64 typically 64 bytes
#elif defined(__riscv)
#define MICROLA_CACHE_LINE_SIZE 64  // RISC-V typically 64 bytes
#else
#define MICROLA_CACHE_LINE_SIZE 32  // Conservative default
#endif
#endif

// Align data to cache line boundary to prevent false sharing
#if defined(__GNUC__) || defined(__clang__)
#define MICROLA_CACHE_ALIGNED alignas(MICROLA_CACHE_LINE_SIZE)
#elif defined(_MSC_VER)
#define MICROLA_CACHE_ALIGNED __declspec(align(MICROLA_CACHE_LINE_SIZE))
#else
#define MICROLA_CACHE_ALIGNED alignas(64)
#endif

// ==================== Memory Barriers ====================
// Memory barriers for multi-core safety
#if defined(__GNUC__) || defined(__clang__)
#define MICROLA_MEMORY_BARRIER() __sync_synchronize()
#define MICROLA_COMPILER_BARRIER() __asm__ __volatile__("" ::: "memory")
#elif defined(_MSC_VER)
#include <intrin.h>
#define MICROLA_MEMORY_BARRIER() _ReadWriteBarrier()
#define MICROLA_COMPILER_BARRIER() _ReadWriteBarrier()
#elif defined(__ICCARM__)
#define MICROLA_MEMORY_BARRIER() __DMB()
#define MICROLA_COMPILER_BARRIER() asm("")
#else
#define MICROLA_MEMORY_BARRIER() \
    do                           \
    {                            \
    } while (0)
#define MICROLA_COMPILER_BARRIER() \
    do                             \
    {                              \
    } while (0)
#endif

// ==================== Interrupt Safety ====================
// Mark functions as interrupt-safe (no heap allocation, no floating point in some contexts)
#if defined(__GNUC__) || defined(__clang__)
#define MICROLA_INTERRUPT_SAFE __attribute__((interrupt))
#elif defined(__ICCARM__)
#define MICROLA_INTERRUPT_SAFE __irq
#else
#define MICROLA_INTERRUPT_SAFE
#endif

// Mark functions that are safe to call from ISR context (reentrant, lock-free)
#define MICROLA_ISR_SAFE  // Documentation marker only

// ==================== DMA Alignment ====================
// Ensure proper alignment for DMA operations
#if defined(__ARM_ARCH)
#define MICROLA_DMA_ALIGNED alignas(32)  // ARM DMA typically requires 32-byte alignment
#elif defined(__x86_64__) || defined(__i386__)
#define MICROLA_DMA_ALIGNED alignas(64)  // x86 DMA typically 64-byte aligned
#else
#define MICROLA_DMA_ALIGNED alignas(32)
#endif

// ==================== Deterministic Timing ====================
// Mark code sections where timing determinism is critical
#define MICROLA_TIMING_CRITICAL  // Documentation marker
#define MICROLA_CONSTANT_TIME    // Documentation marker for constant-time operations
// ==================== Type Conversion ====================

#if defined(__cpp_lib_bit_cast) && (__cpp_lib_bit_cast >= 201806L)
#define MICROLA_BIT_CAST(T, val) std::bit_cast<T>(val)
#else
// Fallback: use memcpy-based type conversion (safe, well-defined)
// This avoids strict aliasing violations that reinterpret_cast can cause
#include <cstring>

namespace microla
{
namespace detail
{

template<typename To, typename From>
inline auto bit_cast_memcpy(const From& src) noexcept -> To
{
    static_assert(sizeof(To) == sizeof(From), "bit_cast requires same size types");
    static_assert(std::is_trivially_copyable_v<To>, "To must be trivially copyable");
    static_assert(std::is_trivially_copyable_v<From>, "From must be trivially copyable");
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
    do                             \
    {                              \
        if (!(cond))               \
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
