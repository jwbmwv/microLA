// SPDX-License-Identifier: Apache-2.0
/// @file high_precision.hpp
/// @brief Selects a high-precision intermediate type when beneficial
/// @details Provides a `hi_t<T>` alias that resolves to `long double` on
/// platforms where it offers greater precision than `double`, otherwise
/// falls back to `double`. Controlled by CMake option
/// `MICROLA_USE_LONG_DOUBLE_INTERMEDIATES`.
///
/// @section motivation Motivation
///
/// Many numerical algorithms accumulate intermediate results that benefit from
/// extra precision. For example, dot products of large vectors can lose accuracy
/// from rounding errors when summed in single precision. By using `hi_t<T>` for
/// intermediate accumulation, you can maintain precision without affecting storage.
///
/// @section usage Usage Example
///
/// @code{.cpp}
/// template<typename T, std::size_t N>
/// auto precise_dot(const Vec<T, N>& a, const Vec<T, N>& b) -> T
/// {
///     using HiPrec = microla::detail::hi_t<T>;
///     HiPrec sum = HiPrec(0);
///     for (std::size_t i = 0; i < N; ++i)
///     {
///         sum += HiPrec(a[i]) * HiPrec(b[i]);
///     }
///     return static_cast<T>(sum);
/// }
/// @endcode
///
/// @section cmake_configuration CMake Configuration
///
/// Enable high-precision intermediates at configure time:
///
/// @code{.sh}
/// cmake -B build -DMICROLA_USE_LONG_DOUBLE_INTERMEDIATES=ON
/// @endcode
///
/// When enabled, `hi_t<T>` resolves to `long double` if it provides more
/// precision than `double` (checked via `std::numeric_limits<T>::digits`).
/// On platforms where `long double` == `double` (e.g., some 64-bit ARM),
/// the type automatically falls back to `double` to avoid overhead.
///
/// @section performance Performance Considerations
///
/// **Precision vs. Speed Trade-off:**
/// - `long double` is typically 80-bit (x86) or 128-bit (some architectures)
/// - Hardware support varies: x87 FPU has native 80-bit, software emulation is slower
/// - ~2-10× slower than `double` depending on platform and compiler
/// - Most beneficial for algorithms with O(n) accumulation where errors compound
///
/// **When to Enable:**
/// - High-dimensional dot products or matrix operations
/// - Iterative solvers with many iterations
/// - Safety-critical applications requiring strict error bounds
/// - Testing/validation against reference implementations
///
/// **When to Disable (default):**
/// - Real-time embedded systems with strict timing constraints
/// - Platforms without hardware long double support
/// - Applications where double precision is sufficient
/// - Memory-constrained systems (larger stack frames)
///
/// @section platforms Platform Behavior
///
/// | Platform      | `long double` | Typical Precision       | Hardware Support |
/// |---------------|---------------|-------------------------|------------------|
/// | x86-64 GCC    | 80-bit        | ~19 decimal digits      | Native (x87)     |
/// | x86-64 MSVC   | 64-bit        | Same as double          | N/A              |
/// | ARM64 GCC     | 128-bit       | ~34 decimal digits      | Software         |
/// | ARM Cortex-M  | 64-bit        | Same as double          | N/A              |
/// | RISC-V        | 128-bit       | ~34 decimal digits      | Varies           |
///
/// @note The library automatically detects when `long double` offers no benefit
///       and falls back to `double`, even when MICROLA_USE_LONG_DOUBLE_INTERMEDIATES
///       is enabled.
///
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include <type_traits>
#include <limits>

namespace microla::detail
{
// Prefer `long double` when it provides strictly more precision than `double`.
// `std::conditional_t` keeps the alias selection compile-time only.
#if defined(MICROLA_USE_LONG_DOUBLE_INTERMEDIATES) && (MICROLA_USE_LONG_DOUBLE_INTERMEDIATES)
using high_prec_base =
    std::conditional_t<(std::numeric_limits<long double>::digits > std::numeric_limits<double>::digits), long double,
                       double>;
#else
using high_prec_base = double;
#endif

/// @brief High-precision intermediate type alias.
/// @tparam T Base type (typically float or double)
/// @details Always resolves to either `double` or `long double` depending on
///          platform capabilities and CMake configuration. Never uses the input
///          type T directly - this ensures intermediate computations always use
///          at least double precision, even when working with float data.
///
/// @note Type-independent: `hi_t<float>` and `hi_t<double>` resolve to the same type.
///       This is intentional - the goal is platform-optimal intermediate precision,
///       not per-type specialization.
template<typename T>
using hi_t = high_prec_base;

}  // namespace microla::detail
