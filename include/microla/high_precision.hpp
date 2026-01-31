// SPDX-License-Identifier: Apache-2.0
/// @file high_precision.hpp
/// @brief Selects a high-precision intermediate type when beneficial
/// @details Provides a `hi_t<T>` alias that resolves to `long double` on
/// platforms where it offers greater precision than `double`, otherwise
/// falls back to `double`. Controlled by CMake option
/// `MICROLA_USE_LONG_DOUBLE_INTERMEDIATES`.

#pragma once

#include <type_traits>
#include <limits>

namespace microla::detail
{
// Prefer `long double` when it provides strictly more precision than `double`.
// Use `std::conditional_t` under the C++17 baseline.
#if defined(MICROLA_USE_LONG_DOUBLE_INTERMEDIATES) && (MICROLA_USE_LONG_DOUBLE_INTERMEDIATES)
using high_prec_base =
    std::conditional_t<(std::numeric_limits<long double>::digits > std::numeric_limits<double>::digits), long double,
                       double>;
#else
using high_prec_base = double;
#endif

template<typename T>
using hi_t = high_prec_base;

}  // namespace microla::detail
