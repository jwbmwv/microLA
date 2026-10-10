// SPDX-License-Identifier: Apache-2.0
/// @file numerical_stability.hpp
/// @brief Numerical stability helpers and utilities
/// @details Provides tools for improving numerical accuracy and detecting ill-conditioned problems
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "compiler_features.hpp"
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <type_traits>
#include <limits>
#include "high_precision.hpp"

#ifndef MICROLA_NUMERICAL_NAMESPACE
namespace microla
{
namespace numerical
{
#endif

/// @brief Machine epsilon for a given floating-point type
/// @tparam T Floating-point type
/// @return Machine epsilon
template<typename T>
constexpr auto epsilon() noexcept -> T
{
    return std::numeric_limits<T>::epsilon();
}

/// @brief Approximately equal comparison with relative and absolute tolerance
/// @details Uses both relative and absolute epsilon for better handling of values near zero
/// @tparam T Floating-point type of the values and tolerances.
/// @param a First value
/// @param b Second value
/// @param rel_tol Relative tolerance (default: 100 * epsilon)
/// @param abs_tol Absolute tolerance (default: epsilon)
/// @return true if values are approximately equal
/// @retval false A value is NaN, unequal inputs are non-finite, or the tolerances reject the difference.
/// @retval true The values are exactly equal or differ by no more than an accepted tolerance.
template<typename T>
auto approx_equal(T a, T b, T rel_tol = T(100) * epsilon<T>(), T abs_tol = epsilon<T>()) noexcept -> bool
{
    static_assert(std::is_floating_point_v<T>, "approx_equal requires floating-point type");

    if (a == b)
    {
        return true;
    }
    if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(rel_tol) || !std::isfinite(abs_tol) ||
        rel_tol < T(0) || abs_tol < T(0))
    {
        return false;
    }
    const T diff = std::abs(a - b);

    // Absolute tolerance check (important for values near zero)
    if (diff <= abs_tol)
    {
        return true;
    }

    // Relative tolerance check
    const T max_abs = std::max(std::abs(a), std::abs(b));
    return diff <= rel_tol * max_abs;
}

/// @brief Kahan summation algorithm for reduced floating-point error
/// @details More accurate than naive summation for large arrays
/// @tparam T Floating-point type
/// @tparam Iterator Forward iterator type
/// @param begin Iterator to start of range
/// @param end Iterator to end of range
/// @return Compensated sum
template<typename T, typename Iterator>
auto kahan_sum(Iterator begin, Iterator end) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "kahan_sum requires floating-point type");

    T sum = T(0);
    T c = T(0);  // Compensation for lost low-order bits

    for (Iterator it = begin; it != end; ++it)
    {
        T y = *it - c;
        T t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }

    return sum;
}

/// @brief Compute scale factor for avoiding overflow/underflow in norm computation
/// @details Used to safely compute sqrt(a² + b²) without intermediate overflow
/// @tparam T Floating-point type of the components and result.
/// @param a First value
/// @param b Second value
/// @return sqrt(a² + b²) computed safely
template<typename T>
auto hypot(T a, T b) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "hypot requires floating-point type");

    return std::hypot(a, b);
}

/// @brief Three-value hypot for 3D vectors
/// @tparam T Floating-point type of the components and result.
/// @param a First value
/// @param b Second value
/// @param c Third value
/// @return sqrt(a² + b² + c²) computed safely
/// @details Uses the standard scaled hypot implementation to avoid intermediate overflow.
template<typename T>
auto hypot3(T a, T b, T c) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "hypot3 requires floating-point type");

    return std::hypot(a, b, c);
}

/// @brief Numerically stable computation of log(1 + x) for small x
/// @details More accurate than log(1 + x) when x is close to zero
/// @tparam T Floating-point type of the argument and result.
/// @param x Small value
/// @return log(1 + x)
template<typename T>
auto log1p(T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "log1p requires floating-point type");
    return std::log1p(x);  // Use standard library implementation
}

/// @brief Numerically stable computation of exp(x) - 1 for small x
/// @details More accurate than exp(x) - 1 when x is close to zero
/// @tparam T Floating-point type of the argument and result.
/// @param x Small value
/// @return exp(x) - 1
template<typename T>
auto expm1(T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "expm1 requires floating-point type");
    return std::expm1(x);  // Use standard library implementation
}

/// @brief Check if a value is within the representable range
/// @tparam T Floating-point type whose normal range is inspected.
/// @param value Value to check
/// @return true if value is representable (not infinity or subnormal)
/// @retval true The value is finite and its magnitude is at least the smallest normal value.
/// @retval false The value is zero, subnormal, infinite, or NaN.
template<typename T>
auto is_representable(T value) noexcept -> bool
{
    static_assert(std::is_floating_point_v<T>, "is_representable requires floating-point type");
    return std::isfinite(value) && std::abs(value) >= std::numeric_limits<T>::min();
}

/// @brief Estimate relative error in a floating-point computation
/// @tparam T Floating-point type of the computed and reference values.
/// @param computed Computed value
/// @param exact Exact (reference) value
/// @return Relative error
template<typename T>
auto relative_error(T computed, T exact) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "relative_error requires floating-point type");

    if (exact == T(0))
    {
        return std::abs(computed);
    }

    return std::abs((computed - exact) / exact);
}

/// @brief Compute condition number estimate for a 2x2 matrix
/// @details Ratio of largest to smallest singular value using SVD approach
/// @tparam T Floating-point type of the matrix entries and condition estimate.
/// @param a Matrix element (0,0)
/// @param b Matrix element (0,1)
/// @param c Matrix element (1,0)
/// @param d Matrix element (1,1)
/// @return Approximate condition number
template<typename T>
auto condition_number_2x2(T a, T b, T c, T d) noexcept -> T
{
    T max_elem = std::max(std::max(std::abs(a), std::abs(b)), std::max(std::abs(c), std::abs(d)));
    if (max_elem == T(0) || !std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) || !std::isfinite(d))
    {
        return std::numeric_limits<T>::infinity();
    }
    using high_t = microla::detail::hi_t<T>;
    const high_t left_top = static_cast<high_t>(a) / max_elem;
    const high_t right_top = static_cast<high_t>(b) / max_elem;
    const high_t left_bottom = static_cast<high_t>(c) / max_elem;
    const high_t right_bottom = static_cast<high_t>(d) / max_elem;
    const high_t determinant = std::abs(left_top * right_bottom - right_top * left_bottom);
    if (determinant == static_cast<high_t>(0))
    {
        return std::numeric_limits<T>::infinity();
    }
    const high_t aa = left_top * left_top + left_bottom * left_bottom;
    const high_t bb = right_top * right_top + right_bottom * right_bottom;
    const high_t ab = left_top * right_top + left_bottom * right_bottom;
    const high_t largest = (aa + bb + std::hypot(aa - bb, static_cast<high_t>(2) * ab)) / static_cast<high_t>(2);
    return static_cast<T>(largest / determinant);
}

/// @brief Numerically stable dot product using Kahan summation
/// @tparam T Floating-point type
/// @param a Array of values
/// @param b Array of values
/// @param n Number of elements
/// @return Dot product computed with reduced error
template<typename T>
auto stable_dot_product(const T* a, const T* b, std::size_t n) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "stable_dot_product requires floating-point type");

    T sum = T(0);
    T c = T(0);  // Compensation

    for (std::size_t i = 0; i < n; ++i)
    {
        T product = a[i] * b[i];
        T y = product - c;
        T t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }

    return sum;
}

/// @brief Numerically stable polynomial evaluation using Horner's method
/// @tparam T Floating-point type
/// @param coeffs Array of polynomial coefficients [a0, a1, a2, ..., an]
/// @param n Degree of polynomial + 1 (number of coefficients)
/// @param x Point at which to evaluate
/// @return P(x) = a0 + a1*x + a2*x² + ... + an*x^n
template<typename T>
auto horner_eval(const T* coeffs, std::size_t n, T x) noexcept -> T
{
    static_assert(std::is_floating_point_v<T>, "horner_eval requires floating-point type");

    if (n == 0)
    {
        return T(0);
    }

    T result = coeffs[n - 1];
    for (int i = static_cast<int>(n) - 2; i >= 0; --i)
    {
        result = result * x + coeffs[i];
    }

    return result;
}

/// @brief Check if matrix is likely to be ill-conditioned
/// @tparam T Floating-point type of the condition estimate and threshold.
/// @param condition_number Estimated condition number
/// @param threshold Warning threshold (default: 1e10)
/// @return true if matrix appears ill-conditioned
/// @retval true The estimate is non-finite or exceeds the threshold.
/// @retval false The estimate is finite and does not exceed the threshold.
template<typename T>
auto is_ill_conditioned(T condition_number, T threshold = T(1e10)) noexcept -> bool
{
    return condition_number > threshold || !std::isfinite(condition_number);
}

/// @brief Estimate loss of precision in bits
/// @tparam T Floating-point type of the condition estimate.
/// @param condition_number Condition number of problem
/// @return Truncated log2 of the estimate, zero for estimates at most one, or INT_MAX for non-finite input.
template<typename T>
auto precision_loss_bits(T condition_number) noexcept -> int
{
    if (!std::isfinite(condition_number))
    {
        return std::numeric_limits<int>::max();
    }
    if (condition_number <= T(1))
    {
        return 0;
    }

    // log2(condition_number) gives bits of precision lost
    return static_cast<int>(std::log2(condition_number));
}

/// @brief Compute ULP (Units in Last Place) distance between two floats
/// @details Measures how many representable floats are between two values
/// @tparam T IEEE-754 float or double type whose representable steps are counted.
/// @param a First value
/// @param b Second value
/// @return ULP distance, saturated at INT64_MAX; NaN also returns INT64_MAX.
template<typename T>
auto ulp_distance(T a, T b) noexcept -> std::int64_t
{
    static_assert((std::is_same_v<T, float> || std::is_same_v<T, double>) && std::numeric_limits<T>::is_iec559,
                  "ulp_distance requires IEEE float or double");
    if (std::isnan(a) || std::isnan(b))
    {
        return std::numeric_limits<std::int64_t>::max();
    }
    if (a == b)
    {
        return 0;
    }

    using Bits = std::conditional_t<std::is_same_v<T, float>, std::uint32_t, std::uint64_t>;
    constexpr Bits sign = Bits(1) << (std::numeric_limits<Bits>::digits - 1);
    const auto ordered = [](T value) -> Bits
    {
        const Bits bits = MICROLA_BIT_CAST(Bits, value);
        return (bits & sign) != Bits(0) ? sign - (bits & ~sign) : sign + bits;
    };
    const Bits left = ordered(a);
    const Bits right = ordered(b);
    const Bits distance = left > right ? left - right : right - left;
    const auto limit = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
    return static_cast<std::int64_t>(std::min(static_cast<std::uint64_t>(distance), limit));
}

#ifndef MICROLA_NUMERICAL_NAMESPACE
}  // namespace numerical
}  // namespace microla
#endif
