// SPDX-License-Identifier: MIT
/// @file numerical_stability.hpp
/// @brief Numerical stability helpers and utilities
/// @details Provides tools for improving numerical accuracy and detecting ill-conditioned problems
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_NUMERICAL_STABILITY_HPP_
#define MICROLA_NUMERICAL_STABILITY_HPP_

#include "compiler_features.hpp"
#include <cmath>
#include <limits>

namespace microla
{
namespace numerical
{

/// @brief Machine epsilon for a given floating-point type
/// @tparam T Floating-point type
/// @return Machine epsilon
template<typename T>
MICROLA_CONSTEXPR T epsilon() noexcept
{
    return std::numeric_limits<T>::epsilon();
}

/// @brief Approximately equal comparison with relative and absolute tolerance
/// @details Uses both relative and absolute epsilon for better handling of values near zero
/// @param a First value
/// @param b Second value
/// @param rel_tol Relative tolerance (default: 100 * epsilon)
/// @param abs_tol Absolute tolerance (default: epsilon)
/// @return true if values are approximately equal
template<typename T>
bool approx_equal(T a, T b, T rel_tol = T(100) * epsilon<T>(), T abs_tol = epsilon<T>()) noexcept
{
    static_assert(std::is_floating_point<T>::value, "approx_equal requires floating-point type");

    const T diff = std::abs(a - b);

    // Absolute tolerance check (important for values near zero)
    if (diff <= abs_tol)
        return true;

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
T kahan_sum(Iterator begin, Iterator end) noexcept
{
    static_assert(std::is_floating_point<T>::value, "kahan_sum requires floating-point type");

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
/// @param a First value
/// @param b Second value
/// @return sqrt(a² + b²) computed safely
template<typename T>
T hypot(T a, T b) noexcept
{
    static_assert(std::is_floating_point<T>::value, "hypot requires floating-point type");

    T abs_a = std::abs(a);
    T abs_b = std::abs(b);

    if (abs_a == T(0))
        return abs_b;
    if (abs_b == T(0))
        return abs_a;

    // Scale to avoid overflow
    T max_val = std::max(abs_a, abs_b);
    T min_val = std::min(abs_a, abs_b);

    T ratio = min_val / max_val;
    return max_val * std::sqrt(T(1) + ratio * ratio);
}

/// @brief Three-value hypot for 3D vectors
/// @param a First value
/// @param b Second value
/// @param c Third value
/// @return sqrt(a² + b² + c²) computed safely
template<typename T>
T hypot3(T a, T b, T c) noexcept
{
    static_assert(std::is_floating_point<T>::value, "hypot3 requires floating-point type");

    // Use absolute values
    a = std::abs(a);
    b = std::abs(b);
    c = std::abs(c);

    // Find the maximum to use for scaling (avoid initializer list for portability)
    T max_val = std::max(std::max(a, b), c);

    if (max_val == T(0) || !std::isfinite(max_val))
        return max_val;

    // Scale by max to prevent overflow/underflow
    a /= max_val;
    b /= max_val;
    c /= max_val;

    return max_val * std::sqrt(a * a + b * b + c * c);
}

/// @brief Numerically stable computation of log(1 + x) for small x
/// @details More accurate than log(1 + x) when x is close to zero
/// @param x Small value
/// @return log(1 + x)
template<typename T>
T log1p(T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "log1p requires floating-point type");
    return std::log1p(x);  // Use standard library implementation
}

/// @brief Numerically stable computation of exp(x) - 1 for small x
/// @details More accurate than exp(x) - 1 when x is close to zero
/// @param x Small value
/// @return exp(x) - 1
template<typename T>
T expm1(T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "expm1 requires floating-point type");
    return std::expm1(x);  // Use standard library implementation
}

/// @brief Check if a value is within the representable range
/// @param value Value to check
/// @return true if value is representable (not infinity or subnormal)
template<typename T>
bool is_representable(T value) noexcept
{
    static_assert(std::is_floating_point<T>::value, "is_representable requires floating-point type");
    return std::isfinite(value) && std::abs(value) >= std::numeric_limits<T>::min();
}

/// @brief Estimate relative error in a floating-point computation
/// @param computed Computed value
/// @param exact Exact (reference) value
/// @return Relative error
template<typename T>
T relative_error(T computed, T exact) noexcept
{
    static_assert(std::is_floating_point<T>::value, "relative_error requires floating-point type");

    if (exact == T(0))
        return std::abs(computed);

    return std::abs((computed - exact) / exact);
}

/// @brief Compute condition number estimate for a 2x2 matrix
/// @details Ratio of largest to smallest singular value using SVD approach
/// @param a Matrix element (0,0)
/// @param b Matrix element (0,1)  
/// @param c Matrix element (1,0)
/// @param d Matrix element (1,1)
/// @return Approximate condition number
template<typename T>
T condition_number_2x2(T a, T b, T c, T d) noexcept
{
    // Handle trivial case
    if (a == T(0) && b == T(0) && c == T(0) && d == T(0))
        return T(1);  // Zero matrix has condition number 1 by convention

    // Calculate A^T A for [[a,b],[c,d]]: A^T A = [[a²+c², ab+cd], [ab+cd, b²+d²]]
    T aa = a * a + c * c;
    T bb = b * b + d * d;
    T ab = a * b + c * d;
    
    // Eigenvalues of A^T A: λ = (trace ± sqrt(trace² - 4*det)) / 2
    T trace = aa + bb;
    T det_ata = aa * bb - ab * ab;
    
    // Check for singular matrix
    T eps = epsilon<T>() * trace * trace;
    if (det_ata <= eps || det_ata <= T(0))
        return std::numeric_limits<T>::infinity();
    
    T discriminant = trace * trace - T(4) * det_ata;
    // Protect against numerical noise making discriminant negative
    if (discriminant < T(0))
        discriminant = T(0);
    
    T sqrt_d = std::sqrt(discriminant);
    T lambda_max = (trace + sqrt_d) / T(2);
    T lambda_min = (trace - sqrt_d) / T(2);
    
    // Check if minimum eigenvalue is too small
    if (lambda_min <= eps || lambda_min <= T(0))
        return std::numeric_limits<T>::infinity();
    
    // Condition number κ = σ_max / σ_min = sqrt(λ_max / λ_min)
    T ratio = lambda_max / lambda_min;
    return std::sqrt(ratio);
}

/// @brief Numerically stable dot product using Kahan summation
/// @tparam T Floating-point type
/// @param a Array of values
/// @param b Array of values
/// @param n Number of elements
/// @return Dot product computed with reduced error
template<typename T>
T stable_dot_product(const T* a, const T* b, std::uint32_t n) noexcept
{
    static_assert(std::is_floating_point<T>::value, "stable_dot_product requires floating-point type");

    T sum = T(0);
    T c = T(0);  // Compensation

    for (std::uint32_t i = 0; i < n; ++i)
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
T horner_eval(const T* coeffs, std::uint32_t n, T x) noexcept
{
    static_assert(std::is_floating_point<T>::value, "horner_eval requires floating-point type");

    if (n == 0)
        return T(0);

    T result = coeffs[n - 1];
    for (int i = static_cast<int>(n) - 2; i >= 0; --i)
    {
        result = result * x + coeffs[i];
    }

    return result;
}

/// @brief Check if matrix is likely to be ill-conditioned
/// @param condition_number Estimated condition number
/// @param threshold Warning threshold (default: 1e10)
/// @return true if matrix appears ill-conditioned
template<typename T>
bool is_ill_conditioned(T condition_number, T threshold = T(1e10)) noexcept
{
    return condition_number > threshold || !std::isfinite(condition_number);
}

/// @brief Estimate loss of precision in bits
/// @param condition_number Condition number of problem
/// @return Approximate number of bits of precision lost
template<typename T>
int precision_loss_bits(T condition_number) noexcept
{
    if (condition_number <= T(1))
        return 0;

    // log2(condition_number) gives bits of precision lost
    return static_cast<int>(std::log2(condition_number));
}

/// @brief Compute ULP (Units in Last Place) distance between two floats
/// @details Measures how many representable floats are between two values
/// @param a First value
/// @param b Second value
/// @return ULP distance
template<typename T>
std::int64_t ulp_distance(T a, T b) noexcept
{
    static_assert(std::is_floating_point<T>::value, "ulp_distance requires floating-point type");

    // For simplicity, use relative difference scaled by epsilon
    if (a == b)
        return 0;

    T diff = std::abs(a - b);
    T eps = epsilon<T>();
    T scale = std::max(std::abs(a), std::abs(b));

    return static_cast<std::int64_t>(diff / (scale * eps));
}

}  // namespace numerical
}  // namespace microla

#endif  // MICROLA_NUMERICAL_STABILITY_HPP_
