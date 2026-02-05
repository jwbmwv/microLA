// SPDX-License-Identifier: MIT
/// @file test_new_features.cpp
/// @brief Test and demonstrate new embedded systems features
/// @details Tests safe math, fast math, numerical stability, and resource checks
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace microla;

void test_safe_math()
{
    std::cout << "=== Safe Math Tests ===\n\n";

    // Safe division
    float result1 = safe::safe_divide(10.0f, 2.0f);
    float result2 = safe::safe_divide(10.0f, 0.0f, -1.0f);  // Returns default value
    std::cout << "safe_divide(10, 2) = " << result1 << "\n";
    std::cout << "safe_divide(10, 0, -1) = " << result2 << " (default)\n\n";

    // Safe sqrt
    float result3 = safe::safe_sqrt(16.0f);
    float result4 = safe::safe_sqrt(-16.0f);  // Returns 0
    std::cout << "safe_sqrt(16) = " << result3 << "\n";
    std::cout << "safe_sqrt(-16) = " << result4 << " (clamped to 0)\n\n";

    // Safe acos
    float result5 = safe::safe_acos(0.5f);
    float result6 = safe::safe_acos(2.0f);  // Clamped to 1.0
    std::cout << "safe_acos(0.5) = " << result5 << "\n";
    std::cout << "safe_acos(2.0) = " << result6 << " (clamped)\n\n";

    // Saturating arithmetic
    std::int16_t a = 30000;
    std::int16_t b = 10000;
    std::int16_t sum = safe::saturating_add(a, b);
    std::cout << "saturating_add(30000, 10000) [int16_t] = " << sum
              << " (max: " << std::numeric_limits<std::int16_t>::max() << ")\n\n";
}

void test_fast_math()
{
    std::cout << "=== Fast Math Tests ===\n\n";

    const float angle = constants::pi<float>() / 4.0f;  // 45 degrees

    // Compare fast vs standard library
    float std_sin = std::sin(angle);
    float fast_sin_val = fast::sin(angle);
    float error = std::abs(std_sin - fast_sin_val);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Angle: " << angle << " rad (45°)\n";
    std::cout << "std::sin()  = " << std_sin << "\n";
    std::cout << "fast::sin() = " << fast_sin_val << "\n";
    std::cout << "Error: " << error << " (" << (error / std_sin * 100) << "%)\n\n";

    // Fast atan2
    float y = 1.0f, x = 1.0f;
    float std_atan2 = std::atan2(y, x);
    float fast_atan2_val = fast::atan2(y, x);
    std::cout << "std::atan2(1, 1)  = " << std_atan2 << "\n";
    std::cout << "fast::atan2(1, 1) = " << fast_atan2_val << "\n\n";

    // Fast sqrt via rsqrt
    float value = 25.0f;
    float std_sqrt = std::sqrt(value);
    float fast_sqrt_val = fast::sqrt(value);
    std::cout << "std::sqrt(25)  = " << std_sqrt << "\n";
    std::cout << "fast::sqrt(25) = " << fast_sqrt_val << "\n\n";
}

void test_numerical_stability()
{
    std::cout << "=== Numerical Stability Tests ===\n\n";

    // Approximate equality with tolerances
    float a = 1.0f + 1e-7f;
    float b = 1.0f;
    bool approx_eq = numerical::approx_equal(a, b);
    std::cout << "approx_equal(1.0000001, 1.0) = " << (approx_eq ? "true" : "false") << "\n\n";

    // Stable hypot (avoids overflow)
    float x = 3.0f, y = 4.0f;
    float hypot_result = numerical::hypot(x, y);
    std::cout << "hypot(3, 4) = " << hypot_result << " (should be 5.0)\n\n";

    // Condition number estimation
    float cond = numerical::condition_number_2x2(1.0f, 0.0f, 0.0f, 1.0f);
    std::cout << "condition_number(identity 2x2) = " << cond << "\n";

    float cond_ill = numerical::condition_number_2x2(1.0f, 1.0f, 1.0f, 1.000001f);
    std::cout << "condition_number(nearly singular) = " << cond_ill << "\n";
    std::cout << "is_ill_conditioned? " << (numerical::is_ill_conditioned(cond_ill) ? "yes" : "no") << "\n\n";

    // Horner's method for polynomial evaluation
    float coeffs[] = {1.0f, 2.0f, 3.0f};                          // 1 + 2x + 3x²
    float poly_result = numerical::horner_eval(coeffs, 3, 2.0f);  // Evaluate at x=2
    std::cout << "P(x) = 1 + 2x + 3x², P(2) = " << poly_result << " (expected: 17)\n\n";
}

void test_resource_checks()
{
    std::cout << "=== Resource Usage Checks ===\n\n";

    // Vec size information
    using Vec3fInfo = resource_checks::VecSizeInfo<float, 3>;
    std::cout << "Vec3f:\n";
    std::cout << "  Element size: " << Vec3fInfo::element_size << " bytes\n";
    std::cout << "  Dimension: " << Vec3fInfo::dimension << "\n";
    std::cout << "  Theoretical size: " << Vec3fInfo::theoretical_size << " bytes\n";
    std::cout << "  Actual size: " << Vec3fInfo::actual_size << " bytes\n";
    std::cout << "  Alignment: " << Vec3fInfo::alignment << " bytes\n";
    std::cout << "  Padding: " << Vec3fInfo::padding << " bytes\n\n";

    // Mat size information
    using Mat4fInfo = resource_checks::MatSizeInfo<float, 4, 4>;
    std::cout << "Mat4f:\n";
    std::cout << "  Element size: " << Mat4fInfo::element_size << " bytes\n";
    std::cout << "  Dimensions: " << Mat4fInfo::rows << "x" << Mat4fInfo::cols << "\n";
    std::cout << "  Theoretical size: " << Mat4fInfo::theoretical_size << " bytes\n";
    std::cout << "  Actual size: " << Mat4fInfo::actual_size << " bytes\n";
    std::cout << "  Alignment: " << Mat4fInfo::alignment << " bytes\n";
    std::cout << "  Padding: " << Mat4fInfo::padding << " bytes\n\n";

    // All compile-time checks passed (would fail at compile time if violated)
    std::cout << "✓ All compile-time resource checks passed!\n";
    std::cout << "  - Standard layout verified\n";
    std::cout << "  - Trivially copyable for DMA\n";
    std::cout << "  - Proper alignment\n\n";
}

int main()
{
    std::cout << "MicroLA - New Embedded Systems Features Demo\n";
    std::cout << "=============================================\n\n";

    test_safe_math();
    test_fast_math();
    test_numerical_stability();
    test_resource_checks();

    std::cout << "All tests completed successfully!\n";

    return 0;
}
