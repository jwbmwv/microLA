// test_safe_math.cpp - Unit tests for safe_math.hpp
#include <gtest/gtest.h>
#include "../../include/microla/safe_math.hpp"
#include <cmath>
#include <limits>

using namespace microla;
using namespace microla::safe;

// Test safe_divide
TEST(SafeMath, SafeDivideNormalCases)
{
    float result = safe_divide(10.0f, 2.0f);
    EXPECT_FLOAT_EQ(result, 5.0f);

    result = safe_divide(-10.0f, 2.0f);
    EXPECT_FLOAT_EQ(result, -5.0f);

    result = safe_divide(1.0f, 3.0f);
    EXPECT_FLOAT_EQ(result, 1.0f / 3.0f);
}

TEST(SafeMath, SafeDivideByZero)
{
    float result = safe_divide(10.0f, 0.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);  // Should fallback to 0
}

TEST(SafeMath, SafeDivideByVerySmall)
{
    float epsilon = std::numeric_limits<float>::epsilon();

    // Very small divisor should fall back to default
    float result = safe_divide(10.0f, epsilon * 0.1f);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST(SafeMath, SafeDivideWithCustomFallback)
{
    float result = safe_divide(10.0f, 0.0f, 42.0f);
    EXPECT_FLOAT_EQ(result, 42.0f);
}

// Test saturating_add
TEST(SafeMath, SaturatingAddNormal)
{
    EXPECT_EQ(saturating_add(10, 20), 30);
    EXPECT_EQ(saturating_add(-10, -20), -30);
    EXPECT_EQ(saturating_add(10, -5), 5);
}

TEST(SafeMath, SaturatingAddOverflow)
{
    int max = std::numeric_limits<int>::max();
    EXPECT_EQ(saturating_add(max, 1), max);
    EXPECT_EQ(saturating_add(max, 100), max);
    EXPECT_EQ(saturating_add(max / 2, max / 2 + 2), max);
}

TEST(SafeMath, SaturatingAddUnderflow)
{
    int min = std::numeric_limits<int>::min();
    EXPECT_EQ(saturating_add(min, -1), min);
    EXPECT_EQ(saturating_add(min, -100), min);
}

// Test saturating_sub
TEST(SafeMath, SaturatingSubNormal)
{
    EXPECT_EQ(saturating_sub(20, 10), 10);
    EXPECT_EQ(saturating_sub(10, 20), -10);
    EXPECT_EQ(saturating_sub(-10, -20), 10);
}

TEST(SafeMath, SaturatingSubOverflow)
{
    int max = std::numeric_limits<int>::max();
    int min = std::numeric_limits<int>::min();
    EXPECT_EQ(saturating_sub(max, -1), max);
    EXPECT_EQ(saturating_sub(max, min), max);
}

TEST(SafeMath, SaturatingSubUnderflow)
{
    int min = std::numeric_limits<int>::min();
    int max = std::numeric_limits<int>::max();
    EXPECT_EQ(saturating_sub(min, 1), min);
    EXPECT_EQ(saturating_sub(min, max), min);
}

// Test saturating_mul
TEST(SafeMath, SaturatingMulNormal)
{
    EXPECT_EQ(saturating_mul(10, 20), 200);
    EXPECT_EQ(saturating_mul(-10, 20), -200);
    EXPECT_EQ(saturating_mul(-10, -20), 200);
    EXPECT_EQ(saturating_mul(0, 1000000), 0);
}

TEST(SafeMath, SaturatingMulOverflow)
{
    int max = std::numeric_limits<int>::max();
    EXPECT_EQ(saturating_mul(max, 2), max);
    EXPECT_EQ(saturating_mul(max / 2 + 1, 2), max);
    // 100,000 * 100,000 = 10,000,000,000 which exceeds INT_MAX (2,147,483,647)
    EXPECT_EQ(saturating_mul(100000, 100000), max);
}

TEST(SafeMath, SaturatingMulUnderflow)
{
    int min = std::numeric_limits<int>::min();
    EXPECT_EQ(saturating_mul(min, 2), min);
    // -100,000 * 100,000 = -10,000,000,000 which is below INT_MIN (-2,147,483,648)
    EXPECT_EQ(saturating_mul(-100000, 100000), min);
}

// Test safe_sqrt
TEST(SafeMath, SafeSqrtPositive)
{
    float result = safe_sqrt(4.0f);
    EXPECT_FLOAT_EQ(result, 2.0f);

    result = safe_sqrt(0.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);

    result = safe_sqrt(2.0f);
    EXPECT_NEAR(result, 1.414213f, 1e-5f);
}

TEST(SafeMath, SafeSqrtNegative)
{
    float result = safe_sqrt(-1.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);  // Fallback

    result = safe_sqrt(-100.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);  // Returns 0 for negative
}

// Test safe_acos
TEST(SafeMath, SafeAcosValidRange)
{
    float result = safe_acos(1.0f);
    EXPECT_NEAR(result, 0.0f, 1e-5f);

    result = safe_acos(-1.0f);
    EXPECT_NEAR(result, 3.14159f, 1e-5f);

    result = safe_acos(0.0f);
    EXPECT_NEAR(result, 1.5708f, 1e-4f);  // π/2

    result = safe_acos(0.5f);
    EXPECT_NEAR(result, 1.0472f, 1e-4f);  // π/3
}

TEST(SafeMath, SafeAcosOutOfRange)
{
    // Just above 1
    float result = safe_acos(1.0001f);
    EXPECT_NEAR(result, std::acos(1.0f), 1e-5f);  // Clamped to 1

    // Just below -1
    result = safe_acos(-1.0001f);
    EXPECT_NEAR(result, std::acos(-1.0f), 1e-5f);  // Clamped to -1

    // Far out of range
    result = safe_acos(2.0f);
    EXPECT_NEAR(result, 0.0f, 1e-5f);

    result = safe_acos(-2.0f);
    EXPECT_NEAR(result, 3.14159f, 1e-5f);
}

// Test safe_asin
TEST(SafeMath, SafeAsinValidRange)
{
    float result = safe_asin(0.0f);
    EXPECT_NEAR(result, 0.0f, 1e-5f);

    result = safe_asin(1.0f);
    EXPECT_NEAR(result, 1.5708f, 1e-4f);  // π/2

    result = safe_asin(-1.0f);
    EXPECT_NEAR(result, -1.5708f, 1e-4f);  // -π/2

    result = safe_asin(0.5f);
    EXPECT_NEAR(result, 0.5236f, 1e-4f);  // π/6
}

TEST(SafeMath, SafeAsinOutOfRange)
{
    // Just above 1
    float result = safe_asin(1.0001f);
    EXPECT_NEAR(result, std::asin(1.0f), 1e-5f);  // Clamped to 1

    // Just below -1
    result = safe_asin(-1.0001f);
    EXPECT_NEAR(result, std::asin(-1.0f), 1e-5f);  // Clamped to -1

    // Far out of range
    result = safe_asin(2.0f);
    EXPECT_NEAR(result, 1.5708f, 1e-4f);

    result = safe_asin(-2.0f);
    EXPECT_NEAR(result, -1.5708f, 1e-4f);
}

// Test is_finite
TEST(SafeMath, IsFinite)
{
    EXPECT_TRUE(is_finite(0.0f));
    EXPECT_TRUE(is_finite(1.0f));
    EXPECT_TRUE(is_finite(-1.0f));
    EXPECT_TRUE(is_finite(1e10f));
    EXPECT_TRUE(is_finite(-1e10f));

    EXPECT_FALSE(is_finite(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_finite(-std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_finite(std::numeric_limits<float>::quiet_NaN()));
}

// Test is_nan
TEST(SafeMath, IsNan)
{
    EXPECT_FALSE(is_nan(0.0f));
    EXPECT_FALSE(is_nan(1.0f));
    EXPECT_FALSE(is_nan(-1.0f));
    EXPECT_FALSE(is_nan(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_nan(-std::numeric_limits<float>::infinity()));

    EXPECT_TRUE(is_nan(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_TRUE(is_nan(std::numeric_limits<float>::signaling_NaN()));

    // NaN from invalid operation
    float nan_result = std::sqrt(-1.0f);
    EXPECT_TRUE(is_nan(nan_result));
}

// Test double precision
TEST(SafeMath, DoublePrecision)
{
    double result = safe_divide(10.0, 2.0);
    EXPECT_DOUBLE_EQ(result, 5.0);

    result = safe_sqrt(4.0);
    EXPECT_DOUBLE_EQ(result, 2.0);

    EXPECT_TRUE(is_finite(1.0));
    EXPECT_FALSE(is_finite(std::numeric_limits<double>::infinity()));
}
