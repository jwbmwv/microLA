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
    float result = safe_divide(10.0F, 2.0F);
    EXPECT_FLOAT_EQ(result, 5.0F);

    result = safe_divide(-10.0F, 2.0F);
    EXPECT_FLOAT_EQ(result, -5.0F);

    result = safe_divide(1.0F, 3.0F);
    EXPECT_FLOAT_EQ(result, 1.0F / 3.0F);
}

TEST(SafeMath, SafeDivideByZero)
{
    float result = safe_divide(10.0F, 0.0F);
    EXPECT_FLOAT_EQ(result, 0.0F);  // Should fallback to 0
}

TEST(SafeMath, SafeDivideByVerySmall)
{
    float epsilon = std::numeric_limits<float>::epsilon();

    // Very small divisor should fall back to default
    float result = safe_divide(10.0F, epsilon * 0.1F);
    EXPECT_FLOAT_EQ(result, 0.0F);
}

TEST(SafeMath, SafeDivideWithCustomFallback)
{
    float result = safe_divide(10.0F, 0.0F, 42.0F);
    EXPECT_FLOAT_EQ(result, 42.0F);
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
    float result = safe_sqrt(4.0F);
    EXPECT_FLOAT_EQ(result, 2.0F);

    result = safe_sqrt(0.0F);
    EXPECT_FLOAT_EQ(result, 0.0F);

    result = safe_sqrt(2.0F);
    EXPECT_NEAR(result, 1.414213F, 1e-5F);
}

TEST(SafeMath, SafeSqrtNegative)
{
    float result = safe_sqrt(-1.0F);
    EXPECT_TRUE(std::isnan(result));  // Returns NaN for error detection

    result = safe_sqrt(-100.0F);
    EXPECT_TRUE(std::isnan(result));  // Returns NaN for negative
}

// Test safe_acos
TEST(SafeMath, SafeAcosValidRange)
{
    float result = safe_acos(1.0F);
    EXPECT_NEAR(result, 0.0F, 1e-5F);

    result = safe_acos(-1.0F);
    EXPECT_NEAR(result, 3.14159F, 1e-5F);

    result = safe_acos(0.0F);
    EXPECT_NEAR(result, 1.5708F, 1e-4F);  // π/2

    result = safe_acos(0.5F);
    EXPECT_NEAR(result, 1.0472F, 1e-4F);  // π/3
}

TEST(SafeMath, SafeAcosOutOfRange)
{
    // Just above 1
    float result = safe_acos(1.0001F);
    EXPECT_NEAR(result, std::acos(1.0F), 1e-5F);  // Clamped to 1

    // Just below -1
    result = safe_acos(-1.0001F);
    EXPECT_NEAR(result, std::acos(-1.0F), 1e-5F);  // Clamped to -1

    // Far out of range
    result = safe_acos(2.0F);
    EXPECT_NEAR(result, 0.0F, 1e-5F);

    result = safe_acos(-2.0F);
    EXPECT_NEAR(result, 3.14159F, 1e-5F);
}

// Test safe_asin
TEST(SafeMath, SafeAsinValidRange)
{
    float result = safe_asin(0.0F);
    EXPECT_NEAR(result, 0.0F, 1e-5F);

    result = safe_asin(1.0F);
    EXPECT_NEAR(result, 1.5708F, 1e-4F);  // π/2

    result = safe_asin(-1.0F);
    EXPECT_NEAR(result, -1.5708F, 1e-4F);  // -π/2

    result = safe_asin(0.5F);
    EXPECT_NEAR(result, 0.5236F, 1e-4F);  // π/6
}

TEST(SafeMath, SafeAsinOutOfRange)
{
    // Just above 1
    float result = safe_asin(1.0001F);
    EXPECT_NEAR(result, std::asin(1.0F), 1e-5F);  // Clamped to 1

    // Just below -1
    result = safe_asin(-1.0001F);
    EXPECT_NEAR(result, std::asin(-1.0F), 1e-5F);  // Clamped to -1

    // Far out of range
    result = safe_asin(2.0F);
    EXPECT_NEAR(result, 1.5708F, 1e-4F);

    result = safe_asin(-2.0F);
    EXPECT_NEAR(result, -1.5708F, 1e-4F);
}

// Test is_finite
TEST(SafeMath, IsFinite)
{
    EXPECT_TRUE(is_finite(0.0F));
    EXPECT_TRUE(is_finite(1.0F));
    EXPECT_TRUE(is_finite(-1.0F));
    EXPECT_TRUE(is_finite(1e10F));
    EXPECT_TRUE(is_finite(-1e10F));

    EXPECT_FALSE(is_finite(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_finite(-std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_finite(std::numeric_limits<float>::quiet_NaN()));
}

// Test is_nan
TEST(SafeMath, IsNan)
{
    EXPECT_FALSE(is_nan(0.0F));
    EXPECT_FALSE(is_nan(1.0F));
    EXPECT_FALSE(is_nan(-1.0F));
    EXPECT_FALSE(is_nan(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_nan(-std::numeric_limits<float>::infinity()));

    EXPECT_TRUE(is_nan(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_TRUE(is_nan(std::numeric_limits<float>::signaling_NaN()));

    // NaN value for testing (avoid calling std::sqrt with negative literal)
    float nan_result = std::numeric_limits<float>::quiet_NaN();
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
