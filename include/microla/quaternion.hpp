// SPDX-License-Identifier: MIT
/// @file quaternion.hpp
/// @brief Quaternion class for 3D rotations with SIMD optimizations
/// @details Provides a complete quaternion implementation optimized for embedded systems,
///          with Vec<T,3> integration for efficient operations and CMSIS-DSP support.
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef _MICROLA_QUATERNION_HPP_
#define _MICROLA_QUATERNION_HPP_

#include "vector.hpp"

namespace microla
{

/// @class Quaternion
/// @brief A quaternion class for 3D rotations, templated on type T
/// @tparam T The storage type (typically float or double)
template<typename T = float>
class Quaternion
{
public:
    /// @brief Component indices (shared with Vec class)
    enum
    {
        X = 0,
        Y = 1,
        Z = 2,
        W = 3
    };
    /// @brief Euler angle indices for Vec<T,3> returned by to_euler()
    enum
    {
        ROLL = 0,
        PITCH = 1,
        YAW = 2
    };
    // Imaginary part (x,y,z) first enables Vec<T,3> optimizations and better SIMD efficiency
    alignas(16) T data[4];  // x, y, z, w

    /// \brief Default constructor (identity quaternion).
    Quaternion() : data{T(0), T(0), T(0), T(1)} {}

    /// \brief Constructor from components.
    /// \param w The real part.
    /// \param x The i component.
    /// \param y The j component.
    /// \param z The k component.
    MICROLA_CONSTEXPR Quaternion(T w, T x, T y, T z) : data{x, y, z, w} {}

    /// \brief Constructor from array.
    /// \param arr The array [x, y, z, w].
    MICROLA_CONSTEXPR explicit Quaternion(const T* arr) : data{arr[0], arr[1], arr[2], arr[3]} {}

    /// \brief Constructor from axis-angle.
    /// \param axis The rotation axis (must be normalized).
    /// \param angle The rotation angle in radians.
    Quaternion(const Vec<T, 3>& axis, T angle)
    {
        const T half_angle = angle * T(0.5);
        const T s = std::sin(half_angle);
        // Use Vec operations to compute imaginary part: xyz = axis * s
        const Vec<T, 3> imaginary = axis * s;
        data[X] = imaginary[X];
        data[Y] = imaginary[Y];
        data[Z] = imaginary[Z];
        data[W] = std::cos(half_angle);
    }

    /// \brief Constructor from rotation matrix.
    /// \param mat The 3x3 rotation matrix.
    explicit Quaternion(const Mat<T, 3, 3>& mat)
    {
        const T trace = mat[0][0] + mat[1][1] + mat[2][2];
        if (trace > T(0))
        {
            T s = std::sqrt(trace + T(1)) * T(2);
            data[X] = (mat[1][2] - mat[2][1]) / s;
            data[Y] = (mat[2][0] - mat[0][2]) / s;
            data[Z] = (mat[0][1] - mat[1][0]) / s;
            data[W] = s * T(0.25);
        }
        else if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2])
        {
            T s = std::sqrt(T(1) + mat[0][0] - mat[1][1] - mat[2][2]) * T(2);
            data[X] = s * T(0.25);
            data[Y] = (mat[0][1] + mat[1][0]) / s;
            data[Z] = (mat[2][0] + mat[0][2]) / s;
            data[W] = (mat[1][2] - mat[2][1]) / s;
        }
        else if (mat[1][1] > mat[2][2])
        {
            T s = std::sqrt(T(1) + mat[1][1] - mat[0][0] - mat[2][2]) * T(2);
            data[X] = (mat[0][1] + mat[1][0]) / s;
            data[Y] = s * T(0.25);
            data[Z] = (mat[1][2] + mat[2][1]) / s;
            data[W] = (mat[2][0] - mat[0][2]) / s;
        }
        else
        {
            T s = std::sqrt(T(1) + mat[2][2] - mat[0][0] - mat[1][1]) * T(2);
            data[X] = (mat[2][0] + mat[0][2]) / s;
            data[Y] = (mat[1][2] + mat[2][1]) / s;
            data[Z] = s * T(0.25);
            data[W] = (mat[0][1] - mat[1][0]) / s;
        }
    }

    /// \brief Copy constructor.
    /// \param other The quaternion to copy.
    MICROLA_CONSTEXPR Quaternion(const Quaternion& other) : data{other[X], other[Y], other[Z], other[W]} {}

    /// \brief Subscript operator.
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Reference to the component.
    T& operator[](std::uint32_t index) noexcept { return data[index]; }

    /// \brief Subscript operator (const).
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Const reference to the component.
    const T& operator[](std::uint32_t index) const noexcept { return data[index]; }

    /// \brief Bounds-checked element access.
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Reference to the component.
    /// \note In debug builds (MICROLA_DEBUG defined), triggers assertion if index >= 4.
    T& at(std::uint32_t index)
    {
#ifdef MICROLA_DEBUG
        assert(index < 4 && "Quaternion::at: index out of range");
#endif
        return data[index];
    }

    /// \brief Bounds-checked element access (const).
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Const reference to the component.
    /// \note In debug builds (MICROLA_DEBUG defined), triggers assertion if index >= 4.
    const T& at(std::uint32_t index) const
    {
#ifdef MICROLA_DEBUG
        assert(index < 4 && "Quaternion::at: index out of range");
#endif
        return data[index];
    }

    /// \brief Addition operator.
    /// \param other The quaternion to add.
    /// \return The sum.
    MICROLA_NODISCARD Quaternion operator+(const Quaternion& other) const noexcept
    {
#ifdef CONFIG_MICROLA_RISCV
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            size_t vl = vsetvl_e32m1(4);
            vfloat32m1_t a = vle32_v_f32m1(reinterpret_cast<const float*>(data), vl);
            vfloat32m1_t b = vle32_v_f32m1(reinterpret_cast<const float*>(other.data), vl);
            vfloat32m1_t r = vfadd_vv_f32m1(a, b, vl);
            vse32_v_f32m1(reinterpret_cast<float*>(result.data), r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t r = vaddq_f32(a, b);
            vst1q_f32(reinterpret_cast<float*>(result.data), r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            arm_add_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                        reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        // Use Vec operations for imaginary part
        const Vec<T, 3> imag_sum = vec() + other.vec();
        return Quaternion((*this)[W] + other[W], imag_sum[X], imag_sum[Y], imag_sum[Z]);
    }

    /// \brief Subtraction operator.
    /// \param other The quaternion to subtract.
    /// \return The difference.
    MICROLA_NODISCARD Quaternion operator-(const Quaternion& other) const noexcept
    {
#ifdef CONFIG_MICROLA_RISCV
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            size_t vl = vsetvl_e32m1(4);
            vfloat32m1_t a = vle32_v_f32m1(reinterpret_cast<const float*>(data), vl);
            vfloat32m1_t b = vle32_v_f32m1(reinterpret_cast<const float*>(other.data), vl);
            vfloat32m1_t r = vfsub_vv_f32m1(a, b, vl);
            vse32_v_f32m1(reinterpret_cast<float*>(result.data), r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t r = vsubq_f32(a, b);
            vst1q_f32(reinterpret_cast<float*>(result.data), r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            arm_sub_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                        reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        // Use Vec operations for imaginary part
        const Vec<T, 3> imag_diff = vec() - other.vec();
        return Quaternion((*this)[W] - other[W], imag_diff[X], imag_diff[Y], imag_diff[Z]);
    }

    /// \brief Unary minus operator.
    /// \return The negated quaternion.
    MICROLA_NODISCARD Quaternion operator-() const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t r = vnegq_f32(a);
            vst1q_f32(reinterpret_cast<float*>(result.data), r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            arm_negate_f32(reinterpret_cast<const float*>(data), reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        // Use Vec operations for imaginary part
        const Vec<T, 3> imag_neg = -vec();
        return Quaternion(-(*this)[W], imag_neg[X], imag_neg[Y], imag_neg[Z]);
    }

    /// \brief Scalar multiplication operator.
    /// \param scalar The scalar.
    /// \return The result.
    MICROLA_NODISCARD Quaternion operator*(T scalar) const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t r = vmulq_f32(a, s);
            vst1q_f32(reinterpret_cast<float*>(result.data), r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            arm_scale_f32(reinterpret_cast<const float*>(data), static_cast<float>(scalar),
                          reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        // Use Vec operations for imaginary part
        const Vec<T, 3> imag_scaled = vec() * scalar;
        return Quaternion((*this)[W] * scalar, imag_scaled[X], imag_scaled[Y], imag_scaled[Z]);
    }

    /// \brief Scalar division operator.
    /// \param scalar The scalar.
    /// \return The result.
    MICROLA_NODISCARD MICROLA_CONSTEXPR Quaternion operator/(T scalar) const noexcept
    {
        // Check for zero to avoid undefined behavior (minimal overhead)
        if (scalar == T(0))
        {
            return Quaternion(T(0), T(0), T(0), T(0));
        }
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            const float inv_scalar = 1.0f / static_cast<float>(scalar);
            arm_scale_f32(reinterpret_cast<const float*>(data), inv_scalar, reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        const T inv = T(1) / scalar;
        return *this * inv;
    }

    /// \brief Quaternion multiplication.
    /// \param other The other quaternion.
    /// \return The product.
    MICROLA_NODISCARD Quaternion operator*(const Quaternion& other) const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        MICROLA_IF_CONSTEXPR(std::is_same<T, float>::value)
        {
            Quaternion result;
            // Load quaternions: data layout is [x, y, z, w]
            float32x4_t q1 = vld1q_f32(reinterpret_cast<const float*>(data));        // [x1, y1, z1, w1]
            float32x4_t q2 = vld1q_f32(reinterpret_cast<const float*>(other.data));  // [x2, y2, z2, w2]

            // Quaternion multiplication formulas:
            // result.x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            // result.y = w1*y2 + y1*w2 + z1*x2 - x1*z2
            // result.z = w1*z2 + z1*w2 + x1*y2 - y1*x2
            // result.w = w1*w2 - x1*x2 - y1*y2 - z1*z2

            // Broadcast each component of q1 (ARMv7 compatible)
            float32x2_t q1_low = vget_low_f32(q1);
            float32x2_t q1_high = vget_high_f32(q1);
            float32x4_t q1_xxxx = vdupq_lane_f32(q1_low, 0);   // [x1, x1, x1, x1]
            float32x4_t q1_yyyy = vdupq_lane_f32(q1_low, 1);   // [y1, y1, y1, y1]
            float32x4_t q1_zzzz = vdupq_lane_f32(q1_high, 0);  // [z1, z1, z1, z1]
            float32x4_t q1_wwww = vdupq_lane_f32(q1_high, 1);  // [w1, w1, w1, w1]

            // Compute: w1*q2
            float32x4_t t0 = vmulq_f32(q1_wwww, q2);  // [w1*x2, w1*y2, w1*z2, w1*w2]

            // Shuffle q2 for x1 terms: [w2, z2, y2, x2]
            float32x4_t q2_wzyx = vrev64q_f32(q2);                                  // [y2, x2, w2, z2]
            q2_wzyx = vcombine_f32(vget_high_f32(q2_wzyx), vget_low_f32(q2_wzyx));  // [w2, z2, y2, x2]
            float32x4_t t1 = vmulq_f32(q1_xxxx, q2_wzyx);                           // [x1*w2, x1*z2, x1*y2, x1*x2]

            // Shuffle q2 for y1 terms: [z2, w2, x2, y2]
            float32x4_t q2_zwxy = vextq_f32(q2, q2, 2);    // [z2, w2, x2, y2]
            float32x4_t t2 = vmulq_f32(q1_yyyy, q2_zwxy);  // [y1*z2, y1*w2, y1*x2, y1*y2]

            // Shuffle q2 for z1 terms: [y2, x2, w2, z2]
            float32x4_t q2_yxwz = vrev64q_f32(q2);         // [y2, x2, w2, z2]
            float32x4_t t3 = vmulq_f32(q1_zzzz, q2_yxwz);  // [z1*y2, z1*x2, z1*w2, z1*z2]

            // Combine with correct signs: [+,+,+,-] for t1, [+,+,-,-] for t2, [-,+,+,-] for t3
            float32x4_t sign1 = {1.0f, -1.0f, 1.0f, -1.0f};
            float32x4_t sign2 = {1.0f, 1.0f, -1.0f, -1.0f};
            float32x4_t sign3 = {-1.0f, 1.0f, 1.0f, -1.0f};

            t1 = vmulq_f32(t1, sign1);
            t2 = vmulq_f32(t2, sign2);
            t3 = vmulq_f32(t3, sign3);

            // Sum all terms
            float32x4_t r = vaddq_f32(t0, t1);
            r = vaddq_f32(r, t2);
            r = vaddq_f32(r, t3);

            vst1q_f32(reinterpret_cast<float*>(result.data), r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            arm_quaternion_product_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                                       reinterpret_cast<float*>(result.data));
            return result;
        }
#endif
        return Quaternion((*this)[W] * other[W] - (*this)[X] * other[X] - (*this)[Y] * other[Y] - (*this)[Z] * other[Z],
                          (*this)[W] * other[X] + (*this)[X] * other[W] + (*this)[Y] * other[Z] - (*this)[Z] * other[Y],
                          (*this)[W] * other[Y] - (*this)[X] * other[Z] + (*this)[Y] * other[W] + (*this)[Z] * other[X],
                          (*this)[W] * other[Z] + (*this)[X] * other[Y] - (*this)[Y] * other[X] +
                              (*this)[Z] * other[W]);
    }

    /// \brief Rotate a 3D vector by this quaternion.
    /// \param v The vector to rotate.
    /// \return The rotated vector.
    MICROLA_NODISCARD Vec<T, 3> operator*(const Vec<T, 3>& v) const noexcept
    {
        // Use the formula: v' = q * v * q^-1
        // Optimized version: v' = v + 2 * cross(q.xyz, cross(q.xyz, v) + q.w * v)
        const Vec<T, 3> qvec = vec();
        const Vec<T, 3> uv = qvec.cross(v);
        const Vec<T, 3> uuv = qvec.cross(uv);
        return v + ((uv * (*this)[W]) + uuv) * T(2);
    }

    /// \brief Equality operator.
    /// \param other The quaternion to compare.
    /// \return True if equal (uses epsilon comparison for floating point types).
    MICROLA_CONSTEXPR bool operator==(const Quaternion& other) const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        MICROLA_IF_CONSTEXPR(std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t diff = vabdq_f32(a, b);
            float32x4_t eps = vdupq_n_f32(std::numeric_limits<float>::epsilon());
            uint32x4_t cmp = vcleq_f32(diff, eps);
            uint64x2_t cmp64 = vreinterpretq_u64_u32(cmp);
            return vgetq_lane_u64(cmp64, 0) == ~0ULL && vgetq_lane_u64(cmp64, 1) == ~0ULL;
        }
#endif
        // For floating point types, use epsilon comparison
        MICROLA_IF_CONSTEXPR(std::is_floating_point<T>::value)
        {
            return std::abs((*this)[X] - other[X]) <= std::numeric_limits<T>::epsilon() &&
                   std::abs((*this)[Y] - other[Y]) <= std::numeric_limits<T>::epsilon() &&
                   std::abs((*this)[Z] - other[Z]) <= std::numeric_limits<T>::epsilon() &&
                   std::abs((*this)[W] - other[W]) <= std::numeric_limits<T>::epsilon();
        }
        // For integral types, use exact comparison
        return (*this)[X] == other[X] && (*this)[Y] == other[Y] && (*this)[Z] == other[Z] && (*this)[W] == other[W];
    }

    /// \brief Inequality operator.
    /// \param other The quaternion to compare.
    /// \return True if not equal.
    bool operator!=(const Quaternion& other) const noexcept { return !(*this == other); }

    /// \brief Conjugate of the quaternion.
    /// \return The conjugate.
    MICROLA_NODISCARD Quaternion conjugate() const noexcept
    {
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            Quaternion result;
            // Copy w component
            result.data[W] = data[W];
            // Negate imaginary components (x, y, z)
            arm_negate_f32(reinterpret_cast<const float*>(&data[X]), reinterpret_cast<float*>(&result.data[X]), 3);
            return result;
        }
#endif
        // Use Vec operations for negating imaginary part
        const Vec<T, 3> imag_neg = -vec();
        return Quaternion((*this)[W], imag_neg[X], imag_neg[Y], imag_neg[Z]);
    }

    /// \brief Norm (magnitude) of the quaternion.
    /// \return The norm.
    /// \details In C++26+, this function is constexpr when the quaternion is known at compile-time.
    MICROLA_CONSTEXPR26 T norm() const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return std::sqrt(static_cast<T>(vget_lane_f32(sum, 0)));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            float result;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(data), 4, &result);
            return std::sqrt(static_cast<T>(result));
        }
#endif
        // Use Vec<T,3> dot product for imaginary part + w^2
        const Vec<T, 3> imag = vec();
        return std::sqrt(imag.dot(imag) + (*this)[W] * (*this)[W]);
    }

    /// \brief Normalized quaternion.
    /// \return The unit quaternion.
    Quaternion normalized() const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            // Compute norm squared
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            float norm_sq = vget_lane_f32(sum, 0);

            if (norm_sq == 0.0f)
                return *this;

            // Fast inverse square root with Newton-Raphson refinement
            float32x2_t norm_sq_v = vdup_n_f32(norm_sq);
            float32x2_t rsqrt = vrsqrte_f32(norm_sq_v);
            rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(norm_sq_v, rsqrt), rsqrt));
            rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(norm_sq_v, rsqrt), rsqrt));

            float inv_norm = vget_lane_f32(rsqrt, 0);
            return *this * static_cast<T>(inv_norm);
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            float norm_sq;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(data), 4, &norm_sq);

            if (norm_sq == 0.0f)
                return *this;

            float n;
            arm_sqrt_f32(norm_sq, &n);

            const float inv_norm = 1.0f / n;
            Quaternion result;
            arm_scale_f32(reinterpret_cast<const float*>(data), inv_norm, reinterpret_cast<float*>(result.data), 4);
            return result;
        }
#endif
        const T n = norm();
        if (n == T(0))
            return *this;
        return *this / n;
    }

    /// \brief Squared magnitude of the quaternion.
    /// \return The squared magnitude.
    MICROLA_NODISCARD T norm_squared() const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            float result;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(data), 4, &result);
            return static_cast<T>(result);
        }
#endif
        const Vec<T, 3> imag = vec();
        return imag.dot(imag) + (*this)[W] * (*this)[W];
    }

    /// \brief Normalize this quaternion in-place.
    void normalize() noexcept
    {
        const T n = norm();
        if (n != T(0))
        {
            *this = *this / n;
        }
    }

    /// \brief Angle between two quaternions.
    /// \param other The other quaternion.
    /// \return The angle in radians.
    MICROLA_NODISCARD T angle_to(const Quaternion& other) const noexcept
    {
        const T d = dot(other);
        // Clamp to handle numerical errors
        const T clamped = (d < T(-1)) ? T(-1) : ((d > T(1)) ? T(1) : d);
        return std::acos(clamped) * T(2);
    }

    /// \brief Create quaternion from two vectors.
    /// \param from The starting vector.
    /// \param to The ending vector.
    /// \return The rotation quaternion.
    MICROLA_NODISCARD static Quaternion from_two_vectors(const Vec<T, 3>& from, const Vec<T, 3>& to) noexcept
    {
        const Vec<T, 3> f = from.normalized();
        const Vec<T, 3> t = to.normalized();
        const T d = f.dot(t);

        // Vectors are the same
        if (d >= T(1) - T(1e-6))
        {
            return Quaternion(T(1), T(0), T(0), T(0));
        }

        // Vectors are opposite
        if (d <= T(-1) + T(1e-6))
        {
            // Find an orthogonal axis
            Vec<T, 3> axis = Vec<T, 3>(T(1), T(0), T(0)).cross(f);
            if (axis.dot(axis) < T(1e-6))
            {
                axis = Vec<T, 3>(T(0), T(1), T(0)).cross(f);
            }
            return from_axis_angle(axis.normalized(), constants::pi<T>());
        }

        // Normal case
        const Vec<T, 3> axis = f.cross(t);
        const T s = std::sqrt((T(1) + d) * T(2));
        const T inv_s = T(1) / s;

        return Quaternion(s * T(0.5), axis[0] * inv_s, axis[1] * inv_s, axis[2] * inv_s);
    }

    /// \brief Inverse of the quaternion.
    /// \return The inverse.
    MICROLA_NODISCARD Quaternion inverse() const noexcept
    {
        // Compute norm squared directly to avoid redundant sqrt
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            const T n2 = static_cast<T>(vget_lane_f32(sum, 0));
            return conjugate() / n2;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            float result;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(data), 4, &result);
            return conjugate() / static_cast<T>(result);
        }
#endif
        const Vec<T, 3> imag = vec();
        const T n2 = imag.dot(imag) + (*this)[W] * (*this)[W];
        // Check for zero quaternion (minimal overhead)
        if (n2 == T(0))
        {
            return Quaternion(T(1), T(0), T(0), T(0));  // Return identity quaternion
        }
        return conjugate() / n2;
    }

    /// \brief Convert to 3x3 rotation matrix.
    /// \return The rotation matrix.
    Mat<T, 3, 3> to_matrix() const noexcept
    {
        const T w = (*this)[W], x = (*this)[X], y = (*this)[Y], z = (*this)[Z];
        const T xx = x * x, yy = y * y, zz = z * z;
        const T xy = x * y, xz = x * z, yz = y * z;
        const T wx = w * x, wy = w * y, wz = w * z;
        Mat<T, 3, 3> mat;
        mat[0][0] = T(1) - T(2) * (yy + zz);
        mat[0][1] = T(2) * (xy - wz);
        mat[0][2] = T(2) * (xz + wy);
        mat[1][0] = T(2) * (xy + wz);
        mat[1][1] = T(1) - T(2) * (xx + zz);
        mat[1][2] = T(2) * (yz - wx);
        mat[2][0] = T(2) * (xz - wy);
        mat[2][1] = T(2) * (yz + wx);
        mat[2][2] = T(1) - T(2) * (xx + yy);
        return mat;
    }

    /// \brief Dot product.
    /// \param other The other quaternion.
    /// \return The dot product.
    T dot(const Quaternion& other) const noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t mul = vmulq_f32(a, b);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            float result;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data), 4,
                             &result);
            return static_cast<T>(result);
        }
#endif
        // Use Vec<T,3> dot product for imaginary part + w*w
        const Vec<T, 3> imag1 = vec();
        const Vec<T, 3> imag2 = other.vec();
        return imag1.dot(imag2) + (*this)[W] * other[W];
    }

    /// \brief Rotate a vector by this quaternion.
    /// \param v The vector to rotate.
    /// \return The rotated vector.
    Vec<T, 3> rotate(const Vec<T, 3>& v) const noexcept
    {
        // Optimized rotation: v' = v + 2*w*(xyz x v) + 2*(xyz x (xyz x v))
        const Vec<T, 3> imag = vec();
        const Vec<T, 3> cross1 = imag.cross(v);
        const Vec<T, 3> cross2 = imag.cross(cross1);
        return v + cross1 * (T(2) * (*this)[W]) + cross2 * T(2);
    }

    /// \brief Component accessors.
    T& w() noexcept { return data[W]; }
    const T& w() const noexcept { return data[W]; }
    T& x() noexcept { return data[X]; }
    const T& x() const noexcept { return data[X]; }
    T& y() noexcept { return data[Y]; }
    const T& y() const noexcept { return data[Y]; }
    T& z() noexcept { return data[Z]; }
    const T& z() const noexcept { return data[Z]; }

    /// \brief Get copy of imaginary part as Vec<T,3>.
    /// \return Copy of the imaginary components (x,y,z).
    /// \note Returns by value to avoid type-punning UB. Compilers optimize this to zero-cost.
    MICROLA_NODISCARD Vec<T, 3> vec() const noexcept { return Vec<T, 3>(data[X], data[Y], data[Z]); }

    /// \brief Set imaginary part from Vec<T,3>.
    /// \param v The vector to copy imaginary components from.
    void set_vec(const Vec<T, 3>& v) noexcept
    {
        data[X] = v[0];
        data[Y] = v[1];
        data[Z] = v[2];
    }

    /// \brief Addition assignment operator.
    /// \param other The quaternion to add.
    /// \return Reference to this.
    Quaternion& operator+=(const Quaternion& other) noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t r = vaddq_f32(a, b);
            vst1q_f32(reinterpret_cast<float*>(data), r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            arm_add_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                        reinterpret_cast<float*>(data), 4);
            return *this;
        }
#endif
        // Update components directly
        data[X] += other.data[X];
        data[Y] += other.data[Y];
        data[Z] += other.data[Z];
        data[W] += other.data[W];
        return *this;
    }

    /// \brief Subtraction assignment operator.
    /// \param other The quaternion to subtract.
    /// \return Reference to this.
    Quaternion& operator-=(const Quaternion& other) noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(other.data));
            float32x4_t r = vsubq_f32(a, b);
            vst1q_f32(reinterpret_cast<float*>(data), r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            arm_sub_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                        reinterpret_cast<float*>(data), 4);
            return *this;
        }
#endif
        // Update components directly
        data[X] -= other.data[X];
        data[Y] -= other.data[Y];
        data[Z] -= other.data[Z];
        data[W] -= other.data[W];
        return *this;
    }

    /// \brief Scalar multiplication assignment operator.
    /// \param scalar The scalar to multiply by.
    /// \return Reference to this.
    Quaternion& operator*=(T scalar) noexcept
    {
#ifdef CONFIG_MICROLA_NEON
        if (std::is_same<T, float>::value)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(data));
            float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t r = vmulq_f32(a, s);
            vst1q_f32(reinterpret_cast<float*>(data), r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if (std::is_same<T, float>::value)
        {
            arm_scale_f32(reinterpret_cast<const float*>(data), static_cast<float>(scalar),
                          reinterpret_cast<float*>(data), 4);
            return *this;
        }
#endif
        // Use Vec operations for imaginary part
        vec() *= scalar;
        (*this)[W] *= scalar;
        return *this;
    }

    /// \brief Scalar division assignment operator.
    /// \param scalar The scalar to divide by.
    /// \return Reference to this.
    MICROLA_CONSTEXPR Quaternion& operator/=(T scalar) noexcept
    {
        const T inv = T(1) / scalar;
        return *this *= inv;
    }

    /// \brief Quaternion multiplication assignment operator.
    /// \param other The quaternion to multiply by.
    /// \return Reference to this.
    Quaternion& operator*=(const Quaternion& other) noexcept
    {
        *this = *this * other;
        return *this;
    }

    /// \brief Approximate equality comparison.
    /// \param other The quaternion to compare.
    /// \param epsilon The tolerance (defaults to machine epsilon for type T).
    /// \return True if approximately equal.
    bool approx_equal(const Quaternion& other, T epsilon = std::numeric_limits<T>::epsilon()) const noexcept
    {
        // Use Vec operations for imaginary part comparison
        return vec().approx_equal(other.vec(), epsilon) && std::abs((*this)[W] - other[W]) <= epsilon;
    }

    /// \brief Linear interpolation (lerp) between two quaternions.
    /// \param other The target quaternion.
    /// \param t The interpolation parameter [0, 1].
    /// \return The interpolated quaternion (not normalized).
    Quaternion lerp(const Quaternion& other, T t) const noexcept
    {
        // Use Vec operations for imaginary part interpolation
        const Vec<T, 3> imag_lerp = vec() + (other.vec() - vec()) * t;
        return Quaternion((*this)[W] + t * (other[W] - (*this)[W]), imag_lerp[X], imag_lerp[Y], imag_lerp[Z]);
    }

    /// \brief Spherical linear interpolation (slerp) between two quaternions.
    /// \param other The target quaternion.
    /// \param t The interpolation parameter [0, 1].
    /// \return The interpolated quaternion.
    Quaternion slerp(const Quaternion& other, T t) const noexcept
    {
        T cos_theta = dot(other);

        // If quaternions are close, use linear interpolation
        if (std::abs(cos_theta) >= T(0.9995))
        {
            return lerp(other, t).normalized();
        }

        // Handle negative dot product (choose shorter path)
        Quaternion q2 = other;
        if (cos_theta < T(0))
        {
            q2 = -other;
            cos_theta = -cos_theta;
        }

        // Clamp to avoid numerical issues
        cos_theta = std::min(std::max(cos_theta, T(-1)), T(1));

        const T theta = std::acos(cos_theta);
        const T sin_theta = std::sin(theta);

        const T w1 = std::sin((T(1) - t) * theta) / sin_theta;
        const T w2 = std::sin(t * theta) / sin_theta;

        return Quaternion((*this)[W] * w1 + q2[W] * w2, (*this)[X] * w1 + q2[X] * w2, (*this)[Y] * w1 + q2[Y] * w2,
                          (*this)[Z] * w1 + q2[Z] * w2);
    }

    /// \brief Get raw pointer to data.
    /// \return Pointer to the underlying data array.
    T* ptr() noexcept { return data; }

    /// \brief Get raw pointer to data (const).
    /// \return Const pointer to the underlying data array.
    const T* ptr() const noexcept { return data; }

    /// \brief Get size of the quaternion.
    /// \return Always returns 4.
    MICROLA_CONSTEXPR std::uint32_t size() const noexcept { return 4; }

    /// \brief Create identity quaternion.
    /// \return The identity quaternion (0, 0, 0, 1).
    static MICROLA_CONSTEXPR Quaternion identity() noexcept { return Quaternion(T(1), T(0), T(0), T(0)); }

    /// \brief Create quaternion from axis-angle representation.
    /// \param axis The rotation axis (should be normalized).
    /// \param angle The rotation angle in radians.
    /// \return Quaternion representing the rotation.
    static Quaternion from_axis_angle(const Vec<T, 3>& axis, T angle) noexcept { return Quaternion(axis, angle); }

    /// \brief Convert quaternion to Euler angles (roll, pitch, yaw)
    /// \return Vec<T,3> containing (roll, pitch, yaw) in radians
    /// Roll (rotation about X), Pitch (rotation about Y), Yaw (rotation about Z)
    Vec<T, 3> to_euler() const noexcept
    {
        Vec<T, 3> euler;

        // Roll (X-axis rotation)
        const T sinr_cosp = T(2) * ((*this)[W] * (*this)[X] + (*this)[Y] * (*this)[Z]);
        const T cosr_cosp = T(1) - T(2) * ((*this)[X] * (*this)[X] + (*this)[Y] * (*this)[Y]);
        euler[ROLL] = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (Y-axis rotation)
        const T sinp = T(2) * ((*this)[W] * (*this)[Y] - (*this)[Z] * (*this)[X]);
        if (std::abs(sinp) >= T(1))
        {
            // Use 90 degrees if out of range (gimbal lock)
            euler[PITCH] = std::copysign(constants::half_pi<T>(), sinp);
        }
        else
        {
            euler[PITCH] = std::asin(sinp);
        }

        // Yaw (Z-axis rotation)
        const T siny_cosp = T(2) * ((*this)[W] * (*this)[Z] + (*this)[X] * (*this)[Y]);
        const T cosy_cosp = T(1) - T(2) * ((*this)[Y] * (*this)[Y] + (*this)[Z] * (*this)[Z]);
        euler[YAW] = std::atan2(siny_cosp, cosy_cosp);

        return euler;
    }

    /// \brief Create quaternion from Euler angles (roll, pitch, yaw)
    /// \param euler Vec<T,3> containing (roll, pitch, yaw) in radians
    /// \return Quaternion representing the rotation
    static Quaternion from_euler(const Vec<T, 3>& euler) noexcept
    {
        return from_euler(euler[ROLL], euler[PITCH], euler[YAW]);
    }

    /// \brief Create quaternion from Euler angles (roll, pitch, yaw)
    /// \param roll Rotation about X axis in radians
    /// \param pitch Rotation about Y axis in radians
    /// \param yaw Rotation about Z axis in radians
    /// \return Quaternion representing the rotation
    static Quaternion from_euler(T roll, T pitch, T yaw) noexcept
    {
        const T cr = std::cos(roll * T(0.5));
        const T sr = std::sin(roll * T(0.5));
        const T cp = std::cos(pitch * T(0.5));
        const T sp = std::sin(pitch * T(0.5));
        const T cy = std::cos(yaw * T(0.5));
        const T sy = std::sin(yaw * T(0.5));

        Quaternion q;
        q[W] = cr * cp * cy + sr * sp * sy;
        q[X] = sr * cp * cy - cr * sp * sy;
        q[Y] = cr * sp * cy + sr * cp * sy;
        q[Z] = cr * cp * sy - sr * sp * cy;

        return q;
    }

    /// \brief Extract roll angle (rotation about X axis)
    /// \return Roll angle in radians
    T roll() const noexcept
    {
        const T sinr_cosp = T(2) * ((*this)[W] * (*this)[X] + (*this)[Y] * (*this)[Z]);
        const T cosr_cosp = T(1) - T(2) * ((*this)[X] * (*this)[X] + (*this)[Y] * (*this)[Y]);
        return std::atan2(sinr_cosp, cosr_cosp);
    }

    /// \brief Extract pitch angle (rotation about Y axis)
    /// \return Pitch angle in radians
    T pitch() const noexcept
    {
        const T sinp = T(2) * ((*this)[W] * (*this)[Y] - (*this)[Z] * (*this)[X]);
        if (std::abs(sinp) >= T(1))
        {
            return std::copysign(constants::half_pi<T>(), sinp);
        }
        return std::asin(sinp);
    }

    /// \brief Extract yaw angle (rotation about Z axis)
    /// \return Yaw angle in radians
    T yaw() const noexcept
    {
        const T siny_cosp = T(2) * ((*this)[W] * (*this)[Z] + (*this)[X] * (*this)[Y]);
        const T cosy_cosp = T(1) - T(2) * ((*this)[Y] * (*this)[Y] + (*this)[Z] * (*this)[Z]);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

/// \brief Scalar multiplication (commutative).
/// \tparam T The type.
/// \param scalar The scalar.
/// \param q The quaternion.
/// \return The result.
template<typename T>
MICROLA_NODISCARD MICROLA_CONSTEXPR Quaternion<T> operator*(T scalar, const Quaternion<T>& q) noexcept
{
    return q * scalar;
}

/// @brief Alias for Quaternion<float>
using Quatf = Quaternion<float>;

}  // namespace microla

#endif  // _MICROLA_QUATERNION_HPP_
