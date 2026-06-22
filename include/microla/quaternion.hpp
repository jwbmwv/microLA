// SPDX-License-Identifier: Apache-2.0
/// @file quaternion.hpp
/// @brief Quaternion class for 3D rotations with SIMD optimizations
/// @details Provides a complete quaternion implementation optimized for embedded systems,
///          with Vec<T,3> integration for efficient operations and CMSIS-DSP support.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "vector.hpp"
#include <type_traits>
#include "simd_helpers.hpp"

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
    // Memory layout: [x, y, z, w] where x,y,z are imaginary components (i,j,k) and w is real
    // This ordering matches Hamilton convention: q = w + xi + yj + zk
    // IMPORTANT: Some libraries use [w,x,y,z] ordering - verify compatibility when interfacing
    alignas(MICROLA_DATA_ALIGNMENT) T data[4] = {T(0), T(0), T(0), T(1)};  // x, y, z, w (default identity)

    /// \brief Default constructor producing identity quaternion.
    constexpr Quaternion() noexcept = default;

    /// \brief Constructor from components.
    /// \param w The real part.
    /// \param x The i component.
    /// \param y The j component.
    /// \param z The k component.
    constexpr Quaternion(T w, T x, T y, T z) : data{x, y, z, w} {}

    /// \brief Constructor from array.
    /// \param arr The array [x, y, z, w].
    constexpr explicit Quaternion(const T* arr) : data{arr[0], arr[1], arr[2], arr[3]}
    {
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            microla::simd::copy4_float(arr, data);
        }
    }

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
    constexpr Quaternion(const Quaternion& other) noexcept = default;

    /// \brief Move constructor (default for fixed-size arrays).
    constexpr Quaternion(Quaternion&& other) noexcept = default;

    /// \brief Copy assignment operator.
    constexpr auto operator=(const Quaternion& other) noexcept -> Quaternion& = default;

    /// \brief Move assignment operator (default for fixed-size arrays).
    constexpr auto operator=(Quaternion&& other) noexcept -> Quaternion& = default;

    /// \brief Destructor (defaulted to satisfy rule-of-five checkers).
    ~Quaternion() noexcept = default;

    /// \brief Subscript operator.
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Reference to the component.
    auto operator[](std::size_t index) noexcept -> T& { return data[index]; }

    /// \brief Subscript operator (const).
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Const reference to the component.
    auto operator[](std::size_t index) const noexcept -> const T& { return data[index]; }

    /// \brief Bounds-checked element access.
    /// \param index The index (0=x, 1=y, 2=z, 3=w).
    /// \return Reference to the component.
    /// \note In debug builds (MICROLA_DEBUG defined), triggers assertion if index >= 4.
    auto at(std::size_t index) -> T&
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
    [[nodiscard]] auto at(std::size_t index) const -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(index < 4 && "Quaternion::at: index out of range");
#endif
        return data[index];
    }

    /// \brief Addition operator.
    /// \param other The quaternion to add.
    /// \return The sum.
    [[nodiscard]] auto operator+(const Quaternion& other) const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            size_t vl = vsetvl_e32m1(4);
            /* cppcheck-suppress invalidPointerCast */
            vfloat32m1_t a = vle32_v_f32m1(data, vl);
            /* cppcheck-suppress invalidPointerCast */
            vfloat32m1_t b = vle32_v_f32m1(other.data, vl);
            vfloat32m1_t r = vfadd_vv_f32m1(a, b, vl);
            vse32_v_f32m1(result.data, r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            /* cppcheck-suppress invalidPointerCast */
            float32x4_t a = vld1q_f32(data);
            /* cppcheck-suppress invalidPointerCast */
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t r = vaddq_f32(a, b);
            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            /* cppcheck-suppress invalidPointerCast */
            arm_add_f32(data, other.data, /* cppcheck-suppress invalidPointerCast */ result.data, 4);
            return result;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS) && !defined(CONFIG_MICROLA_RISCV)
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            /* cppcheck-suppress invalidPointerCast */
            microla::simd::add4(data, other.data, /* cppcheck-suppress invalidPointerCast */ result.data);
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
    [[nodiscard]] auto operator-(const Quaternion& other) const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            size_t vl = vsetvl_e32m1(4);
            vfloat32m1_t a = vle32_v_f32m1(data, vl);
            vfloat32m1_t b = vle32_v_f32m1(other.data, vl);
            vfloat32m1_t r = vfsub_vv_f32m1(a, b, vl);
            vse32_v_f32m1(result.data, r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(data);
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t r = vsubq_f32(a, b);
            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            arm_sub_f32(data, other.data, result.data, 4);
            return result;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS) && !defined(CONFIG_MICROLA_RISCV)
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            microla::simd::sub4(data, other.data, result.data);
            return result;
        }
#endif
        // Use Vec operations for imaginary part
        const Vec<T, 3> imag_diff = vec() - other.vec();
        return Quaternion((*this)[W] - other[W], imag_diff[X], imag_diff[Y], imag_diff[Z]);
    }

    /// \brief Unary minus operator.
    /// \return The negated quaternion.
    [[nodiscard]] auto operator-() const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(data);
            float32x4_t r = vnegq_f32(a);
            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            arm_negate_f32(data, result.data, 4);
            return result;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS)
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            const float s4[4] = {-1.0F, -1.0F, -1.0F, -1.0F};
            microla::simd::mul4(data, s4, result.data);
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
    [[nodiscard]] auto operator*(T scalar) const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            float32x4_t a = vld1q_f32(data);
            float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t r = vmulq_f32(a, s);
            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            arm_scale_f32(data, static_cast<float>(scalar), result.data, 4);
            return result;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS)
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            float s4[4] = {static_cast<float>(scalar), static_cast<float>(scalar), static_cast<float>(scalar),
                           static_cast<float>(scalar)};
            microla::simd::mul4(data, s4, result.data);
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
    [[nodiscard]] constexpr auto operator/(T scalar) const noexcept -> Quaternion
    {
        // Check for zero to avoid undefined behavior (minimal overhead)
        if (scalar == T(0))
        {
            return Quaternion(T(0), T(0), T(0), T(0));
        }
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            const float inv_scalar = 1.0f / static_cast<float>(scalar);
            arm_scale_f32(data, inv_scalar, result.data, 4);
            return result;
        }
#endif
        const T inv = T(1) / scalar;
        return *this * inv;
    }

    /// \brief Quaternion multiplication.
    /// \param other The other quaternion.
    /// \return The product.
    [[nodiscard]] auto operator*(const Quaternion& other) const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            // Load quaternions: data layout is [x, y, z, w]
            float32x4_t q1 = vld1q_f32(data);        // [x1, y1, z1, w1]
            float32x4_t q2 = vld1q_f32(other.data);  // [x2, y2, z2, w2]

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

            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            arm_quaternion_product_f32(data, other.data, result.data);
            return result;
        }
#endif
        // Optimized scalar path using SIMD helpers for float
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            const float w1 = data[W];
            const float x1 = data[X];
            const float y1 = data[Y];
            const float z1 = data[Z];
            const float w2 = other.data[W];
            const float x2 = other.data[X];
            const float y2 = other.data[Y];
            const float z2 = other.data[Z];

            // Use simd helpers for 4-element operations
            float t0[4] = {w1 * x2, w1 * y2, w1 * z2, w1 * w2};
            float t1[4] = {x1 * w2, -x1 * z2, x1 * y2, -x1 * x2};
            float t2[4] = {y1 * z2, y1 * w2, -y1 * x2, -y1 * y2};
            const float t3[4] = {-z1 * y2, z1 * x2, z1 * w2, -z1 * z2};

            microla::simd::add4(t0, t1, t0);
            microla::simd::add4(t0, t2, t0);
            microla::simd::add4(t0, t3, result.data);

            return result;
        }

        return Quaternion((*this)[W] * other[W] - (*this)[X] * other[X] - (*this)[Y] * other[Y] - (*this)[Z] * other[Z],
                          (*this)[W] * other[X] + (*this)[X] * other[W] + (*this)[Y] * other[Z] - (*this)[Z] * other[Y],
                          (*this)[W] * other[Y] - (*this)[X] * other[Z] + (*this)[Y] * other[W] + (*this)[Z] * other[X],
                          (*this)[W] * other[Z] + (*this)[X] * other[Y] - (*this)[Y] * other[X] +
                              (*this)[Z] * other[W]);
    }

    /// \brief Rotate a 3D vector by this quaternion.
    /// \param v The vector to rotate.
    /// \return The rotated vector.
    [[nodiscard]] auto operator*(const Vec<T, 3>& v) const noexcept -> Vec<T, 3>
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
    /// \return True if equal (uses epsilon comparison for floating-point types).
    /// \note Uses `std::numeric_limits<T>::epsilon()` as an **absolute** tolerance.
    ///       For quaternions representing physical rotations whose components may
    ///       drift from unit scale, prefer `approx_equal(other, tol)` which uses
    ///       a caller-supplied tolerance.
    constexpr auto operator==(const Quaternion& other) const noexcept -> bool
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t diff = vabdq_f32(a, b);
            float32x4_t eps = vdupq_n_f32(std::numeric_limits<float>::epsilon());
            uint32x4_t cmp = vcleq_f32(diff, eps);
            uint64x2_t cmp64 = vreinterpretq_u64_u32(cmp);
            return vgetq_lane_u64(cmp64, 0) == ~0ULL && vgetq_lane_u64(cmp64, 1) == ~0ULL;
        }
#endif
        // For floating point types, use epsilon comparison
        if constexpr (std::is_floating_point<T>::value)
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
    auto operator!=(const Quaternion& other) const noexcept -> bool { return !(*this == other); }

    /// \brief Conjugate of the quaternion.
    /// \return The conjugate.
    [[nodiscard]] auto conjugate() const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            // Copy w component
            result.data[W] = data[W];
            // Negate imaginary components (x, y, z)
            arm_negate_f32(&data[X], &result.data[X], 3);
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            Quaternion result;
            // copy 4 floats then negate first three
            microla::simd::copy4_float(data, result.data);
            result.data[X] = -result.data[X];
            result.data[Y] = -result.data[Y];
            result.data[Z] = -result.data[Z];
            return result;
        }

        // Use Vec operations for negating imaginary part
        const Vec<T, 3> imag_neg = -vec();
        return Quaternion((*this)[W], imag_neg[X], imag_neg[Y], imag_neg[Z]);
    }

    /// \brief Inverse of a normalized quaternion.
    /// \return The conjugate, assuming the quaternion already has unit norm.
    [[nodiscard]] auto inverse_unit() const noexcept -> Quaternion { return conjugate(); }

    /// \brief Norm (magnitude) of the quaternion.
    /// \return The norm.
    /// \details In C++26+, this function is constexpr when the quaternion is known at compile-time.
    MICROLA_CONSTEXPR_TRIG [[nodiscard]] auto norm() const noexcept -> T
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return std::sqrt(static_cast<T>(vget_lane_f32(sum, 0)));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            float result;
            arm_dot_prod_f32(data, data, 4, &result);
            return std::sqrt(static_cast<T>(result));
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            const auto* src = data;
            float acc4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
            float tmp[4];
            microla::simd::load_padded4(src, 4, tmp, 0.0F);
            microla::simd::fma4_accumulate(acc4, tmp, tmp);
            const float sum = acc4[0] + acc4[1] + acc4[2] + acc4[3];
            return std::sqrt(static_cast<T>(sum));
        }

        // Use Vec<T,3> dot product for imaginary part + w^2
        const Vec<T, 3> imag = vec();
        return std::sqrt(imag.dot(imag) + (*this)[W] * (*this)[W]);
    }

    /// \brief Normalized quaternion.
    /// \return The unit quaternion.
    [[nodiscard]] auto normalized() const noexcept -> Quaternion
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            // Compute norm squared
            float32x4_t a = vld1q_f32(data);
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            float norm_sq = vget_lane_f32(sum, 0);

            if (norm_sq != 0.0f)
            {
                // Fast inverse square root with Newton-Raphson refinement
                float32x2_t norm_sq_v = vdup_n_f32(norm_sq);
                float32x2_t rsqrt = vrsqrte_f32(norm_sq_v);
                rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(norm_sq_v, rsqrt), rsqrt));
                rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(norm_sq_v, rsqrt), rsqrt));

                float inv_norm = vget_lane_f32(rsqrt, 0);
                return *this * static_cast<T>(inv_norm);
            }
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            float norm_sq;
            arm_dot_prod_f32(data, data, 4, &norm_sq);

            if (norm_sq != 0.0f)
            {
                float n;
                arm_sqrt_f32(norm_sq, &n);

                const float inv_norm = 1.0f / n;
                Quaternion result;
                arm_scale_f32(data, inv_norm, result.data, 4);
                return result;
            }
            return *this;
        }
#endif
        const T n = norm();
        if (n != T(0))
        {
            return *this / n;
        }
        return *this;
    }

    /// \brief Squared magnitude of the quaternion.
    /// \return The squared magnitude.
    [[nodiscard]] auto norm_squared() const noexcept -> T
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            float result;
            arm_dot_prod_f32(data, data, 4, &result);
            return static_cast<T>(result);
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            const auto* src = data;
            float acc4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
            float tmp[4];
            microla::simd::load_padded4(src, 4, tmp, 0.0F);
            microla::simd::fma4_accumulate(acc4, tmp, tmp);
            const float sum = acc4[0] + acc4[1] + acc4[2] + acc4[3];
            return static_cast<T>(sum);
        }

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
    [[nodiscard]] auto angle_to(const Quaternion& other) const noexcept -> T
    {
        const T d = dot(other);
        // Clamp to handle numerical errors
        T clamped = d;
        if (clamped < T(-1))
        {
            clamped = T(-1);
        }
        else if (clamped > T(1))
        {
            clamped = T(1);
        }
        return std::acos(clamped) * T(2);
    }

    /// \brief Create quaternion from two vectors.
    /// \param from The starting vector.
    /// \param to The ending vector.
    /// \return The rotation quaternion.
    [[nodiscard]] static auto from_two_vectors(const Vec<T, 3>& from, const Vec<T, 3>& to) noexcept -> Quaternion
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
    [[nodiscard]] auto inverse() const noexcept -> Quaternion
    {
        // Compute norm squared directly to avoid redundant sqrt
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t mul = vmulq_f32(a, a);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            const T n2 = static_cast<T>(vget_lane_f32(sum, 0));
            return conjugate() / n2;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            float result;
            arm_dot_prod_f32(data, data, 4, &result);
            return conjugate() / static_cast<T>(result);
        }
#endif
        const Vec<T, 3> imag = vec();
        const T n2 = imag.dot(imag) + (*this)[W] * (*this)[W];
        // Check for zero quaternion (minimal overhead)
#ifdef MICROLA_DEBUG
        assert(n2 != T(0) && "Inverse of zero quaternion is undefined");
#endif
        if (n2 == T(0))
        {
            // Return NaN quaternion to signal error in release builds
            return Quaternion(std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::quiet_NaN(),
                              std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::quiet_NaN());
        }
        return conjugate() / n2;
    }

    /// \brief Convert to 3x3 rotation matrix.
    /// \return The rotation matrix.
    [[nodiscard]] auto to_matrix() const noexcept -> Mat<T, 3, 3>
    {
        const T w = (*this)[W];
        const T x = (*this)[X];
        const T y = (*this)[Y];
        const T z = (*this)[Z];
        const T xx = x * x;
        const T yy = y * y;
        const T zz = z * z;
        const T xy = x * y;
        const T xz = x * z;
        const T yz = y * z;
        const T wx = w * x;
        const T wy = w * y;
        const T wz = w * z;
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
    [[nodiscard]] auto dot(const Quaternion& other) const noexcept -> T
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t mul = vmulq_f32(a, b);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            float result;
            arm_dot_prod_f32(data, other.data, 4, &result);
            return static_cast<T>(result);
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            const auto* a = data;
            const auto* b = other.data;
            float acc4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
            float ta[4];
            float tb[4];
            microla::simd::load_padded4(a, 4, ta, 0.0F);
            microla::simd::load_padded4(b, 4, tb, 0.0F);
            microla::simd::fma4_accumulate(acc4, ta, tb);
            return static_cast<T>(acc4[0] + acc4[1] + acc4[2] + acc4[3]);
        }

        // Use Vec<T,3> dot product for imaginary part + w*w
        const Vec<T, 3> imag1 = vec();
        const Vec<T, 3> imag2 = other.vec();
        return imag1.dot(imag2) + (*this)[W] * other[W];
    }

    /// \brief Rotate a vector by this quaternion.
    /// \param v The vector to rotate.
    /// \return The rotated vector.
    [[nodiscard]] auto rotate(const Vec<T, 3>& v) const noexcept -> Vec<T, 3>
    {
        // Optimized rotation: v' = v + 2*w*(xyz x v) + 2*(xyz x (xyz x v))
        const Vec<T, 3> imag = vec();
        const Vec<T, 3> cross1 = imag.cross(v);
        const Vec<T, 3> cross2 = imag.cross(cross1);
        return v + cross1 * (T(2) * (*this)[W]) + cross2 * T(2);
    }

    /// \brief Rotate a 3D vector by the inverse of a normalized quaternion.
    /// \param v The vector to rotate.
    /// \return The vector rotated by q^-1, assuming q is already normalized.
    [[nodiscard]] auto rotate_inverse(const Vec<T, 3>& v) const noexcept -> Vec<T, 3>
    {
        const Vec<T, 3> imag = vec();
        const Vec<T, 3> cross1 = imag.cross(v);
        const Vec<T, 3> cross2 = imag.cross(cross1);
        return v - cross1 * (T(2) * (*this)[W]) + cross2 * T(2);
    }

    /// \brief Component accessors.
    auto w() noexcept -> T& { return data[W]; }
    [[nodiscard]] auto w() const noexcept -> const T& { return data[W]; }
    auto x() noexcept -> T& { return data[X]; }
    [[nodiscard]] auto x() const noexcept -> const T& { return data[X]; }
    auto y() noexcept -> T& { return data[Y]; }
    [[nodiscard]] auto y() const noexcept -> const T& { return data[Y]; }
    auto z() noexcept -> T& { return data[Z]; }
    [[nodiscard]] auto z() const noexcept -> const T& { return data[Z]; }

    /// \brief Get copy of imaginary part as Vec<T,3>.
    /// \return Copy of the imaginary components (x,y,z).
    /// \note Returns by value to avoid type-punning UB. Compilers optimize this to zero-cost.
    [[nodiscard]] auto vec() const noexcept -> Vec<T, 3> { return Vec<T, 3>(data[X], data[Y], data[Z]); }

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
    auto operator+=(const Quaternion& other) noexcept -> Quaternion&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t r = vaddq_f32(a, b);
            vst1q_f32(data, r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            arm_add_f32(data, other.data, data, 4);
            return *this;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS)
        if constexpr (std::is_same_v<T, float>)
        {
            microla::simd::add4(data, other.data, data);
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
    auto operator-=(const Quaternion& other) noexcept -> Quaternion&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t b = vld1q_f32(other.data);
            float32x4_t r = vsubq_f32(a, b);
            vst1q_f32(data, r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            arm_sub_f32(data, other.data, data, 4);
            return *this;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS)
        if constexpr (std::is_same_v<T, float>)
        {
            microla::simd::sub4(data, other.data, data);
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
    auto operator*=(T scalar) noexcept -> Quaternion&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            float32x4_t a = vld1q_f32(data);
            float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t r = vmulq_f32(a, s);
            vst1q_f32(data, r);
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            arm_scale_f32(data, static_cast<float>(scalar), data, 4);
            return *this;
        }
#endif
#if !defined(CONFIG_MICROLA_NEON) && !defined(CONFIG_MICROLA_CMSIS)
        if constexpr (std::is_same_v<T, float>)
        {
            float s4[4] = {static_cast<float>(scalar), static_cast<float>(scalar), static_cast<float>(scalar),
                           static_cast<float>(scalar)};
            float tmp[4];
            microla::simd::mul4(data, s4, tmp);
            microla::simd::store_extract4(tmp, data, 4);
            return *this;
        }
#endif
        // Scale all components directly
        data[X] *= scalar;
        data[Y] *= scalar;
        data[Z] *= scalar;
        data[W] *= scalar;
        return *this;
    }

    /// \brief Scalar division assignment operator.
    /// \param scalar The scalar to divide by.
    /// \return Reference to this.
    constexpr auto operator/=(T scalar) noexcept -> Quaternion&
    {
        const T inv = T(1) / scalar;
        return *this *= inv;
    }

    /// \brief Quaternion multiplication assignment operator.
    /// \param other The quaternion to multiply by.
    /// \return Reference to this.
    auto operator*=(const Quaternion& other) noexcept -> Quaternion&
    {
        *this = *this * other;
        return *this;
    }

    /// \brief Approximate equality comparison.
    /// \param other The quaternion to compare.
    /// \param epsilon The tolerance (defaults to machine epsilon for type T).
    /// \return True if approximately equal.
    [[nodiscard]] auto approx_equal(const Quaternion& other,
                                    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept -> bool
    {
        // Use Vec operations for imaginary part comparison
        return vec().approx_equal(other.vec(), epsilon) && std::abs((*this)[W] - other[W]) <= epsilon;
    }

    /// \brief Check whether the quaternion is approximately the identity rotation.
    /// \param epsilon Tolerance for floating-point comparisons.
    /// \return True if the quaternion is within epsilon of (0, 0, 0, 1).
    [[nodiscard]] auto is_identity(T epsilon = std::numeric_limits<T>::epsilon() *
                                               static_cast<T>(8)) const noexcept -> bool
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            return vec().is_zero(epsilon) && std::abs((*this)[W] - T(1)) <= epsilon;
        }

        return (*this)[X] == T(0) && (*this)[Y] == T(0) && (*this)[Z] == T(0) && (*this)[W] == T(1);
    }

    /// \brief Linear interpolation (lerp) between two quaternions.
    /// \param other The target quaternion.
    /// \param t The interpolation parameter [0, 1].
    /// \return The interpolated quaternion (not normalized).
    [[nodiscard]] auto lerp(const Quaternion& other, T t) const noexcept -> Quaternion
    {
        // Use Vec operations for imaginary part interpolation
        const Vec<T, 3> imag_lerp = vec() + (other.vec() - vec()) * t;
        return Quaternion((*this)[W] + t * (other[W] - (*this)[W]), imag_lerp[X], imag_lerp[Y], imag_lerp[Z]);
    }

    /// \brief Spherical linear interpolation (slerp) between two quaternions.
    /// \param other The target quaternion.
    /// \param t The interpolation parameter [0, 1].
    /// \return The interpolated quaternion.
    [[nodiscard]] auto slerp(const Quaternion& other, T t) const noexcept -> Quaternion
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
    auto ptr() noexcept -> T* { return data; }

    /// \brief Get raw pointer to data (const).
    /// \return Const pointer to the underlying data array.
    [[nodiscard]] auto ptr() const noexcept -> const T* { return data; }

    /// \brief Get size of the quaternion.
    /// \return Always returns 4.
    [[nodiscard]] constexpr auto size() const noexcept -> std::size_t { return 4; }

    /// \brief Create identity quaternion.
    /// \return The identity quaternion (0, 0, 0, 1).
    static constexpr auto identity() noexcept -> Quaternion { return Quaternion(T(1), T(0), T(0), T(0)); }

    /// \brief Create quaternion from axis-angle representation.
    /// \param axis The rotation axis (should be normalized).
    /// \param angle The rotation angle in radians.
    /// \return Quaternion representing the rotation.
    static auto from_axis_angle(const Vec<T, 3>& axis, T angle) noexcept -> Quaternion
    {
        return Quaternion(axis, angle);
    }

    /// \brief Convert quaternion to Euler angles (roll, pitch, yaw)
    /// \return Vec<T,3> containing (roll, pitch, yaw) in radians
    /// Roll (rotation about X), Pitch (rotation about Y), Yaw (rotation about Z)
    [[nodiscard]] auto to_euler() const noexcept -> Vec<T, 3>
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
    static auto from_euler(const Vec<T, 3>& euler) noexcept -> Quaternion
    {
        return from_euler(euler[ROLL], euler[PITCH], euler[YAW]);
    }

    /// \brief Create quaternion from Euler angles (roll, pitch, yaw)
    /// \param roll Rotation about X axis in radians
    /// \param pitch Rotation about Y axis in radians
    /// \param yaw Rotation about Z axis in radians
    /// \return Quaternion representing the rotation
    static auto from_euler(T roll, T pitch, T yaw) noexcept -> Quaternion
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
    [[nodiscard]] auto roll() const noexcept -> T
    {
        const T sinr_cosp = T(2) * ((*this)[W] * (*this)[X] + (*this)[Y] * (*this)[Z]);
        const T cosr_cosp = T(1) - T(2) * ((*this)[X] * (*this)[X] + (*this)[Y] * (*this)[Y]);
        return std::atan2(sinr_cosp, cosr_cosp);
    }

    /// \brief Extract pitch angle (rotation about Y axis)
    /// \return Pitch angle in radians
    [[nodiscard]] auto pitch() const noexcept -> T
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
    [[nodiscard]] auto yaw() const noexcept -> T
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
[[nodiscard]] constexpr auto operator*(T scalar, const Quaternion<T>& q) noexcept -> Quaternion<T>
{
    return q * scalar;
}

/// @brief Alias for Quaternion<float>
using Quatf = Quaternion<float>;

}  // namespace microla
