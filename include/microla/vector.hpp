// SPDX-License-Identifier: Apache-2.0
/// @file vector.hpp
/// @brief Generic vector class Vec<T,N> with full operator support
/// @details This header provides the template-based vector class with comprehensive
///          operator support, SIMD optimizations (CMSIS-DSP, NEON, MVE), and specialized
///          functionality for embedded systems and real-time applications.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include <cstdint>
#include <type_traits>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cassert>
#include <initializer_list>

#include "compiler_features.hpp"
#include "constants.hpp"
#include "simd_helpers.hpp"

// Optional NEON/MVE/CMSIS/RISC-V blocks (disabled by default)
#ifdef CONFIG_MICROLA_NEON
#include <arm_neon.h>
#endif

#ifdef CONFIG_MICROLA_CMSIS
#include <arm_math.h>
#endif

#ifdef CONFIG_MICROLA_RISCV
#include <riscv_vector.h>
#endif

namespace microla
{

// Minimal index_sequence implementation to allow constexpr factories
namespace detail
{
template<std::size_t... Is>
struct index_sequence
{
    // NOLINT(readability-identifier-naming)
};

template<std::size_t N, std::size_t... Is>
struct make_index_sequence_impl : make_index_sequence_impl<N - 1, N - 1, Is...>
{
    // NOLINT(readability-identifier-naming)
};

template<std::size_t... Is>
struct make_index_sequence_impl<0, Is...>
{
    // NOLINT(readability-identifier-naming)
    using type = index_sequence<Is...>;
};

template<std::size_t N>
using make_index_sequence = typename make_index_sequence_impl<N>::type;

}  // namespace detail

/// \class Vec<T, N>
/// \brief A vector class templated on type T and dimension N.
/// \tparam T The storage type.
/// \tparam N The dimension.
template<typename T = float, std::size_t N = 3>
class Vec
{
public:
    static_assert(N > 0, "Vec dimension N must be greater than 0");
    static_assert(std::is_arithmetic_v<T>, "Vec requires an arithmetic scalar type");

    // Common component indices (shared with Quaternion)
    enum
    {
        X = 0,
        Y = 1,
        Z = 2,
        W = 3
    };

    // Single-dimensional array ensures contiguous memory layout, POD compatibility,
    // and efficient SIMD operations. Row-major storage: data[i] = element i
    // Default-initialize to zero to preserve legacy semantics for `Vec()`.
    alignas(16) T data[N] = {};  // Aligned for SIMD performance

    /// \brief Copy constructor.
    /// \param other The vector to copy.
    constexpr Vec(const Vec& other) noexcept = default;

    /// \brief Move constructor (default for fixed-size arrays).
    constexpr Vec(Vec&& other) noexcept = default;

    /// \brief Copy assignment operator.
    constexpr auto operator=(const Vec& other) noexcept -> Vec& = default;

    /// \brief Move assignment operator (default for fixed-size arrays).
    constexpr auto operator=(Vec&& other) noexcept -> Vec& = default;

    /// \brief Destructor (defaulted to satisfy rule-of-five checkers).
    ~Vec() noexcept = default;

    /// \brief Default constructor.
    constexpr Vec() noexcept = default;

    /// \brief Constructor from array.
    /// \param arr The array to copy from.
    constexpr explicit Vec(const T* arr)
    {
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            simd::copy_n_float(arr, data, N);
        }
        else
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                data[i] = arr[i];
            }
        }
    }

private:
    // Tag constructor used only by constexpr factory helpers to perform
    // pack-based initialization without conflicting with public overloads.
    struct _pack_tag
    {
        // NOLINT(readability-identifier-naming)
    };

    template<typename... U>
    constexpr Vec(_pack_tag, U... u) noexcept : data{static_cast<T>(u)...}
    {
        static_assert(sizeof...(U) == N, "Pack constructor requires exactly N arguments");
    }

#ifdef CONFIG_MICROLA_NEON
    static auto load3_neon(const float* src) noexcept -> float32x4_t
    {
        float32x4_t value = vdupq_n_f32(0.0F);
        value = vld1q_lane_f32(src, value, 0);
        value = vld1q_lane_f32(src + 1, value, 1);
        value = vld1q_lane_f32(src + 2, value, 2);
        return value;
    }

    static void store3_neon(float* dst, float32x4_t value) noexcept
    {
        vst1q_lane_f32(dst, value, 0);
        vst1q_lane_f32(dst + 1, value, 1);
        vst1q_lane_f32(dst + 2, value, 2);
    }
#endif

    template<std::size_t... Is>
    constexpr static Vec zero_impl(detail::index_sequence<Is...>) noexcept
    {
        return Vec(_pack_tag{}, (static_cast<T>(0) + static_cast<T>(Is * 0))...);
    }

    template<std::size_t... Is>
    constexpr static Vec filled_impl(detail::index_sequence<Is...>, T value) noexcept
    {
        return Vec(_pack_tag{}, (value + static_cast<T>(Is * 0))...);
    }

public:
    /// \brief Constructor from fixed-size array with bounds checking.
    /// \tparam M Size of the input array (must be >= N).
    /// \param arr Fixed-size array reference to copy from.
    template<std::size_t M>
    constexpr explicit Vec(const T (&arr)[M])
    {
        static_assert(M >= N, "Array size must be at least N elements");
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            simd::copy_n_float(arr, data, N);
        }
        else
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                data[i] = arr[i];
            }
        }
    }

    /// \brief Constructor from initializer_list to handle brace-init syntax.
    /// \param il Initializer list containing values to initialize the vector.
    constexpr Vec(std::initializer_list<T> il)
    {
        auto it = il.begin();
        std::size_t i = 0;
        for (; i < N && it != il.end(); ++i, ++it)
        {
            data[i] = *it;
        }
        for (; i < N; ++i)
        {
            data[i] = static_cast<T>(0);
        }
    }

    /// \brief Variadic constructor for direct initialization.
    /// \tparam Args The types of the arguments.
    /// \param first The first value to initialize with.
    /// \param args The remaining values to initialize with.
    template<typename... Args>
    constexpr Vec(std::enable_if_t<(sizeof...(Args) + 1 == N) && (N >= 2), T> first, Args... args)
    {
        /* cppcheck-suppress arrayIndexOutOfBounds */
        T temp[N] = {first, static_cast<T>(args)...};
        for (std::size_t i = 0; i < N; ++i)
        {
            data[i] = temp[i];
        }
    }

    /// \brief Single value constructor for N=1 vectors.
    /// \param value The value to initialize with.
    template<typename U = T>
    constexpr Vec(std::enable_if_t<N == 1, U> value)
    {
        data[0] = static_cast<T>(value);
    }

    /// \brief Subscript operator.
    /// \param index The index.
    /// \return Reference to the element.
    constexpr auto operator[](std::size_t index) noexcept -> T& { return data[index]; }

    /// \brief Subscript operator (const).
    /// \param index The index.
    /// \return Const reference to the element.
    constexpr auto operator[](std::size_t index) const noexcept -> const T& { return data[index]; }

    /// \brief Bounds-checked element access.
    /// \param index The index.
    /// \return Reference to the element.
    /// \note In debug builds (MICROLA_DEBUG defined), triggers assertion if index >= N.
    auto at(std::size_t index) noexcept -> T&
    {
#ifdef MICROLA_DEBUG
        assert(index < N && "Vec::at: index out of range");
#endif
        return data[index];
    }

    /// \brief Bounds-checked element access (const).
    /// \param index The index.
    /// \return Const reference to the element.
    /// \note In debug builds (MICROLA_DEBUG defined), triggers assertion if index >= N.
    [[nodiscard]] auto at(std::size_t index) const -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(index < N && "Vec::at: index out of range");
#endif
        return data[index];
    }

    /// \brief Get the number of elements in the vector.
    /// \return The size of the vector (N).
    static constexpr auto size() noexcept -> std::size_t { return N; }

    /// \brief Pointer to underlying storage.
    /// \return Pointer to the first element.
    [[nodiscard]] auto data_ptr() noexcept -> T* { return data; }

    /// \brief Pointer to underlying storage (const).
    /// \return Const pointer to the first element.
    [[nodiscard]] auto data_ptr() const noexcept -> const T* { return data; }

    /// \brief Iterators for ranged-for compatibility.
    [[nodiscard]] auto begin() noexcept -> T* { return data; }
    [[nodiscard]] auto end() noexcept -> T* { return data + N; }
    [[nodiscard]] auto begin() const noexcept -> const T* { return data; }
    [[nodiscard]] auto end() const noexcept -> const T* { return data + N; }
    [[nodiscard]] auto cbegin() const noexcept -> const T* { return data; }
    [[nodiscard]] auto cend() const noexcept -> const T* { return data + N; }

    /// \brief Addition operator.
    /// \param other The vector to add.
    /// \return The result vector.
    [[nodiscard]] constexpr auto operator+(const Vec& other) const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float> && (N == 2 || N == 3 || N == 4))
        {
            Vec result;
            size_t vl = vsetvl_e32m1(N);
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
        if constexpr (std::is_same_v<T, float> && (N == 2))
        {
            Vec result;
            /* cppcheck-suppress invalidPointerCast */
            float32x2_t a = vld1_f32(data);
            /* cppcheck-suppress invalidPointerCast */
            float32x2_t b = vld1_f32(other.data);
            float32x2_t r = vadd_f32(a, b);
            vst1_f32(result.data, r);
            return result;
        }
        if constexpr (std::is_same_v<T, float> && (N == 3))
        {
            Vec result;
            const float32x4_t a = load3_neon(data);
            const float32x4_t b = load3_neon(other.data);
            store3_neon(result.data, vaddq_f32(a, b));
            return result;
        }
        if constexpr (std::is_same_v<T, float> && (N == 4))
        {
            Vec result;
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
            Vec result;
            /* cppcheck-suppress invalidPointerCast */
            arm_add_f32(data, other.data, /* cppcheck-suppress invalidPointerCast */ result.data, N);
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                Vec result;
                /* cppcheck-suppress arrayIndexOutOfBounds */
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                /* cppcheck-suppress arrayIndexOutOfBounds */
                float b4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                /* cppcheck-suppress arrayIndexOutOfBounds */
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::load_padded4(other.data, N, b4);
                simd::add4(a4, b4, r4);
                simd::store_extract4(r4, result.data, N);
                return result;
            }
        }
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    /// \brief Subtraction operator.
    /// \param other The vector to subtract.
    /// \return The result vector.
    [[nodiscard]] constexpr auto operator-(const Vec& other) const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float> && (N == 2 || N == 3 || N == 4))
        {
            Vec result;
            size_t vl = vsetvl_e32m1(N);
            vfloat32m1_t a = vle32_v_f32m1(data, vl);
            vfloat32m1_t b = vle32_v_f32m1(other.data, vl);
            vfloat32m1_t r = vfsub_vv_f32m1(a, b, vl);
            vse32_v_f32m1(result.data, r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && (N == 2))
        {
            Vec result;
            float32x2_t a = vld1_f32(data);
            float32x2_t b = vld1_f32(other.data);
            float32x2_t r = vsub_f32(a, b);
            vst1_f32(result.data, r);
            return result;
        }
        if constexpr (std::is_same_v<T, float> && (N == 3))
        {
            Vec result;
            const float32x4_t a = load3_neon(data);
            const float32x4_t b = load3_neon(other.data);
            store3_neon(result.data, vsubq_f32(a, b));
            return result;
        }
        if constexpr (std::is_same_v<T, float> && (N == 4))
        {
            Vec result;
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
            Vec result;
            arm_sub_f32(data, other.data, result.data, N);
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                Vec result;
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float b4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::load_padded4(other.data, N, b4);
                simd::sub4(a4, b4, r4);
                simd::store_extract4(r4, result.data, N);
                return result;
            }
        }
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    /// \brief Unary minus operator.
    /// \return The negated vector.
    [[nodiscard]] constexpr auto operator-() const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            Vec result;
            store3_neon(result.data, vnegq_f32(load3_neon(data)));
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Vec result;
            arm_negate_f32(data, result.data, N);
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                Vec result;
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float s4[4] = {-1.0F, -1.0F, -1.0F, -1.0F};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::mul4(a4, s4, r4);
                simd::store_extract4(r4, result.data, N);
                return result;
            }
        }
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = -data[i];
        }
        return result;
    }

    /// \brief Scalar multiplication operator.
    /// \param scalar The scalar to multiply by.
    /// \return The result vector.
    [[nodiscard]] constexpr auto operator*(T scalar) const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float> && (N == 2 || N == 3 || N == 4))
        {
            Vec result;
            size_t vl = vsetvl_e32m1(N);
            vfloat32m1_t a = vle32_v_f32m1(data, vl);
            vfloat32m1_t r = vfmul_vf_f32m1(a, static_cast<float>(scalar), vl);
            vse32_v_f32m1(result.data, r, vl);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 2)
        {
            Vec result;
            float32x2_t a = vld1_f32(data);
            float32x2_t s = vdup_n_f32(static_cast<float>(scalar));
            float32x2_t r = vmul_f32(a, s);
            vst1_f32(result.data, r);
            return result;
        }
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            Vec result;
            const float32x4_t a = load3_neon(data);
            const float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            store3_neon(result.data, vmulq_f32(a, s));
            return result;
        }
        if constexpr (std::is_same_v<T, float> && N == 4)
        {
            Vec result;
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
            Vec result;
            arm_scale_f32(data, static_cast<float>(scalar), result.data, N);
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                Vec result;
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float s4[4] = {static_cast<float>(scalar), static_cast<float>(scalar), static_cast<float>(scalar),
                               static_cast<float>(scalar)};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::mul4(a4, s4, r4);
                simd::store_extract4(r4, result.data, N);
                return result;
            }
        }
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    /// \brief Scalar division operator.
    /// \param scalar The scalar to divide by.
    /// \return The result vector.
    /// \note In debug builds, asserts if scalar is zero. In release builds, returns NaN-filled vector.
    [[nodiscard]] constexpr auto operator/(T scalar) const noexcept -> Vec
    {
#ifdef MICROLA_DEBUG
        assert(scalar != T(0) && "Division by zero in Vec::operator/");
#endif
        // Return NaN-filled vector for division by zero in release builds
        if (scalar == T(0))
        {
            Vec result;
            for (std::size_t i = 0; i < N; ++i)
            {
                result.data[i] = std::numeric_limits<T>::quiet_NaN();
            }
            return result;
        }
#ifdef CONFIG_MICROLA_NEON
        // Fast reciprocal for NEON float vectors
        if constexpr (std::is_same_v<T, float> && N == 2)
        {
            Vec result;
            float32x2_t a = vld1_f32(data);
            float32x2_t s = vdup_n_f32(static_cast<float>(scalar));
            // Fast reciprocal with Newton-Raphson refinement
            float32x2_t recip = vrecpe_f32(s);
            recip = vmul_f32(recip, vrecps_f32(s, recip));
            recip = vmul_f32(recip, vrecps_f32(s, recip));
            float32x2_t r = vmul_f32(a, recip);
            vst1_f32(result.data, r);
            return result;
        }
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            Vec result;
            const float32x4_t a = load3_neon(data);
            const float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t recip = vrecpeq_f32(s);
            recip = vmulq_f32(recip, vrecpsq_f32(s, recip));
            recip = vmulq_f32(recip, vrecpsq_f32(s, recip));
            store3_neon(result.data, vmulq_f32(a, recip));
            return result;
        }
        if constexpr (std::is_same_v<T, float> && N == 4)
        {
            Vec result;
            float32x4_t a = vld1q_f32(data);
            float32x4_t s = vdupq_n_f32(static_cast<float>(scalar));
            float32x4_t recip = vrecpeq_f32(s);
            recip = vmulq_f32(recip, vrecpsq_f32(s, recip));
            recip = vmulq_f32(recip, vrecpsq_f32(s, recip));
            float32x4_t r = vmulq_f32(a, recip);
            vst1q_f32(result.data, r);
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Vec result;
            const float inv_scalar = 1.0f / static_cast<float>(scalar);
            arm_scale_f32(data, inv_scalar, result.data, N);
            return result;
        }
#endif
        Vec result;
        const T inv_scalar = T(1) / scalar;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] * inv_scalar;
        }
        return result;
    }

    /// \brief Equality operator.
    /// \param other The vector to compare.
    /// \return True if equal (uses epsilon comparison for floating point types).
    constexpr auto operator==(const Vec& other) const noexcept -> bool
    {
        // Optimized path for integral types (exact comparison)
        if constexpr (std::is_integral_v<T>)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (data[i] != other.data[i])
                {
                    return false;
                }
            }
            return true;
        }
        // Floating-point types use epsilon comparison
        else if constexpr (std::is_floating_point_v<T>)
        {
#ifdef CONFIG_MICROLA_NEON
            // SIMD path for float equality comparison with epsilon
            if constexpr (std::is_same_v<T, float> && N == 4)
            {
                float32x4_t a = vld1q_f32(data);
                float32x4_t b = vld1q_f32(other.data);
                float32x4_t diff = vabdq_f32(a, b);
                float32x4_t eps = vdupq_n_f32(std::numeric_limits<float>::epsilon());
                uint32x4_t cmp = vcleq_f32(diff, eps);
                // All lanes must be true
                uint64x2_t cmp64 = vreinterpretq_u64_u32(cmp);
                return vgetq_lane_u64(cmp64, 0) == ~0ULL && vgetq_lane_u64(cmp64, 1) == ~0ULL;
            }
            if constexpr (std::is_same_v<T, float> && N == 3)
            {
                // Use approximate comparison for floats
                for (std::size_t i = 0; i < 3; ++i)
                {
                    if (std::abs(data[i] - other.data[i]) > std::numeric_limits<float>::epsilon())
                        return false;
                }
                return true;
            }
            if constexpr (std::is_same_v<T, float> && N == 2)
            {
                float32x2_t a = vld1_f32(data);
                float32x2_t b = vld1_f32(other.data);
                float32x2_t diff = vabd_f32(a, b);
                float32x2_t eps = vdup_n_f32(std::numeric_limits<float>::epsilon());
                uint32x2_t cmp = vcle_f32(diff, eps);
                // Both lanes must be true
                return vget_lane_u32(cmp, 0) == ~0U && vget_lane_u32(cmp, 1) == ~0U;
            }
#endif
            // Fallback: epsilon comparison for all floating-point types
            for (std::size_t i = 0; i < N; ++i)
            {
                if (std::abs(data[i] - other.data[i]) > std::numeric_limits<T>::epsilon())
                {
                    return false;
                }
            }
            return true;
        }
        // Fallback for other types: exact comparison
        for (std::size_t i = 0; i < N; ++i)
        {
            if (data[i] != other.data[i])
            {
                return false;
            }
        }
        return true;
    }

    /// \brief Inequality operator.
    /// \param other The vector to compare.
    /// \return True if not equal.
    auto operator!=(const Vec& other) const noexcept -> bool { return !(*this == other); }

    /// \brief Addition assignment operator.
    /// \param other The vector to add.
    /// \return Reference to this.
    constexpr auto operator+=(const Vec& other) noexcept -> Vec&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            store3_neon(data, vaddq_f32(load3_neon(data), load3_neon(other.data)));
            return *this;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float b4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::load_padded4(other.data, N, b4);
                simd::add4(a4, b4, r4);
                simd::store_extract4(r4, data, N);
                return *this;
            }
        }
        for (std::size_t i = 0; i < N; ++i)
        {
            data[i] = data[i] + other.data[i];
        }
        return *this;
    }

    /// \brief Subtraction assignment operator.
    /// \param other The vector to subtract.
    /// \return Reference to this.
    constexpr auto operator-=(const Vec& other) noexcept -> Vec&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            store3_neon(data, vsubq_f32(load3_neon(data), load3_neon(other.data)));
            return *this;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float b4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::load_padded4(other.data, N, b4);
                simd::sub4(a4, b4, r4);
                simd::store_extract4(r4, data, N);
                return *this;
            }
        }
        for (std::size_t i = 0; i < N; ++i)
        {
            data[i] = data[i] - other.data[i];
        }
        return *this;
    }

    /// \brief Scalar multiplication assignment operator.
    /// \param scalar The scalar to multiply by.
    /// \return Reference to this.
    constexpr auto operator*=(T scalar) noexcept -> Vec&
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            store3_neon(data, vmulq_f32(load3_neon(data), vdupq_n_f32(static_cast<float>(scalar))));
            return *this;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float s4[4] = {static_cast<float>(scalar), static_cast<float>(scalar), static_cast<float>(scalar),
                               static_cast<float>(scalar)};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::mul4(a4, s4, r4);
                simd::store_extract4(r4, data, N);
                return *this;
            }
        }
        for (std::size_t i = 0; i < N; ++i)
        {
            data[i] = data[i] * scalar;
        }
        return *this;
    }

    /// \brief Scalar division assignment operator.
    /// \param scalar The scalar to divide by.
    /// \return Reference to this.
    constexpr auto operator/=(T scalar) noexcept -> Vec&
    {
        // Check for zero to avoid undefined behavior (minimal overhead)
        if (scalar == T(0))
        {
            // Set to zero for safety
            for (std::size_t i = 0; i < N; ++i)
            {
                data[i] = T(0);
            }
            return *this;
        }
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                const float inv = 1.0F / static_cast<float>(scalar);
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float s4[4] = {inv, inv, inv, inv};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::mul4(a4, s4, r4);
                simd::store_extract4(r4, data, N);
                return *this;
            }
        }
        const T inv_scalar = T(1) / scalar;
        for (std::size_t i = 0; i < N; ++i)
        {
            data[i] = data[i] * inv_scalar;
        }
        return *this;
    }

    /// \brief Dot product.
    /// \param other The other vector.
    /// \return The dot product.
    [[nodiscard]] constexpr auto dot(const Vec& other) const noexcept -> T
    {
        // RISCV vector extension path
#ifdef CONFIG_MICROLA_RISCV
        if constexpr (std::is_same_v<T, float> && (N == 2 || N == 3 || N == 4))
        {
            size_t vl = vsetvl_e32m1(N);
            vfloat32m1_t a = vle32_v_f32m1(reinterpret_cast<const float*>(data), vl);
            vfloat32m1_t b = vle32_v_f32m1(reinterpret_cast<const float*>(other.data), vl);
            vfloat32m1_t mul = vfmul_vv_f32m1(a, b, vl);
            vfloat32m1_t sumv =
                vfredsum_vs_f32m1_f32m1(vundefined_f32m1(), mul, vfmv_s_f_f32m1(vundefined_f32m1(), 0.0f, vl), vl);
            return static_cast<T>(vfmv_f_s_f32m1_f32(sumv));
        }
#endif

        // NEON paths
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 2)
        {
            float32x2_t a = vld1_f32(reinterpret_cast<const float*>(data));
            float32x2_t b = vld1_f32(reinterpret_cast<const float*>(other.data));
            float32x2_t mul = vmul_f32(a, b);
            float32x2_t sum = vpadd_f32(mul, mul);  // Horizontal add
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            const float32x4_t a = load3_neon(data);
            const float32x4_t b = load3_neon(other.data);
            const float32x4_t mul = vmulq_f32(a, b);
            float32x2_t sum = vadd_f32(vget_low_f32(mul), vget_high_f32(mul));
            sum = vpadd_f32(sum, sum);
            return static_cast<T>(vget_lane_f32(sum, 0));
        }
        if constexpr (std::is_same_v<T, float> && N == 4)
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
        if constexpr (std::is_same_v<T, float>)
        {
            float result_f = 0.0F;
            arm_dot_prod_f32(reinterpret_cast<const float*>(data), reinterpret_cast<const float*>(other.data),
                             static_cast<uint16_t>(N), &result_f);
            return static_cast<T>(result_f);
        }
#endif

        // Scalar fallback
        T sum = T(0);
        for (std::size_t i = 0; i < N; ++i)
        {
            sum += data[i] * other.data[i];
        }
        return sum;
    }

    /// \brief Cross product (only for 3D vectors).
    /// \param other The other vector.
    /// \return The cross product.
    template<std::size_t NN = N>
        requires(NN == 3)
    [[nodiscard]] constexpr auto cross(const Vec& other) const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float>)
        {
            Vec result;
            const float32x4_t a = load3_neon(data);
            const float32x4_t b = load3_neon(other.data);

            // Manually construct shuffles to avoid rotation issues with vext
            // a_yzx = [y, z, x, 0]
            float32x4_t a_yzx = vsetq_lane_f32(vgetq_lane_f32(a, 1), a, 0);  // Set lane 0 to y
            a_yzx = vsetq_lane_f32(vgetq_lane_f32(a, 2), a_yzx, 1);          // Set lane 1 to z
            a_yzx = vsetq_lane_f32(vgetq_lane_f32(a, 0), a_yzx, 2);          // Set lane 2 to x
            a_yzx = vsetq_lane_f32(0.0f, a_yzx, 3);                          // Set lane 3 to 0

            // b_yzx = [y, z, x, 0]
            float32x4_t b_yzx = vsetq_lane_f32(vgetq_lane_f32(b, 1), b, 0);
            b_yzx = vsetq_lane_f32(vgetq_lane_f32(b, 2), b_yzx, 1);
            b_yzx = vsetq_lane_f32(vgetq_lane_f32(b, 0), b_yzx, 2);
            b_yzx = vsetq_lane_f32(0.0f, b_yzx, 3);

            // a_zxy = [z, x, y, 0]
            float32x4_t a_zxy = vsetq_lane_f32(vgetq_lane_f32(a, 2), a, 0);
            a_zxy = vsetq_lane_f32(vgetq_lane_f32(a, 0), a_zxy, 1);
            a_zxy = vsetq_lane_f32(vgetq_lane_f32(a, 1), a_zxy, 2);
            a_zxy = vsetq_lane_f32(0.0f, a_zxy, 3);

            // b_zxy = [z, x, y, 0]
            float32x4_t b_zxy = vsetq_lane_f32(vgetq_lane_f32(b, 2), b, 0);
            b_zxy = vsetq_lane_f32(vgetq_lane_f32(b, 0), b_zxy, 1);
            b_zxy = vsetq_lane_f32(vgetq_lane_f32(b, 1), b_zxy, 2);
            b_zxy = vsetq_lane_f32(0.0f, b_zxy, 3);

            // result = a_yzx * b_zxy - a_zxy * b_yzx
            float32x4_t r = vmulq_f32(a_yzx, b_zxy);
            r = vfmsq_f32(r, a_zxy, b_yzx);  // FMA: r = r - (a_zxy * b_yzx)
            store3_neon(result.data, r);
            return result;
        }
#endif
        Vec result;
        result.data[X] = data[Y] * other.data[Z] - data[Z] * other.data[Y];
        result.data[Y] = data[Z] * other.data[X] - data[X] * other.data[Z];
        result.data[Z] = data[X] * other.data[Y] - data[Y] * other.data[X];
        return result;
    }

    /// \brief Dot product operator.
    /// \param other The other vector.
    /// \return The dot product.
    constexpr auto operator|(const Vec& other) const noexcept -> T { return dot(other); }

    /// \brief Cross product operator (only for 3D).
    /// \param other The other vector.
    /// \return The cross product.
    template<std::size_t NN = N>
        requires(NN == 3)
    [[nodiscard]] constexpr auto operator^(const Vec& other) const noexcept -> Vec
    {
        return cross(other);
    }

    /// \brief Length of the vector.
    /// \return The Euclidean length.
    /// \details In C++26+, this function is constexpr when the vector is known at compile-time.
    MICROLA_CONSTEXPR_TRIG [[nodiscard]] auto length() const noexcept -> T { return std::sqrt(dot(*this)); }

    /// \brief Squared length of the vector.
    /// \return The squared Euclidean length.
    [[nodiscard]] auto length_squared() const noexcept -> T { return dot(*this); }

    /// \brief Magnitude of the vector (alias for length).
    /// \return The Euclidean magnitude.
    /// \details Provided for API compatibility. Magnitude and length are synonymous
    ///          for vectors - both return the Euclidean norm ||v||.
    ///          In C++26+, this function is constexpr when the vector is known at compile-time.
    MICROLA_CONSTEXPR_TRIG [[nodiscard]] auto magnitude() const noexcept -> T { return length(); }

    /// \brief Squared magnitude of the vector (alias for length_squared).
    /// \return The squared Euclidean magnitude.
    /// \details Provided for API compatibility. Avoids the sqrt computation.
    [[nodiscard]] auto magnitude_squared() const noexcept -> T { return length_squared(); }

    /// \brief Normalized vector.
    /// \return The unit vector in the same direction.
    [[nodiscard]] auto normalized() const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && (N == 2 || N == 3 || N == 4))
        {
            const T len_sq = dot(*this);
            if (len_sq != T(0))
            {
                // Fast inverse square root with Newton-Raphson refinement
                float32x2_t len_sq_v = vdup_n_f32(static_cast<float>(len_sq));
                float32x2_t rsqrt = vrsqrte_f32(len_sq_v);  // Initial estimate
                // Newton-Raphson: x' = x * (3 - x^2 * a) / 2
                rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(len_sq_v, rsqrt), rsqrt));
                rsqrt = vmul_f32(rsqrt, vrsqrts_f32(vmul_f32(len_sq_v, rsqrt), rsqrt));
                const T inv_len = static_cast<T>(vget_lane_f32(rsqrt, 0));
                return *this * inv_len;
            }
            return *this;
        }
#endif
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            const T len_sq = dot(*this);
            if (len_sq != T(0))
            {
                float len;
                float len_sq_f = static_cast<float>(len_sq);
                arm_sqrt_f32(len_sq_f, &len);

                const float inv_len = 1.0f / len;
                Vec result;
                arm_scale_f32(data, inv_len, result.data, N);
                return result;
            }
            return *this;
        }
#endif
        // Fallback: compute length from cached length_squared
        const T len_sq = dot(*this);
        if (len_sq != T(0)) [[likely]]
        {
            const T len = std::sqrt(len_sq);
            return *this / len;
        }
        return *this;
    }

    /// \brief Angle between two vectors (in radians).
    /// \param other The other vector.
    /// \return The angle in radians.
    [[nodiscard]] auto angle(const Vec& other) const noexcept -> T
    {
        const T len1 = length();
        const T len2 = other.length();
        const T denom = len1 * len2;
        if (denom == T(0)) [[unlikely]]
        {
            return T(0);
        }
        T cos_theta = dot(other) / denom;
        // Clamp to [-1, 1] to avoid domain errors
        if (cos_theta > T(1))
        {
            cos_theta = T(1);
        }
        if (cos_theta < T(-1))
        {
            cos_theta = T(-1);
        }
        return std::acos(cos_theta);
    }

    /// \brief Get x component (index 0).
    /// \return Reference to x component.
    template<std::size_t NN = N>
        requires(NN >= 1)
    auto x() noexcept -> T&
    {
        return data[X];
    }
    template<std::size_t NN = N>
        requires(NN >= 1)
    [[nodiscard]] auto x() const noexcept -> const T&
    {
        return data[X];
    }

    /// \brief Get y component (index 1).
    /// \return Reference to y component.
    template<std::size_t NN = N>
        requires(NN >= 2)
    auto y() noexcept -> T&
    {
        return data[Y];
    }
    template<std::size_t NN = N>
        requires(NN >= 2)
    [[nodiscard]] auto y() const noexcept -> const T&
    {
        return data[Y];
    }

    /// \brief Get z component (index 2).
    /// \return Reference to z component.
    template<std::size_t NN = N>
        requires(NN >= 3)
    auto z() noexcept -> T&
    {
        return data[Z];
    }
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto z() const noexcept -> const T&
    {
        return data[Z];
    }

    /// \brief Get w component (index 3).
    /// \return Reference to w component.
    template<std::size_t NN = N>
        requires(NN >= 4)
    auto w() noexcept -> T&
    {
        return data[W];
    }
    template<std::size_t NN = N>
        requires(NN >= 4)
    [[nodiscard]] auto w() const noexcept -> const T&
    {
        return data[W];
    }

    /// \brief Approximate equality comparison.
    /// \param other The vector to compare.
    /// \param epsilon The tolerance (defaults to machine epsilon for type T).
    /// \return True if approximately equal.
    [[nodiscard]] auto approx_equal(const Vec& other,
                                    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept -> bool
    {
        for (std::size_t i = 0; i < N; ++i)
        {
            if (std::abs(data[i] - other.data[i]) > epsilon)
            {
                return false;
            }
        }
        return true;
    }

    /// \brief Check whether every component is approximately zero.
    /// \param epsilon Tolerance for floating-point comparisons.
    /// \return True if each component is within epsilon of zero.
    [[nodiscard]] auto is_zero(T epsilon = std::numeric_limits<T>::epsilon() * static_cast<T>(8)) const noexcept -> bool
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (std::abs(data[i]) > epsilon)
                {
                    return false;
                }
            }
            return true;
        }

        for (std::size_t i = 0; i < N; ++i)
        {
            if (data[i] != T(0))
            {
                return false;
            }
        }
        return true;
    }

    /// \brief Check whether every component is approximately one.
    /// \param epsilon Tolerance for floating-point comparisons.
    /// \return True if each component is within epsilon of one.
    [[nodiscard]] auto is_one(T epsilon = std::numeric_limits<T>::epsilon() * static_cast<T>(8)) const noexcept -> bool
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (std::abs(data[i] - T(1)) > epsilon)
                {
                    return false;
                }
            }
            return true;
        }

        for (std::size_t i = 0; i < N; ++i)
        {
            if (data[i] != T(1))
            {
                return false;
            }
        }
        return true;
    }

    /// \brief Linear interpolation between two vectors.
    /// \param other The target vector.
    /// \param t The interpolation parameter [0, 1].
    /// \return The interpolated vector.
    [[nodiscard]] auto lerp(const Vec& other, T t) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] + t * (other.data[i] - data[i]);
        }
        return result;
    }

    /// \brief Cubic Hermite interpolation between two vectors with tangents
    /// \param other The target vector
    /// \param tangent1 The tangent at this vector
    /// \param tangent2 The tangent at the other vector
    /// \param t The interpolation parameter [0, 1]
    /// \return The interpolated vector
    [[nodiscard]] auto cubic_hermite(const Vec& other, const Vec& tangent1, const Vec& tangent2,
                                     T t) const noexcept -> Vec
    {
        const T t2 = t * t;
        const T t3 = t2 * t;
        const T h00 = T(2) * t3 - T(3) * t2 + T(1);
        const T h10 = t3 - T(2) * t2 + t;
        const T h01 = T(-2) * t3 + T(3) * t2;
        const T h11 = t3 - t2;

        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = h00 * data[i] + h10 * tangent1.data[i] + h01 * other.data[i] + h11 * tangent2.data[i];
        }
        return result;
    }

    /// \brief Catmull-Rom spline interpolation (passes through control points)
    /// \param p0 Point before this vector
    /// \param p2 Point after other vector
    /// \param other The target vector (p1)
    /// \param t The interpolation parameter [0, 1]
    /// \return The interpolated vector
    [[nodiscard]] auto catmull_rom(const Vec& other, const Vec& p0, const Vec& p2, T t) const noexcept -> Vec
    {
        const T t2 = t * t;
        const T t3 = t2 * t;

        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = T(0.5) * ((T(2) * data[i]) + (-p0.data[i] + other.data[i]) * t +
                                       (T(2) * p0.data[i] - T(5) * data[i] + T(4) * other.data[i] - p2.data[i]) * t2 +
                                       (-p0.data[i] + T(3) * data[i] - T(3) * other.data[i] + p2.data[i]) * t3);
        }
        return result;
    }

    /// \brief Element-wise (Hadamard) product.
    /// \param other The other vector.
    /// \return The element-wise product.
    [[nodiscard]] auto hadamard(const Vec& other) const noexcept -> Vec
    {
#ifdef CONFIG_MICROLA_NEON
        if constexpr (std::is_same_v<T, float> && N == 3)
        {
            Vec result;
            store3_neon(result.data, vmulq_f32(load3_neon(data), load3_neon(other.data)));
            return result;
        }
#endif
        if constexpr (std::is_same_v<T, float>)
        {
            if (N == 2 || N == 3 || N == 4)
            {
                Vec result;
                float a4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float b4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                float r4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                simd::load_padded4(data, N, a4);
                simd::load_padded4(other.data, N, b4);
                simd::mul4(a4, b4, r4);
                simd::store_extract4(r4, result.data, N);
                return result;
            }
        }
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] * other.data[i];
        }
        return result;
    }

    /// \brief Element-wise multiplication.
    /// \param other The vector to multiply element-wise.
    /// \return The result vector.
    [[nodiscard]] constexpr auto elem_mult(const Vec& other) const noexcept -> Vec { return hadamard(other); }

    /// \brief Compute the distance to another vector.
    /// \param other The vector to compute the distance to.
    /// \return The distance.
    [[nodiscard]] constexpr auto distance(const Vec& other) const noexcept -> T
    {
        T sum = T(0);
        for (std::size_t i = 0; i < N; ++i)
        {
            T diff = data[i] - other.data[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    /// \brief Get minimum element.
    /// \return The minimum element value.
    [[nodiscard]] auto min_element() const noexcept -> T
    {
        T min_val = data[X];
        for (std::size_t i = 1; i < N; ++i)
        {
            if (data[i] < min_val)
            {
                min_val = data[i];
            }
        }
        return min_val;
    }

    /// \brief Get maximum element.
    /// \return The maximum element value.
    [[nodiscard]] auto max_element() const noexcept -> T
    {
        T max_val = data[X];
        for (std::size_t i = 1; i < N; ++i)
        {
            if (data[i] > max_val)
            {
                max_val = data[i];
            }
        }
        return max_val;
    }

    /// \brief Element-wise absolute value.
    /// \return Vector with absolute values.
    [[nodiscard]] auto abs() const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = std::abs(data[i]);
        }
        return result;
    }

    /// \brief Clamp vector elements between min and max.
    /// \param min_val The minimum value.
    /// \param max_val The maximum value.
    /// \return The clamped vector.
    [[nodiscard]] auto clamp(T min_val, T max_val) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = std::min(std::max(data[i], min_val), max_val);
        }
        return result;
    }

    /// \brief Element-wise minimum with another vector.
    /// \param other The other vector.
    /// \return Vector with element-wise minimums.
    [[nodiscard]] auto min(const Vec& other) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = std::min(data[i], other.data[i]);
        }
        return result;
    }

    /// \brief Element-wise maximum with another vector.
    /// \param other The other vector.
    /// \return Vector with element-wise maximums.
    [[nodiscard]] auto max(const Vec& other) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = std::max(data[i], other.data[i]);
        }
        return result;
    }

    /// \brief Project this vector onto another vector.
    /// \param other The vector to project onto.
    /// \return The projection of this vector onto other.
    [[nodiscard]] auto project(const Vec& other) const noexcept -> Vec
    {
        const T dot_prod = dot(other);
        const T other_len_sq = other.dot(other);
        if (other_len_sq == T(0))
        {
            return Vec();
        }
        return other * (dot_prod / other_len_sq);
    }

    /// \brief Reject this vector from another (perpendicular component).
    /// \param other The vector to reject from.
    /// \return The component of this vector perpendicular to other.
    [[nodiscard]] auto reject(const Vec& other) const noexcept -> Vec { return *this - project(other); }

    /// \brief Signed angle between two 3D vectors relative to a normal axis.
    /// \param other The other vector.
    /// \param normal The normal vector defining the plane (should be normalized).
    /// \return The signed angle in radians [-π, π].
    template<std::size_t NN = N>
        requires(NN == 3)
    [[nodiscard]] auto signed_angle(const Vec& other, const Vec& normal) const noexcept -> T
    {
        T cos_theta = dot(other) / (length() * other.length());
        // Clamp to [-1, 1] to avoid domain errors
        if (cos_theta > T(1))
        {
            cos_theta = T(1);
        }
        if (cos_theta < T(-1))
        {
            cos_theta = T(-1);
        }
        T angle = std::acos(cos_theta);
        // Determine sign using the normal
        Vec cross_prod = cross(other);
        if (cross_prod.dot(normal) < T(0))
        {
            angle = -angle;
        }
        return angle;
    }

    /// \brief Rotate this vector around an axis by an angle (Rodrigues' rotation formula).
    /// \param axis The rotation axis (should be normalized).
    /// \param angle The rotation angle in radians.
    /// \return The rotated vector.
    template<std::size_t NN = N>
        requires(NN == 3)
    [[nodiscard]] auto rotate(const Vec& axis, T angle) const noexcept -> Vec
    {
        const T cos_a = std::cos(angle);
        const T sin_a = std::sin(angle);
        // Rodrigues' formula: v' = v*cos(θ) + (k×v)*sin(θ) + k*(k·v)*(1-cos(θ))
        const Vec k_cross_v = axis.cross(*this);
        const T k_dot_v = axis.dot(*this);
        Vec result;
        for (std::size_t i = 0; i < 3; ++i)
        {
            result.data[i] = data[i] * cos_a + k_cross_v.data[i] * sin_a + axis.data[i] * k_dot_v * (T(1) - cos_a);
        }
        return result;
    }

    /// \brief Static factory: create a zero vector
    /// \return Zero vector with all components set to 0
    static constexpr auto zero() noexcept -> Vec { return zero_impl(detail::make_index_sequence<N>{}); }

    /// \brief Static factory: create a vector with all components set to 1
    /// \return Vector with all components set to 1
    static constexpr auto one() noexcept -> Vec { return filled(static_cast<T>(1)); }

    /// \brief Create a vector filled with `value` (constexpr-capable where supported).
    static constexpr auto filled(T value) noexcept -> Vec
    {
        return filled_impl(detail::make_index_sequence<N>{}, value);
    }

    /// \brief Static factory: create unit vector along X axis (1, 0, 0, ...)
    /// \return Unit X vector
    static auto unit_x() noexcept -> Vec
    {
        Vec result = zero();
        result.data[X] = T(1);
        return result;
    }

    /// \brief Static factory: create unit vector along Y axis (0, 1, 0, ...)
    /// \return Unit Y vector
    template<std::size_t NN = N>
        requires(NN >= 2)
    static auto unit_y() noexcept -> Vec
    {
        Vec result = zero();
        result.data[Y] = T(1);
        return result;
    }

    /// \brief Static factory: create unit vector along Z axis (0, 0, 1, ...)
    /// \return Unit Z vector
    template<std::size_t NN = N>
        requires(NN >= 3)
    static auto unit_z() noexcept -> Vec
    {
        Vec result = zero();
        result.data[Z] = T(1);
        return result;
    }

    /// \brief Static factory: create unit vector along W axis (0, 0, 0, 1)
    /// \return Unit W vector
    template<std::size_t NN = N>
        requires(NN >= 4)
    static auto unit_w() noexcept -> Vec
    {
        Vec result = zero();
        result.data[W] = T(1);
        return result;
    }

    /// \brief Safe normalization (returns zero vector if magnitude is too small)
    /// \param epsilon Threshold below which to return zero vector (defaults to machine epsilon).
    /// \return Normalized vector, or zero vector if length < epsilon
    [[nodiscard]] auto safe_normalized(T epsilon = std::numeric_limits<T>::epsilon()) const noexcept -> Vec
    {
        const T len = length();
        if (len < epsilon) [[unlikely]]
        {
            return zero();
        }
        return *this / len;
    }

    /// \brief Clamp all components to range [min, max]
    /// \param min Minimum value
    /// \param max Maximum value
    /// \return Vector with clamped components
    [[nodiscard]] auto clamped(T min, T max) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            if (data[i] < min)
            {
                result.data[i] = min;
            }
            else if (data[i] > max)
            {
                result.data[i] = max;
            }
            else
            {
                result.data[i] = data[i];
            }
        }
        return result;
    }

    /// \brief Saturate all components to range [0, 1]
    /// \return Vector with saturated components
    [[nodiscard]] auto saturated() const noexcept -> Vec { return clamped(T(0), T(1)); }

    /// \brief Get 2D swizzle (only for N >= 2)
    /// \return Vec<T,2> with x,y components
    template<std::size_t NN = N>
        requires(NN >= 2)
    [[nodiscard]] auto xy() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[X], data[Y]);
    }

    /// \brief Get 2D swizzle yx (only for N >= 2)
    /// \return Vec<T,2> with y,x components
    template<std::size_t NN = N>
        requires(NN >= 2)
    [[nodiscard]] auto yx() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[Y], data[X]);
    }

    /// \brief Get 3D swizzle (only for N >= 3)
    /// \return Vec<T,3> with x,y,z components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto xyz() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[X], data[Y], data[Z]);
    }

    /// \brief Get xz swizzle (only for N >= 3)
    /// \return Vec<T,2> with x,z components
    template<std::size_t NN = N>
        requires(NN >= 3)
    auto xz() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[X], data[Z]);
    }

    /// \brief Get yz swizzle (only for N >= 3)
    /// \return Vec<T,2> with y,z components
    template<std::size_t NN = N>
        requires(NN >= 3)
    auto yz() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[Y], data[Z]);
    }

    /// \brief Get zx swizzle (only for N >= 3)
    /// \return Vec<T,2> with z,x components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto zx() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[Z], data[X]);
    }

    /// \brief Get zy swizzle (only for N >= 3)
    /// \return Vec<T,2> with z,y components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto zy() const noexcept -> Vec<T, 2>
    {
        return Vec<T, 2>(data[Z], data[Y]);
    }

    /// \brief Get xzy swizzle (only for N >= 3)
    /// \return Vec<T,3> with x,z,y components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto xzy() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[X], data[Z], data[Y]);
    }

    /// \brief Get yxz swizzle (only for N >= 3)
    /// \return Vec<T,3> with y,x,z components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto yxz() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[Y], data[X], data[Z]);
    }

    /// \brief Get yzx swizzle (only for N >= 3)
    /// \return Vec<T,3> with y,z,x components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto yzx() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[Y], data[Z], data[X]);
    }

    /// \brief Get zxy swizzle (only for N >= 3)
    /// \return Vec<T,3> with z,x,y components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto zxy() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[Z], data[X], data[Y]);
    }

    /// \brief Get zyx swizzle (only for N >= 3)
    /// \return Vec<T,3> with z,y,x components
    template<std::size_t NN = N>
        requires(NN >= 3)
    [[nodiscard]] auto zyx() const noexcept -> Vec<T, 3>
    {
        return Vec<T, 3>(data[Z], data[Y], data[X]);
    }

    /// \brief Convert to homogeneous coordinates (add w=1 component)
    /// \return Vec<T,N+1> with w=1 appended
    [[nodiscard]] auto to_homogeneous() const noexcept -> Vec<T, N + 1>
    {
        Vec<T, N + 1> result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i];
        }
        result.data[N] = T(1);
        return result;
    }

    /// \brief Convert from homogeneous coordinates (divide by w, drop w component)
    /// \return Vec<T,N-1> with perspective division applied
    template<std::size_t NN = N>
        requires(NN >= 2)
    [[nodiscard]] auto from_homogeneous() const noexcept -> Vec<T, N - 1>
    {
        Vec<T, N - 1> result;
        const T w = data[N - 1];
        if (std::abs(w) > std::numeric_limits<T>::epsilon()) [[likely]]
        {
            const T inv_w = T(1) / w;
            for (std::size_t i = 0; i < N - 1; ++i)
            {
                result.data[i] = data[i] * inv_w;
            }
        }
        else [[unlikely]]
        {
            // w is too small, return as-is without division
            for (std::size_t i = 0; i < N - 1; ++i)
            {
                result.data[i] = data[i];
            }
        }
        return result;
    }

    /// \brief Fused multiply-add operation.
    /// \param mul Vector to multiply with this.
    /// \param add Vector to add after multiplication.
    /// \return Result of (this * mul) + add, potentially using hardware FMA.
    [[nodiscard]] constexpr auto fma(const Vec& mul, const Vec& add) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] * mul.data[i] + add.data[i];
        }
        return result;
    }

    /// \brief Fused multiply-add with scalar.
    /// \param scalar Scalar to multiply with this.
    /// \param add Vector to add after multiplication.
    /// \return Result of (this * scalar) + add.
    [[nodiscard]] constexpr auto fma(T scalar, const Vec& add) const noexcept -> Vec
    {
        Vec result;
        for (std::size_t i = 0; i < N; ++i)
        {
            result.data[i] = data[i] * scalar + add.data[i];
        }
        return result;
    }

    /// \brief Scale the vector by a scalar using operator*.
    /// \param scalar The scalar value to scale by.
    /// \return A new vector scaled by the scalar.
    [[nodiscard]] constexpr auto scale(T scalar) const noexcept -> Vec
    {
        return *this * scalar;  // Leverage SIMD optimizations in operator*
    }
};

/// \brief Scalar multiplication (commutative).
/// \tparam T The type.
/// \tparam N The dimension.
/// \param scalar The scalar.
/// \param v The vector.
/// \return The result vector.
template<typename T, std::size_t N>
[[nodiscard]] constexpr auto operator*(T scalar, const Vec<T, N>& v) noexcept -> Vec<T, N>
{
    return v * scalar;
}

// ============================================================================
// Type Aliases for Common Vector Types
// ============================================================================

// Float vectors
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;

// Double vectors
using Vec2d = Vec<double, 2>;
using Vec3d = Vec<double, 3>;
using Vec4d = Vec<double, 4>;

// Integer vectors
using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;
using Vec4i = Vec<int, 4>;

// Unsigned integer vectors
using Vec2u = Vec<unsigned int, 2>;
using Vec3u = Vec<unsigned int, 3>;
using Vec4u = Vec<unsigned int, 4>;

}  // namespace microla
