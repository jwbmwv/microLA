// SPDX-License-Identifier: Apache-2.0
/// @file simd_helpers.hpp
/// @brief Small SIMD utility helpers used by view implementations.
///
/// These helpers provide a thin abstraction over common float operations
/// (fill and copy) that take advantage of platform SIMD intrinsics when
/// available (AVX, NEON). When SIMD is not enabled the implementations
/// fall back to well-optimized scalar loops.
///
/// The functions are intentionally low-level and operate on raw pointers
/// to enable use from view implementations without introducing
/// dependencies or allocations.
///
/// @note All functions are declared `inline` and `noexcept` to allow
///       compile-time inlining and to simplify use in header-only code.
/// @since 1.0
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "compiler_features.hpp"

#if defined(CONFIG_MICROLA_AVX)
#include <immintrin.h>
#endif

#if defined(CONFIG_MICROLA_NEON)
#include <arm_neon.h>
#endif

namespace microla
{
namespace simd
{
/// @brief Fill a contiguous array of floats with a constant value.
///
/// This routine writes `n` elements of `value` into the memory pointed
/// to by `dst`. When available, platform-specific SIMD intrinsics are
/// used to accelerate the operation (AVX for x86_64, NEON for ARM). If
/// SIMD support is not compiled in, the implementation falls back to
/// `std::fill_n`.
///
/// @param[out] dst Pointer to the destination buffer. Must be valid for
///                 `n` elements.
/// @param[in] n   Number of floats to write.
/// @param[in] value Value to write into each element.
///
/// @pre `dst` must be non-null when `n > 0`.
/// @thread_safety Not thread-safe for overlapping buffers; callers must
///                ensure externally that concurrent writes are safe.
inline void fill_float(float* dst, std::size_t n, float value) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    std::size_t i = 0;
    const std::size_t stride = 8;
    __m256 v = _mm256_set1_ps(value);
    for (; i + stride <= n; i += stride)
    {
        _mm256_storeu_ps(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = value;
    }
#elif defined(CONFIG_MICROLA_NEON)
    std::size_t i = 0;
    const std::size_t stride = 4;
    float32x4_t v = vdupq_n_f32(value);
    for (; i + stride <= n; i += stride)
    {
        vst1q_f32(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = value;
    }
#else
    std::fill_n(dst, n, value);
#endif
}

/// @brief Copy `n` floats from `src` to `dst`.
///
/// Performs an element-wise copy of `n` floats from the source buffer to
/// the destination buffer. When platform SIMD is available, wide loads and
/// stores are used for better throughput. The implementation assumes that
/// the caller has ensured that the copy is safe for the given buffers
/// (i.e. non-overlapping or otherwise handled by the caller).
///
/// @param[in] src Pointer to the source buffer. Must be valid for `n` elements.
/// @param[out] dst Pointer to the destination buffer. Must be valid for `n` elements.
/// @param[in] n Number of floats to copy.
///
/// @pre If `n > 0`, both `src` and `dst` must be non-null. Overlapping
///      regions must be handled by the caller.
/// @thread_safety Not thread-safe for overlapping buffers.
inline void copy_n_float(const float* src, float* dst, std::size_t n) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    std::size_t i = 0;
    const std::size_t stride = 8;
    for (; i + stride <= n; i += stride)
    {
        __m256 v = _mm256_loadu_ps(src + i);
        _mm256_storeu_ps(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = src[i];
    }
#elif defined(CONFIG_MICROLA_NEON)
    std::size_t i = 0;
    const std::size_t stride = 4;
    for (; i + stride <= n; i += stride)
    {
        float32x4_t v = vld1q_f32(src + i);
        vst1q_f32(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = src[i];
    }
#else
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i] = src[i];
    }
#endif
}

/// @brief Copy exactly 4 floats using the widest available vector instruction.
///
/// This provides a small, convenience helper for places that operate on
/// fixed-size 4-element blocks. On NEON/AVX capable platforms this will
/// use a single vector load/store; otherwise it falls back to scalar
/// element copies.
///
/// @param[in] src Source pointer (must be valid for 4 floats).
/// @param[out] dst Destination pointer (must be valid for 4 floats).
inline void copy4_float(const float* src, float* dst) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    __m128 v = _mm_loadu_ps(src);
    _mm_storeu_ps(dst, v);
#elif defined(CONFIG_MICROLA_NEON)
    float32x4_t v = vld1q_f32(src);
    vst1q_f32(dst, v);
#else
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
#endif
}

/// @brief Copy `n` floats using arbitrary source/destination strides.
///
/// For the common contiguous case (both strides == 1) this forwards to
/// `copy_n_float` for best throughput. For other stride combinations a
/// simple element-wise loop is used.
///
/// @param[in] src Pointer to source buffer.
/// @param[in] src_stride Stride (in elements) between consecutive source items.
/// @param[out] dst Pointer to destination buffer.
/// @param[in] dst_stride Stride (in elements) between consecutive destination items.
/// @param[in] n Number of elements to copy.
inline void strided_copy_float(const float* src, std::size_t src_stride, float* dst, std::size_t dst_stride,
                               std::size_t n) noexcept
{
    if (src_stride == 1 && dst_stride == 1)
    {
        copy_n_float(src, dst, n);
        return;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i * dst_stride] = src[i * src_stride];
    }
}

// ==================== Generic SIMD Helpers ====================

/// @brief Generic fill for any arithmetic type
/// @tparam T Arithmetic type
/// @param dst Destination pointer
/// @param n Number of elements
/// @param value Fill value
template<typename T>
inline void fill(T* dst, std::size_t n, T value) noexcept
{
    static_assert(std::is_arithmetic_v<T>, "fill requires arithmetic type");
    if constexpr (std::is_same_v<T, float>)
    {
        fill_float(dst, n, value);
    }
    else
    {
        std::fill_n(dst, n, value);
    }
}

/// @brief Generic copy for any arithmetic type
/// @tparam T Arithmetic type
/// @param src Source pointer
/// @param dst Destination pointer
/// @param n Number of elements
template<typename T>
inline void copy_n(const T* src, T* dst, std::size_t n) noexcept
{
    static_assert(std::is_arithmetic_v<T>, "copy_n requires arithmetic type");
    if constexpr (std::is_same_v<T, float>)
    {
        copy_n_float(src, dst, n);
    }
    else
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            dst[i] = src[i];
        }
    }
}

/// @brief Generic strided copy for any arithmetic type
/// @tparam T Arithmetic type
template<typename T>
inline void strided_copy(const T* src, std::size_t src_stride, T* dst, std::size_t dst_stride, std::size_t n) noexcept
{
    static_assert(std::is_arithmetic_v<T>, "strided_copy requires arithmetic type");
    if constexpr (std::is_same_v<T, float>)
    {
        strided_copy_float(src, src_stride, dst, dst_stride, n);
    }
    else
    {
        if (src_stride == 1 && dst_stride == 1)
        {
            copy_n(src, dst, n);
            return;
        }
        for (std::size_t i = 0; i < n; ++i)
        {
            dst[i * dst_stride] = src[i * src_stride];
        }
    }
}

/// @brief Load up to 4 floats from `src` into `out`, padding with `pad`.
///
/// Useful for handling N==3 vector cases where a 4-wide SIMD load is
/// convenient. `n` specifies how many valid elements are in `src` (0..4).
inline void load_padded4(const float* src, std::size_t n, float out[4], float pad = 0.0F) noexcept
{
    std::size_t i = 0;
    for (; i < n && i < 4; ++i)
    {
        out[i] = src[i];
    }
    for (; i < 4; ++i)
    {
        out[i] = pad;
    }
}

/// @brief Store the first `n` elements from `src4` into `dst`.
inline void store_extract4(const float src4[4], float* dst, std::size_t n) noexcept
{
    for (std::size_t i = 0; i < n && i < 4; ++i)
    {
        dst[i] = src4[i];
    }
}

/// @brief Element-wise add of 4 floats: out = a + b.
inline void add4(const float a[4], const float b[4], float out[4]) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
    __m128 vr = _mm_add_ps(va, vb);
    _mm_storeu_ps(out, vr);
#elif defined(CONFIG_MICROLA_NEON)
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vaddq_f32(va, vb);
    vst1q_f32(out, vr);
#else
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
    out[3] = a[3] + b[3];
#endif
}

/// @brief Element-wise subtract of 4 floats: out = a - b.
inline void sub4(const float a[4], const float b[4], float out[4]) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
    __m128 vr = _mm_sub_ps(va, vb);
    _mm_storeu_ps(out, vr);
#elif defined(CONFIG_MICROLA_NEON)
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vsubq_f32(va, vb);
    vst1q_f32(out, vr);
#else
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
    out[3] = a[3] - b[3];
#endif
}

/// @brief Element-wise multiply of 4 floats: out = a * b.
inline void mul4(const float a[4], const float b[4], float out[4]) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
    __m128 vr = _mm_mul_ps(va, vb);
    _mm_storeu_ps(out, vr);
#elif defined(CONFIG_MICROLA_NEON)
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
    float32x4_t vr = vmulq_f32(va, vb);
    vst1q_f32(out, vr);
#else
    out[0] = a[0] * b[0];
    out[1] = a[1] * b[1];
    out[2] = a[2] * b[2];
    out[3] = a[3] * b[3];
#endif
}

/// @brief Fused multiply-add accumulate: r += a * b for 4-element vectors.
inline void fma4_accumulate(float r[4], const float a[4], const float b[4]) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    __m128 vr = _mm_loadu_ps(r);
    __m128 va = _mm_loadu_ps(a);
    __m128 vb = _mm_loadu_ps(b);
#if defined(__FMA__)
    vr = _mm_fmadd_ps(va, vb, vr);
#else
    __m128 tmp = _mm_mul_ps(va, vb);
    vr = _mm_add_ps(vr, tmp);
#endif
    _mm_storeu_ps(r, vr);
#elif defined(CONFIG_MICROLA_NEON)
    float32x4_t vr = vld1q_f32(r);
    float32x4_t va = vld1q_f32(a);
    float32x4_t vb = vld1q_f32(b);
#if defined(__ARM_FEATURE_FMA)
    vr = vfmaq_f32(vr, va, vb);
#else
    vr = vaddq_f32(vr, vmulq_f32(va, vb));
#endif
    vst1q_f32(r, vr);
#else
    for (std::size_t i = 0; i < 4; ++i)
    {
        r[i] += a[i] * b[i];
    }
#endif
}

/// @brief Gather `n` floats from `src` using an index array `idx` into `dst`.
///
/// This is a simple scalar gather helper. For small fixed-size gathers
/// (4-element) callers can use `gather4_float` which may be optimized
/// by platform intrinsics when available.
inline void gather_n_float(const float* src, const std::size_t* idx, float* dst, std::size_t n) noexcept
{
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i] = src[idx[i]];
    }
}

/// @brief Scatter `n` floats from `src` into `dst` using index array `idx`.
inline void scatter_n_float(const float* src, float* dst, const std::size_t* idx, std::size_t n) noexcept
{
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[idx[i]] = src[i];
    }
}

/// @brief Gather exactly 4 floats (scalar fallback; AVX2 gather if available).
inline void gather4_float(const float* src, const std::size_t idx[4], float out[4]) noexcept
{
#if defined(CONFIG_MICROLA_AVX) && defined(__AVX2__)
    int32_t ix[4] = {static_cast<int32_t>(idx[0]), static_cast<int32_t>(idx[1]), static_cast<int32_t>(idx[2]),
                     static_cast<int32_t>(idx[3])};
    // cppcheck-suppress invalidPointerCast - intrinsic requires loading integer indices as __m128i
    __m128 v = _mm_i32gather_ps(src, _mm_loadu_si128(reinterpret_cast<const __m128i*>(ix)), 4);
    _mm_storeu_ps(out, v);
    return;
#else
    // Try a NEON-friendly contiguous check first for common cases
#if defined(CONFIG_MICROLA_NEON)
    if (idx[1] == idx[0] + 1 && idx[2] == idx[0] + 2 && idx[3] == idx[0] + 3)
    {
        float32x4_t v = vld1q_f32(src + idx[0]);
        vst1q_f32(out, v);
        return;
    }
#endif
    out[0] = src[idx[0]];
    out[1] = src[idx[1]];
    out[2] = src[idx[2]];
    out[3] = src[idx[3]];
#endif
}

/// @brief Scatter exactly 4 floats from `in4` into `dst` at the indices `idx`.
inline void scatter4_float(const float in4[4], float* dst, const std::size_t idx[4]) noexcept
{
    dst[idx[0]] = in4[0];
    dst[idx[1]] = in4[1];
    dst[idx[2]] = in4[2];
    dst[idx[3]] = in4[3];
}

/// @brief Gather `n` floats from `src` using `idx`, but detect contiguous runs and
///        forward to `copy_n_float` for improved throughput when possible.
inline void gather_n_optimized(const float* src, const std::size_t* idx, float* dst, std::size_t n) noexcept
{
    if (n == 0)
    {
        return;
    }
    // Detect contiguous run starting at idx[0]
    bool contiguous = true;
    for (std::size_t i = 1; i < n; ++i)
    {
        if (idx[i] != idx[0] + i)
        {
            contiguous = false;
            break;
        }
    }
    if (contiguous)
    {
        copy_n_float(src + idx[0], dst, n);
        return;
    }
    // Fallback scalar gather
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i] = src[idx[i]];
    }
}

/// @brief Scatter `n` floats from `src` into `dst` using `idx`, but detect
///        contiguous target ranges and forward to `copy_n_float` when possible.
inline void scatter_n_optimized(const float* src, float* dst, const std::size_t* idx, std::size_t n) noexcept
{
    if (n == 0)
    {
        return;
    }
    bool contiguous = true;
    for (std::size_t i = 1; i < n; ++i)
    {
        if (idx[i] != idx[0] + i)
        {
            contiguous = false;
            break;
        }
    }
    if (contiguous)
    {
        copy_n_float(src, dst + idx[0], n);
        return;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[idx[i]] = src[i];
    }
}

/// @brief Gather with a fixed start and stride: dst[i] = src[start + i*stride].
///        If stride==1 this forwards to `copy_n_float` for best throughput.
inline void gather_strided_float(const float* src, std::size_t start, std::size_t stride, float* dst,
                                 std::size_t n) noexcept
{
    if (n == 0)
    {
        return;
    }
    if (stride == 1)
    {
        copy_n_float(src + start, dst, n);
        return;
    }
    // Fast-path for exactly 4-element gathers on AVX2 using vectorized gather
#if defined(CONFIG_MICROLA_AVX) && defined(__AVX2__)
    if (n == 4)
    {
        int ix[4];
        ix[0] = static_cast<int>(start + stride * 0);
        ix[1] = static_cast<int>(start + stride * 1);
        ix[2] = static_cast<int>(start + stride * 2);
        ix[3] = static_cast<int>(start + stride * 3);
        // cppcheck-suppress invalidPointerCast - intrinsic requires loading integer indices as __m128i
        __m128 v = _mm_i32gather_ps(src, _mm_loadu_si128(reinterpret_cast<const __m128i*>(ix)), 4);
        _mm_storeu_ps(dst, v);
        return;
    }
#endif
#if defined(CONFIG_MICROLA_NEON)
    // NEON has no scalar gather; if n==4 and indices are contiguous we already handled stride==1.
    // For small fixed gathers, perform individual loads into a vector register for better locality.
    if (n == 4)
    {
        float tmp[4];
        tmp[0] = src[start + stride * 0];
        tmp[1] = src[start + stride * 1];
        tmp[2] = src[start + stride * 2];
        tmp[3] = src[start + stride * 3];
        float32x4_t v = vld1q_f32(tmp);
        vst1q_f32(dst, v);
        return;
    }
#endif
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i] = src[start + i * stride];
    }
}

/// @brief Scatter with a fixed start and stride: dst[start + i*stride] = src[i].
inline void scatter_strided_float(const float* src, float* dst, std::size_t start, std::size_t stride,
                                  std::size_t n) noexcept
{
    if (n == 0)
    {
        return;
    }
    if (stride == 1)
    {
        copy_n_float(src, dst + start, n);
        return;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[start + i * stride] = src[i];
    }
}

// ==================== Double Precision SIMD Support ====================

/// @brief Fill array of doubles with constant value (SIMD-optimized)
/// @param dst Destination buffer
/// @param n Number of doubles to write
/// @param value Value to write
inline void fill_double(double* dst, std::size_t n, double value) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    std::size_t i = 0;
    const std::size_t stride = 4;  // AVX 256-bit = 4 doubles
    __m256d v = _mm256_set1_pd(value);
    for (; i + stride <= n; i += stride)
    {
        _mm256_storeu_pd(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = value;
    }
#elif defined(CONFIG_MICROLA_NEON) && defined(__aarch64__)
    // ARM64 NEON supports 128-bit double precision (2 doubles)
    std::size_t i = 0;
    const std::size_t stride = 2;
    float64x2_t v = vdupq_n_f64(value);
    for (; i + stride <= n; i += stride)
    {
        vst1q_f64(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = value;
    }
#else
    std::fill_n(dst, n, value);
#endif
}

/// @brief Copy n doubles from src to dst (SIMD-optimized)
inline void copy_n_double(const double* src, double* dst, std::size_t n) noexcept
{
#if defined(CONFIG_MICROLA_AVX)
    std::size_t i = 0;
    const std::size_t stride = 4;
    for (; i + stride <= n; i += stride)
    {
        __m256d v = _mm256_loadu_pd(src + i);
        _mm256_storeu_pd(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = src[i];
    }
#elif defined(CONFIG_MICROLA_NEON) && defined(__aarch64__)
    std::size_t i = 0;
    const std::size_t stride = 2;
    for (; i + stride <= n; i += stride)
    {
        float64x2_t v = vld1q_f64(src + i);
        vst1q_f64(dst + i, v);
    }
    for (; i < n; ++i)
    {
        dst[i] = src[i];
    }
#else
    for (std::size_t i = 0; i < n; ++i)
    {
        dst[i] = src[i];
    }
#endif
}

}  // namespace simd
}  // namespace microla
