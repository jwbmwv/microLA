// SPDX-License-Identifier: Apache-2.0
/// @file vector_view.hpp
/// @brief Non-owning vector views for subvector operations
/// @details CRITICAL SAFETY: Views do not manage lifetime - the underlying data
///          must remain valid for the entire lifetime of the view.
///          See matrix_view.hpp for comprehensive lifetime safety guidelines.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#pragma once

#include <cstdint>
#include <cassert>
#include "vector.hpp"
#include "compiler_features.hpp"
#include "simd_helpers.hpp"

namespace microla
{

template<typename T, std::size_t N>
class VectorView
{
public:
    /**
     * @brief Construct a non-owning view into parent linear storage.
     *
     * @param data Pointer to the parent buffer containing elements.
     * @param parent_size Number of elements in the parent buffer.
     * @param start_index Index of the first element in the parent buffer to include in the view.
     * @param stride Step between successive elements in the view (default: 1).
     *
     * @note The view does not take ownership of `data`. The caller must ensure
     *       that the parent storage outlives this view. When `MICROLA_DEBUG` is
     *       defined, runtime bounds checks are performed.
     */
    VectorView(T* data, std::size_t parent_size, std::size_t start_index, std::size_t stride = 1) noexcept
        : m_data(data), m_parent_size(parent_size), m_start(start_index), m_stride(stride)
    {
#ifdef MICROLA_DEBUG
        assert(N > 0);
        assert(start_index < parent_size && "VectorView: start_index out of bounds");
        assert(start_index + (N - 1) * stride < parent_size && "VectorView: view exceeds parent bounds");
#endif
    }

    /**
     * @brief Mutable element access.
     *
     * @param idx Index within the view [0, N).
     * @return Reference to the underlying element in the parent buffer.
     */
    auto operator[](std::size_t idx) noexcept -> T&
    {
#ifdef MICROLA_DEBUG
        assert(idx < N && "VectorView: index out of bounds");
#endif
        return m_data[m_start + idx * m_stride];
    }

    /**
     * @brief Const element access.
     *
     * @param idx Index within the view [0, N).
     * @return Const reference to the underlying element in the parent buffer.
     */
    auto operator[](std::size_t idx) const noexcept -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(idx < N && "VectorView: index out of bounds");
#endif
        return m_data[m_start + idx * m_stride];
    }

    /**
     * @brief Copy values from a `Vec` into the view (writes to parent storage).
     *
     * @param src Source vector to copy from. Must be of size `N`.
     */
    void set(const Vec<T, N>& src) noexcept
    {
        if constexpr (std::is_same_v<T, float>)
        {
            if (m_stride == 1)
            {
                /* cppcheck-suppress invalidPointerCast */
                microla::simd::copy_n_float(src.data, m_data + m_start, N);
                return;
            }
        }

        for (std::size_t i = 0; i < N; ++i)
        {
            (*this)[i] = src[i];
        }
    }

    /**
     * @brief Produce an owning `Vec` copy of the view's values.
     *
     * @return A `Vec<T, N>` containing the values referenced by the view.
     */
    [[nodiscard]] auto to_vec() const noexcept -> Vec<T, N>
    {
        Vec<T, N> out;
        if constexpr (std::is_same_v<T, float>)
        {
            if (m_stride == 1)
            {
                /* cppcheck-suppress invalidPointerCast */
                microla::simd::copy_n_float(m_data + m_start, out.data, N);
                return out;
            }
        }

        for (std::size_t i = 0; i < N; ++i)
        {
            out[i] = (*this)[i];
        }
        return out;
    }

    /**
     * @brief Fill all elements referenced by the view with a scalar value.
     *
     * @param value Value to write into each referenced element.
     */
    void fill(T value) noexcept
    {
        if constexpr (std::is_same_v<T, float>)
        {
            if (m_stride == 1)
            {
                /* cppcheck-suppress invalidPointerCast */
                microla::simd::fill_float(m_data + m_start, N, value);
                return;
            }
        }

        for (std::size_t i = 0; i < N; ++i)
        {
            (*this)[i] = value;
        }
    }

    /**
     * @brief Number of elements in the view (compile-time constant `N`).
     */
    [[nodiscard]] constexpr auto size() const noexcept -> std::size_t { return N; }

    /**
     * @brief Stride (in parent elements) between successive elements in the view.
     */
    [[nodiscard]] auto stride() const noexcept -> std::size_t { return m_stride; }

private:
    T* m_data;                 /**< Pointer to parent storage. */
    std::size_t m_parent_size; /**< Size of the parent buffer. */
    std::size_t m_start;       /**< Start index within parent buffer. */
    std::size_t m_stride;      /**< Stride between elements in the view. */
};

template<typename T, std::size_t N>
class ConstVectorView
{
public:
    /**
     * @brief Construct a read-only, non-owning view into parent linear storage.
     *
     * @param data Pointer to the parent buffer containing elements.
     * @param parent_size Number of elements in the parent buffer.
     * @param start_index Index of the first element in the parent buffer to include in the view.
     * @param stride Step between successive elements in the view (default: 1).
     */
    ConstVectorView(const T* data, std::size_t parent_size, std::size_t start_index, std::size_t stride = 1) noexcept
        : m_data(data), m_parent_size(parent_size), m_start(start_index), m_stride(stride)
    {
#ifdef MICROLA_DEBUG
        assert(N > 0);
        assert(start_index < parent_size && "ConstVectorView: start_index out of bounds");
        assert(start_index + (N - 1) * stride < parent_size && "ConstVectorView: view exceeds parent bounds");
#endif
    }

    /**
     * @brief Const element access.
     *
     * @param idx Index within the view [0, N).
     * @return Const reference to the underlying element in the parent buffer.
     */
    auto operator[](std::size_t idx) const noexcept -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(idx < N && "ConstVectorView: index out of bounds");
#endif
        return m_data[m_start + idx * m_stride];
    }

    /**
     * @brief Produce an owning `Vec` copy of the view's values.
     *
     * @return A `Vec<T, N>` containing the values referenced by the view.
     */
    [[nodiscard]] auto to_vec() const noexcept -> Vec<T, N>
    {
        Vec<T, N> out;
        for (std::size_t i = 0; i < N; ++i)
        {
            out[i] = (*this)[i];
        }
        return out;
    }

    /**
     * @brief Number of elements in the view (compile-time constant `N`).
     */
    [[nodiscard]] constexpr auto size() const noexcept -> std::size_t { return N; }

    /**
     * @brief Stride (in parent elements) between successive elements in the view.
     */
    [[nodiscard]] auto stride() const noexcept -> std::size_t { return m_stride; }

private:
    const T* m_data;           /**< Pointer to parent storage (read-only). */
    std::size_t m_parent_size; /**< Size of the parent buffer. */
    std::size_t m_start;       /**< Start index within parent buffer. */
    std::size_t m_stride;      /**< Stride between elements in the view. */
};

}  // namespace microla
