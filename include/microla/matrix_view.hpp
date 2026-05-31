// SPDX-License-Identifier: Apache-2.0
/// @file matrix_view.hpp
/// @brief Non-owning matrix views for submatrix operations
/// @details Provides lightweight matrix views without data copying, enabling
///          efficient block operations and submatrix manipulation.
///
/// @section lifetime_safety Lifetime Safety Guidelines
/// - **Rule 1**: Never store a view longer than the data it references
/// - **Rule 2**: Pass views by value (they're just pointers + dimensions)
/// - **Rule 3**: Treat views like iterators - they can dangle
/// - **Rule 4**: Use views in function parameters, not as class members (unless carefully managed)
///
/// @section safe_usage Safe Usage Patterns
/// @code
/// // SAFE: View used immediately within scope
/// void process(const Mat<float, 4, 4>& matrix) {
///     auto view = matrix.view();
///     // Use view here - matrix is still alive
/// }
///
/// // UNSAFE: View outlives the data
/// auto get_view() {
///     Mat<float, 4, 4> temp;
///     return temp.view();  // DANGER: temp destroyed, view dangles
/// }
/// @endcode
///
/// @section practical_examples Practical Usage Examples
///
/// **Example 1: Block Matrix Assembly**
/// @code{.cpp}
/// // Build a 4×4 transform from rotation + translation
/// Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();
/// Mat<float, 3, 3> rotation = Mat<float, 3, 3>::rotation_z(deg_to_rad(45.0f));
/// Vec<float, 3> translation{1.0f, 2.0f, 3.0f};
///
/// // Write rotation block (top-left 3×3)
/// auto rot_view = MatrixView<float, 3, 3>(transform.data, 4, 0, 0);
/// rot_view.set(rotation);
///
/// // Write translation (top-right 3×1)
/// for (std::size_t i = 0; i < 3; ++i) {
///     transform(i, 3) = translation[i];
/// }
/// @endcode
///
/// **Example 2: Block Matrix Multiplication**
/// @code{.cpp}
/// // Compute C = A * B using block decomposition
/// Mat<float, 4, 4> A, B, C;
/// // ... initialize A, B ...
///
/// // Process 2×2 blocks
/// for (std::size_t i = 0; i < 2; ++i) {
///     for (std::size_t j = 0; j < 2; ++j) {
///         auto c_block = MatrixView<float, 2, 2>(C.data, 4, i*2, j*2);
///         Mat<float, 2, 2> sum = Mat<float, 2, 2>::zero();
///
///         for (std::size_t k = 0; k < 2; ++k) {
///             auto a_block = ConstMatrixView<float, 2, 2>(A.data, 4, i*2, k*2);
///             auto b_block = ConstMatrixView<float, 2, 2>(B.data, 4, k*2, j*2);
///             sum = sum + a_block.to_matrix() * b_block.to_matrix();
///         }
///
///         c_block.set(sum);
///     }
/// }
/// @endcode
///
/// **Example 3: In-Place Submatrix Modification**
/// @code{.cpp}
/// Mat<float, 8, 8> large_matrix = Mat<float, 8, 8>::identity();
///
/// // Zero out bottom-right 4×4 block
/// auto corner = MatrixView<float, 4, 4>(large_matrix.data, 8, 4, 4);
/// corner.fill(0.0f);
///
/// // Extract and process middle 2×2 block
/// auto middle = ConstMatrixView<float, 2, 2>(large_matrix.data, 8, 3, 3);
/// Mat<float, 2, 2> extracted = middle.to_matrix();
/// float trace = extracted(0, 0) + extracted(1, 1);
/// @endcode
///
/// **Example 4: Kalman Filter Covariance Block Updates**
/// @code{.cpp}
/// Mat<float, 6, 6> P = Mat<float, 6, 6>::identity();  // Full state covariance
///
/// // Update only position covariance (top-left 3×3)
/// auto pos_cov = MatrixView<float, 3, 3>(P.data, 6, 0, 0);
/// Mat<float, 3, 3> measurement_update = /* ... */;
/// pos_cov.set(measurement_update);
///
/// // Read velocity covariance (bottom-right 3×3) without copying full matrix
/// auto vel_cov = ConstMatrixView<float, 3, 3>(P.data, 6, 3, 3);
/// float vel_variance_x = vel_cov(0, 0);
/// @endcode
///
/// @section performance_notes Performance Notes
///
/// - **Zero-copy operations**: Views reference existing data, no allocation
/// - **SIMD optimization**: `set()`, `to_matrix()`, and `fill()` use SIMD for float types
/// - **Cache-friendly**: Block operations via views can improve cache locality
/// - **Overhead**: View construction is ~3-4 instructions (pointer + stride arithmetic)
/// - **Pass by value**: Views are 24-32 bytes, passing by value avoids indirection
///
/// @section common_pitfalls Common Pitfalls
///
/// **❌ Storing views in class members:**
/// @code{.cpp}
/// class BadExample {
///     MatrixView<float, 2, 2> m_view;  // DANGER: lifetime tracking required
/// };
/// @endcode
///
/// **✅ Using views in local scope:**
/// @code{.cpp}
/// void goodExample(Mat<float, 4, 4>& matrix) {
///     auto view = MatrixView<float, 2, 2>(matrix.data, 4, 0, 0);  // Safe: matrix outlives function
///     view.fill(1.0f);
/// }
/// @endcode
///
/// **❌ Returning views from functions:**
/// @code{.cpp}
/// auto bad() {
///     Mat<float, 4, 4> temp;
///     return MatrixView<float, 2, 2>(temp.data, 4, 0, 0);  // DANGLING!
/// }
/// @endcode
///
/// **✅ Returning extracted data:**
/// @code{.cpp}
/// auto good(const Mat<float, 4, 4>& matrix) {
///     auto view = ConstMatrixView<float, 2, 2>(matrix.data, 4, 0, 0);
///     return view.to_matrix();  // Safe: returns owned data
/// }
/// @endcode
///
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "compiler_features.hpp"
#include <type_traits>
#include "simd_helpers.hpp"
#include "matrix.hpp"

namespace microla
{

/// \class MatrixView<T, R, C>
/// \brief Non-owning view into a matrix or submatrix.
/// \tparam T The element type.
/// \tparam R Number of rows in the view.
/// \tparam C Number of columns in the view.
///
/// A MatrixView provides a window into existing matrix data without copying.
/// Useful for block algorithms and submatrix operations.
///
/// **Example:**
/// \code{.cpp}
/// Mat<float, 4, 4> M = Mat<float, 4, 4>::identity();
/// auto topLeft = MatrixView<float, 2, 2>(M.data, 4, 0, 0);  // View 2x2 top-left block
/// topLeft.set(Mat<float, 2, 2>{{1, 2}, {3, 4}});  // Modifies M
/// \endcode
template<typename T, std::size_t R, std::size_t C>
class MatrixView
{
public:
    /// \brief Construct view from existing data.
    /// \param data Pointer to the start of the matrix data.
    /// \param parent_cols Number of columns in parent matrix (for stride calculation).
    /// \param start_row Starting row index in parent matrix.
    /// \param start_col Starting column index in parent matrix.
    MatrixView(T* data, std::size_t parent_cols, std::size_t start_row, std::size_t start_col) noexcept
        : m_data(data), m_stride(parent_cols), m_offset_row(start_row), m_offset_col(start_col)
    {
    }

    /// \brief Access element at (row, col).
    auto operator()(std::size_t row, std::size_t col) noexcept -> T&
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "MatrixView: row out of bounds");
        assert(col < C && "MatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    auto operator()(std::size_t row, std::size_t col) const noexcept -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "MatrixView: row out of bounds");
        assert(col < C && "MatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    /// \brief Copy data from a matrix into this view.
    /// \param source The matrix to copy from.
    void set(const Mat<T, R, C>& source) noexcept
    {
        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < R; ++i)
            {
                /* cppcheck-suppress invalidPointerCast */
                auto* dst_row = m_data + ((m_offset_row + i) * m_stride) + m_offset_col;
                /* cppcheck-suppress invalidPointerCast */
                const auto* src_row = source.data + (i * C);
                microla::simd::copy_n_float(src_row, dst_row, C);
            }
            return;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                (*this)(i, j) = source(i, j);
            }
        }
    }

    /// \brief Extract view data into a new matrix.
    /// \return A new matrix containing a copy of the view data.
    [[nodiscard]] auto to_matrix() const noexcept -> Mat<T, R, C>
    {
        Mat<T, R, C> result;
        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < R; ++i)
            {
                /* cppcheck-suppress invalidPointerCast */
                const auto* src_row = m_data + ((m_offset_row + i) * m_stride) + m_offset_col;
                /* cppcheck-suppress invalidPointerCast */
                auto* dst_row = result.data + (i * C);
                microla::simd::copy_n_float(src_row, dst_row, C);
            }
            return result;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                result(i, j) = (*this)(i, j);
            }
        }
        return result;
    }

    /// \brief Fill view with a scalar value.
    /// \param value The value to fill with.
    void fill(T value) noexcept
    {
        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < R; ++i)
            {
                /* cppcheck-suppress invalidPointerCast */
                auto* dst_row = m_data + ((m_offset_row + i) * m_stride) + m_offset_col;
                /* cppcheck-suppress invalidPointerCast */
                microla::simd::fill_float(dst_row, C, static_cast<float>(value));
            }
            return;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                (*this)(i, j) = value;
            }
        }
    }

    /// \brief Get number of rows in view.
    [[nodiscard]] constexpr auto rows() const noexcept -> std::size_t { return R; }

    /// \brief Get number of columns in view.
    [[nodiscard]] constexpr auto cols() const noexcept -> std::size_t { return C; }

private:
    T* m_data;                 ///< Pointer to parent matrix data
    std::size_t m_stride;      ///< Column stride (parent matrix column count)
    std::size_t m_offset_row;  ///< Row offset in parent matrix
    std::size_t m_offset_col;  ///< Column offset in parent matrix
};

/// \class ConstMatrixView<T, R, C>
/// \brief Const non-owning view into a matrix.
template<typename T, std::size_t R, std::size_t C>
class ConstMatrixView
{
public:
    /// \brief Construct const view from existing data.
    ConstMatrixView(const T* data, std::size_t parent_cols, std::size_t start_row, std::size_t start_col) noexcept
        : m_data(data), m_stride(parent_cols), m_offset_row(start_row), m_offset_col(start_col)
    {
    }

    /// \brief Access element at (row, col) [const].
    auto operator()(std::size_t row, std::size_t col) const noexcept -> const T&
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "ConstMatrixView: row out of bounds");
        assert(col < C && "ConstMatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    /// \brief Extract view data into a new matrix.
    [[nodiscard]] auto to_matrix() const noexcept -> Mat<T, R, C>
    {
        Mat<T, R, C> result;
        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < R; ++i)
            {
                const auto* src_row =
                    reinterpret_cast<const float*>(m_data + ((m_offset_row + i) * m_stride) + m_offset_col);
                auto* dst_row = reinterpret_cast<float*>(result.data + (i * C));
                microla::simd::copy_n_float(src_row, dst_row, C);
            }
            return result;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                result(i, j) = (*this)(i, j);
            }
        }
        return result;
    }

    /// \brief Get number of rows in view.
    [[nodiscard]] constexpr auto rows() const noexcept -> std::size_t { return R; }

    /// \brief Get number of columns in view.
    [[nodiscard]] constexpr auto cols() const noexcept -> std::size_t { return C; }

private:
    const T* m_data;           ///< Pointer to parent matrix data
    std::size_t m_stride;      ///< Column stride (parent matrix column count)
    std::size_t m_offset_row;  ///< Row offset in parent matrix
    std::size_t m_offset_col;  ///< Column offset in parent matrix
};

/// Helper functions for Mat to create views
template<typename T, std::size_t R, std::size_t C>
class Mat;  // Forward declaration

}  // namespace microla

// Add to Mat<T,R,C> class:
//
// /// \brief Create a view of a submatrix block.
// /// \tparam ViewR Number of rows in the view.
// /// \tparam ViewC Number of columns in the view.
// /// \param start_row Starting row index.
// /// \param start_col Starting column index.
// /// \return MatrixView into this matrix.
// template<std::size_t ViewR, std::size_t ViewC>
// MatrixView<T, ViewR, ViewC> block(std::size_t start_row, std::size_t start_col)
// {
//     return MatrixView<T, ViewR, ViewC>(data, C, start_row, start_col);
// }
//
// template<std::size_t ViewR, std::size_t ViewC>
// ConstMatrixView<T, ViewR, ViewC> block(std::size_t start_row, std::size_t start_col) const
// {
//     return ConstMatrixView<T, ViewR, ViewC>(data, C, start_row, start_col);
// }
