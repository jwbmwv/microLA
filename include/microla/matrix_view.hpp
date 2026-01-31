// SPDX-License-Identifier: MIT
/// @file matrix_view.hpp
/// @brief Non-owning matrix views for submatrix operations
/// @details Provides lightweight matrix views without data copying, enabling
///          efficient block operations and submatrix manipulation.
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef _MICROLA_MICROLA_VIEW_HPP_
#define _MICROLA_MICROLA_VIEW_HPP_

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
template<typename T, std::uint32_t R, std::uint32_t C>
class MatrixView
{
public:
    /// \brief Construct view from existing data.
    /// \param data Pointer to the start of the matrix data.
    /// \param parent_cols Number of columns in parent matrix (for stride calculation).
    /// \param start_row Starting row index in parent matrix.
    /// \param start_col Starting column index in parent matrix.
    MatrixView(T* data, std::uint32_t parent_cols, std::uint32_t start_row, std::uint32_t start_col)
        : m_data(data), m_stride(parent_cols), m_offset_row(start_row), m_offset_col(start_col)
    {
    }

    /// \brief Access element at (row, col).
    T& operator()(std::uint32_t row, std::uint32_t col)
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "MatrixView: row out of bounds");
        assert(col < C && "MatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    const T& operator()(std::uint32_t row, std::uint32_t col) const
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "MatrixView: row out of bounds");
        assert(col < C && "MatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    /// \brief Copy data from a matrix into this view.
    /// \param source The matrix to copy from.
    void set(const Mat<T, R, C>& source)
    {
        for (std::uint32_t i = 0; i < R; ++i)
        {
            for (std::uint32_t j = 0; j < C; ++j)
            {
                (*this)(i, j) = source(i, j);
            }
        }
    }

    /// \brief Extract view data into a new matrix.
    /// \return A new matrix containing a copy of the view data.
    Mat<T, R, C> toMatrix() const
    {
        Mat<T, R, C> result;
        for (std::uint32_t i = 0; i < R; ++i)
        {
            for (std::uint32_t j = 0; j < C; ++j)
            {
                result(i, j) = (*this)(i, j);
            }
        }
        return result;
    }

    /// \brief Fill view with a scalar value.
    /// \param value The value to fill with.
    void fill(T value)
    {
        for (std::uint32_t i = 0; i < R; ++i)
        {
            for (std::uint32_t j = 0; j < C; ++j)
            {
                (*this)(i, j) = value;
            }
        }
    }

    /// \brief Get number of rows in view.
    constexpr std::uint32_t rows() const { return R; }

    /// \brief Get number of columns in view.
    constexpr std::uint32_t cols() const { return C; }

private:
    T* m_data;                   ///< Pointer to parent matrix data
    std::uint32_t m_stride;      ///< Column stride (parent matrix column count)
    std::uint32_t m_offset_row;  ///< Row offset in parent matrix
    std::uint32_t m_offset_col;  ///< Column offset in parent matrix
};

/// \class ConstMatrixView<T, R, C>
/// \brief Const non-owning view into a matrix.
template<typename T, std::uint32_t R, std::uint32_t C>
class ConstMatrixView
{
public:
    /// \brief Construct const view from existing data.
    ConstMatrixView(const T* data, std::uint32_t parent_cols, std::uint32_t start_row, std::uint32_t start_col)
        : m_data(data), m_stride(parent_cols), m_offset_row(start_row), m_offset_col(start_col)
    {
    }

    /// \brief Access element at (row, col) [const].
    const T& operator()(std::uint32_t row, std::uint32_t col) const
    {
#ifdef MICROLA_DEBUG
        assert(row < R && "ConstMatrixView: row out of bounds");
        assert(col < C && "ConstMatrixView: col out of bounds");
#endif
        return m_data[((m_offset_row + row) * m_stride) + (m_offset_col + col)];
    }

    /// \brief Extract view data into a new matrix.
    Mat<T, R, C> toMatrix() const
    {
        Mat<T, R, C> result;
        for (std::uint32_t i = 0; i < R; ++i)
        {
            for (std::uint32_t j = 0; j < C; ++j)
            {
                result(i, j) = (*this)(i, j);
            }
        }
        return result;
    }

    /// \brief Get number of rows in view.
    constexpr std::uint32_t rows() const { return R; }

    /// \brief Get number of columns in view.
    constexpr std::uint32_t cols() const { return C; }

private:
    const T* m_data;             ///< Pointer to parent matrix data
    std::uint32_t m_stride;      ///< Column stride (parent matrix column count)
    std::uint32_t m_offset_row;  ///< Row offset in parent matrix
    std::uint32_t m_offset_col;  ///< Column offset in parent matrix
};

/// Helper functions for Mat to create views
template<typename T, std::uint32_t R, std::uint32_t C>
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
// template<std::uint32_t ViewR, std::uint32_t ViewC>
// MatrixView<T, ViewR, ViewC> block(std::uint32_t start_row, std::uint32_t start_col)
// {
//     return MatrixView<T, ViewR, ViewC>(data, C, start_row, start_col);
// }
//
// template<std::uint32_t ViewR, std::uint32_t ViewC>
// ConstMatrixView<T, ViewR, ViewC> block(std::uint32_t start_row, std::uint32_t start_col) const
// {
//     return ConstMatrixView<T, ViewR, ViewC>(data, C, start_row, start_col);
// }

#endif  // _MICROLA_MICROLA_VIEW_HPP_
