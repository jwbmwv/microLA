// SPDX-License-Identifier: Apache-2.0
/// @file matrix.hpp
/// @brief Generic matrix class Mat<T,R,C> with full operator support
/// @details This header provides the template-based matrix class with comprehensive
///          operator support, SIMD optimizations (CMSIS-DSP, NEON, MVE), and specialized
///          functionality for embedded systems and real-time applications.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "constants.hpp"          // Ensure constants are included for mathematical operations
#include "compiler_features.hpp"  // Ensure compiler-specific optimizations are included
#include "vector.hpp"

#include <cstdint>
#include <cmath>
#include <initializer_list>
#include <cassert>
#include <stdexcept>
#include <tuple>
#if MICROLA_HAS_DYNAMIC_ALLOC
#include <vector>
#endif
#include <optional>
#include <variant>
#include <type_traits>
#include "simd_helpers.hpp"
#include <algorithm>

// SIMD support
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

// Forward declarations
template<typename T, std::size_t N>
class Vec;

}  // namespace microla

namespace microla
{

// Forward declarations
template<typename T, std::size_t R, std::size_t C>
class Mat;

/// \brief Type traits for Mat class
/// \tparam T Scalar type
/// \tparam R Number of rows
/// \tparam C Number of columns
template<typename T, std::size_t R, std::size_t C>
struct mat_traits  // NOLINT(readability-identifier-naming)
{
    using scalar_type = T;
    using matrix_type = Mat<T, R, C>;
    static constexpr std::size_t rows = R;
    static constexpr std::size_t cols = C;
    static constexpr std::size_t size = R * C;
    static constexpr std::size_t byte_size = sizeof(T) * R * C;
    static constexpr bool is_square = (R == C);
    static constexpr bool is_floating_point = std::is_floating_point_v<T>;
    static constexpr bool is_integral = std::is_integral_v<T>;
};

// ============================================================================
// Identity Matrix Helper
// ============================================================================

/// @brief Helper struct for creating identity matrices (forward declaration)
template<typename T, std::size_t R, std::size_t C, typename Enable>
struct IdentityHelper;

/// @brief Generic matrix class template with fixed dimensions at compile time
/// @tparam T The element type (e.g., float, double, int)
/// @tparam R The number of rows
/// @tparam C The number of columns
/// @details This class provides a comprehensive matrix implementation with support for
///          arithmetic operations, transformations, decompositions, and specialized methods
///          for embedded systems and real-time applications. All dimensions are fixed at
///          compile time for optimal performance and memory layout. Includes SIMD optimizations
///          for ARM NEON/MVE and CMSIS-DSP when enabled.
template<typename T = float, std::size_t R = 4, std::size_t C = 4>
class Mat
{
public:
    enum class InverseError
    {
        Singular,
        IllConditioned
    };

    enum class EigenError
    {
        InvalidOutputBuffer,
        NotConverged
    };

    /// @brief Raw data storage for matrix elements in row-major order
    /// @details Aligned to 16 bytes for SIMD performance
    alignas(16) T data[R * C] = {};

    /// @brief Number of rows (compile-time)
    static constexpr auto rows() noexcept -> std::size_t { return R; }

    /// @brief Number of columns (compile-time)
    static constexpr auto cols() noexcept -> std::size_t { return C; }

    /// @brief Pointer to underlying storage.
    /// @return Pointer to first element in row-major order.
    [[nodiscard]] auto data_ptr() noexcept -> T* { return data; }

    /// @brief Pointer to underlying storage (const).
    [[nodiscard]] auto data_ptr() const noexcept -> const T* { return data; }

    /// @brief Default constructor - constexpr-trivial to allow compile-time use
    /// @details The `data` member is default-initialized at declaration, so the
    ///          default constructor is safe to be a constexpr trivial default.
    constexpr Mat() noexcept = default;

    /// @brief Copy constructor (default for fixed-size arrays).
    constexpr Mat(const Mat& other) noexcept = default;

    /// @brief Move constructor (default for fixed-size arrays).
    constexpr Mat(Mat&& other) noexcept = default;

    /// @brief Copy assignment operator.
    constexpr auto operator=(const Mat& other) noexcept -> Mat& = default;

    /// @brief Move assignment operator (default for fixed-size arrays).
    constexpr auto operator=(Mat&& other) noexcept -> Mat& = default;

    /// @brief Destructor (defaulted to satisfy rule-of-five checkers).
    ~Mat() noexcept = default;

    /// @brief Variadic constexpr constructor to allow direct compile-time initialization
    ///        with a list of R*C values (enables Mat<T,R,C>{v0, v1, ...} in constexpr).
    template<typename... U>
        requires(sizeof...(U) == (R * C))
    constexpr Mat(U... vals) noexcept : data{static_cast<T>(vals)...}
    {
    }

    /// @brief Constructs a matrix from an initializer list
    /// @param init Initializer list containing matrix elements in row-major order
    /// @throws Assertion failure if init.size() != R * C
    /// @details Elements should be provided in row-major order. Example:
    ///          Mat<float, 2, 2> m = {1.0f, 2.0f, 3.0f, 4.0f};
    constexpr Mat(std::initializer_list<T> init)
    {
        assert(init.size() == R * C);
        std::copy(init.begin(), init.end(), data);
    }

    /// @brief Constructs a matrix from nested initializer lists (rows)
    /// @param rows Nested initializer list where each inner list is a row
    /// @throws Assertion failure if rows.size() != R or any row.size() != C
    constexpr Mat(std::initializer_list<std::initializer_list<T>> rows)
    {
        assert(rows.size() == R);
        std::size_t i = 0;
        for (const auto& row : rows)
        {
            assert(row.size() == C);
            std::size_t j = 0;
            for (const T& val : row)
            {
                data[i * C + j] = val;
                ++j;
            }
            ++i;
        }
    }

    /// @brief Creates an identity matrix
    /// @return Identity matrix with 1s on the diagonal and 0s elsewhere
    /// @details Only available for square matrices (R == C). This is a constexpr function
    ///          allowing compile-time generation of identity matrices for all sizes.
    /// @note Static assertion will fail if called on non-square matrix types
    static constexpr auto identity() -> Mat { return IdentityHelper<T, R, C, void>::create(); }

    /// @brief Creates a zero matrix
    /// @return Matrix with all elements set to zero
    /// @details Creates a matrix where all elements are initialized to T(0).
    ///          This is a constexpr function for compile-time initialization.
    static constexpr auto zero() -> Mat
    {
        Mat result;
        if constexpr (std::is_same_v<T, float>)
        {
            microla::simd::fill_float(result.data, R * C, static_cast<float>(0));
            return result;
        }
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = T(0);
        }
        return result;
    }

    /// @brief Creates a matrix with all elements set to one
    /// @return Matrix with all elements set to T(1)
    /// @details Creates a matrix where all elements are initialized to T(1).
    ///          This is a constexpr function for compile-time initialization.
    static constexpr auto ones() -> Mat
    {
        Mat result;
        if constexpr (std::is_same_v<T, float>)
        {
            microla::simd::fill_float(result.data, R * C, static_cast<float>(1));
            return result;
        }
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = T(1);
        }
        return result;
    }

    /// @brief Creates a diagonal matrix from a vector
    /// @param diag The diagonal elements as a Vec<T, N> where N = min(R, C)
    /// @return Matrix with diagonal elements from the vector, zeros elsewhere
    /// @details Only available for square matrices (R == C).
    template<std::size_t N>
    static auto diagonal(const Vec<T, N>& diag) -> Mat
    {
        static_assert(R == C, "diagonal() is only available for square matrices");
        static_assert(N == R, "Vector size must match matrix dimension");
        Mat result = zero();
        for (std::size_t i = 0; i < N; ++i)
        {
            result(i, i) = diag[i];
        }
        return result;
    }

    /// @brief Subscript operator for row access (non-const)
    /// @param row The row index (0-based)
    /// @return Pointer to the first element of the specified row
    /// @details Allows accessing matrix elements using m[i][j] syntax
    [[nodiscard]] auto operator[](std::size_t row) noexcept -> T* { return &data[row * C]; }

    /// @brief Subscript operator for row access (const)
    /// @param row The row index (0-based)
    /// @return Const pointer to the first element of the specified row
    /// @details Allows accessing matrix elements using m[i][j] syntax on const matrices
    [[nodiscard]] auto operator[](std::size_t row) const noexcept -> const T* { return &data[row * C]; }

    /// @brief Function call operator for element access (non-const)
    /// @param row The row index (0-based)
    /// @param col The column index (0-based)
    /// @return Reference to the element at (row, col)
    /// @details Allows accessing matrix elements using m(i, j) syntax
    [[nodiscard]] constexpr auto operator()(std::size_t row, std::size_t col) noexcept -> T&
    {
        return data[row * C + col];
    }

    /// @brief Function call operator for element access (const)
    /// @param row The row index (0-based)
    /// @param col The column index (0-based)
    /// @return Const reference to the element at (row, col)
    /// @details Allows accessing matrix elements using m(i, j) syntax on const matrices
    [[nodiscard]] constexpr auto operator()(std::size_t row, std::size_t col) const noexcept -> const T&
    {
        return data[row * C + col];
    }

    /// @brief Extract a submatrix block
    /// @tparam BR Number of rows in the block
    /// @tparam BC Number of columns in the block
    /// @param start_row Starting row index (0-based)
    /// @param start_col Starting column index (0-based)
    /// @return New matrix containing the extracted block
    /// @details Extracts a BR×BC submatrix starting at (start_row, start_col).
    ///          This creates a copy of the block data.
    /// @note Compile-time assertion ensures the block fits within matrix bounds
    template<std::size_t BR, std::size_t BC>
    [[nodiscard]] auto block(std::size_t start_row, std::size_t start_col) const -> Mat<T, BR, BC>
    {
        assert(start_row + BR <= R && "Block exceeds matrix row bounds");
        assert(start_col + BC <= C && "Block exceeds matrix column bounds");

        Mat<T, BR, BC> result;
        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < BR; ++i)
            {
                const auto* src_row = data + ((start_row + i) * C + start_col);
                auto* dst_row = result.data + (i * BC);
                microla::simd::copy_n_float(src_row, dst_row, BC);
            }
            return result;
        }

        for (std::size_t i = 0; i < BR; ++i)
        {
            for (std::size_t j = 0; j < BC; ++j)
            {
                result(i, j) = (*this)(start_row + i, start_col + j);
            }
        }
        return result;
    }

    /// @brief Set a submatrix block
    /// @tparam BR Number of rows in the block
    /// @tparam BC Number of columns in the block
    /// @param start_row Starting row index (0-based)
    /// @param start_col Starting column index (0-based)
    /// @param block_data Matrix containing data to insert
    /// @details Copies BR×BC elements from block_data into this matrix starting at (start_row, start_col)
    /// @note Compile-time assertion ensures the block fits within matrix bounds
    template<std::size_t BR, std::size_t BC>
    void set_block(std::size_t start_row, std::size_t start_col, const Mat<T, BR, BC>& block_data)
    {
        assert(start_row + BR <= R && "Block exceeds matrix row bounds");
        assert(start_col + BC <= C && "Block exceeds matrix column bounds");

        if constexpr (std::is_same_v<T, float>)
        {
            for (std::size_t i = 0; i < BR; ++i)
            {
                const auto* src_row = block_data.data + (i * BC);
                auto* dst_row = data + ((start_row + i) * C + start_col);
                microla::simd::copy_n_float(src_row, dst_row, BC);
            }
            return;
        }

        for (std::size_t i = 0; i < BR; ++i)
        {
            for (std::size_t j = 0; j < BC; ++j)
            {
                (*this)(start_row + i, start_col + j) = block_data(i, j);
            }
        }
    }

    /// @brief Get a single row as a vector
    /// @param row_idx Row index (0-based)
    /// @return Row vector containing the row elements
    /// @details Extracts row_idx as a 1×C row vector (or Vec<T,C> if appropriate)
    [[nodiscard]] auto row(std::size_t row_idx) const -> Mat<T, 1, C>
    {
        assert(row_idx < R && "Row index out of bounds");
        Mat<T, 1, C> result;
        if constexpr (std::is_same_v<T, float>)
        {
            const auto* src_row = data + (row_idx * C);
            auto* dst_row = result.data;
            microla::simd::copy_n_float(src_row, dst_row, C);
            return result;
        }

        for (std::size_t j = 0; j < C; ++j)
        {
            result(0, j) = (*this)(row_idx, j);
        }
        return result;
    }

    /// @brief Get a single column as a vector
    /// @param col_idx Column index (0-based)
    /// @return Column vector containing the column elements
    /// @details Extracts col_idx as an R×1 column vector
    [[nodiscard]] auto col(std::size_t col_idx) const -> Mat<T, R, 1>
    {
        assert(col_idx < C && "Column index out of bounds");
        Mat<T, R, 1> result;
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            const auto* src = data;
            /* cppcheck-suppress invalidPointerCast */
            auto* dst = result.data;
            microla::simd::gather_strided_float(src, col_idx, C, dst, R);
            return result;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            result(i, 0) = (*this)(i, col_idx);
        }
        return result;
    }

    /// @brief Set a single row from a vector
    /// @param row_idx Row index (0-based)
    /// @param row_data Row vector containing data to insert
    /// @details Copies C elements from row_data into row row_idx
    void set_row(std::size_t row_idx, const Mat<T, 1, C>& row_data)
    {
        assert(row_idx < R && "Row index out of bounds");
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            const auto* src = row_data.data;
            /* cppcheck-suppress invalidPointerCast */
            auto* dst = data + row_idx * C;
            microla::simd::copy_n_float(src, dst, C);
            return;
        }

        for (std::size_t j = 0; j < C; ++j)
        {
            (*this)(row_idx, j) = row_data(0, j);
        }
    }

    /// @brief Set a single column from a vector
    /// @param col_idx Column index (0-based)
    /// @param col_data Column vector containing data to insert
    /// @details Copies R elements from col_data into column col_idx
    void set_col(std::size_t col_idx, const Mat<T, R, 1>& col_data)
    {
        assert(col_idx < C && "Column index out of bounds");
        if constexpr (std::is_same_v<T, float>)
        {
            const auto* src = col_data.data;
            auto* dst = data;
            microla::simd::scatter_strided_float(src, dst, col_idx, C, R);
            return;
        }

        for (std::size_t i = 0; i < R; ++i)
        {
            (*this)(i, col_idx) = col_data(i, 0);
        }
    }

    /// @brief Computes the determinant of the matrix
    /// @return The determinant value
    /// @details Implements analytical formulas for 2x2 and 3x3 matrices for performance.
    ///          For larger matrices, uses LU decomposition.
    ///          Only available for square matrices (R == C).
    [[nodiscard]] auto determinant() const -> T
    {
        static_assert(R == C, "Determinant is only defined for square matrices.");

        if constexpr (R == 1)
        {
            return data[0];
        }
        else if constexpr (R == 2)
        {
            return data[0] * data[3] - data[1] * data[2];
        }
        else if constexpr (R == 3)
        {
            return data[0] * (data[4] * data[8] - data[5] * data[7]) -
                   data[1] * (data[3] * data[8] - data[5] * data[6]) +
                   data[2] * (data[3] * data[7] - data[4] * data[6]);
        }
        else
        {
            // Generic determinant using LU decomposition
            // det(A) = det(P) * det(L) * det(U) = det(P) * det(U)
            // det(L) = 1 (unit lower triangular), det(U) = product of diagonal
            Mat temp = *this;
            T det = T(1);

            for (std::size_t i = 0; i < R; ++i)
            {
                // Find pivot
                std::size_t pivot = i;
                T max_val = std::abs(temp(i, i));
                for (std::size_t k = i + 1; k < R; ++k)
                {
                    T abs_val = std::abs(temp(k, i));
                    if (abs_val > max_val)
                    {
                        max_val = abs_val;
                        pivot = k;
                    }
                }

                if (max_val == T(0))
                {
                    return T(0);  // Singular matrix
                }

                // Swap rows if needed
                if (pivot != i)
                {
                    for (std::size_t j = 0; j < C; ++j)
                    {
                        std::swap(temp(i, j), temp(pivot, j));
                    }
                    det = -det;  // Row swap changes sign
                }

                // Multiply determinant by pivot
                det *= temp(i, i);

                // Eliminate column below pivot
                for (std::size_t k = i + 1; k < R; ++k)
                {
                    T factor = temp(k, i) / temp(i, i);
                    for (std::size_t j = i; j < C; ++j)
                    {
                        temp(k, j) -= factor * temp(i, j);
                    }
                }
            }

            return det;
        }
    }

    /// @brief Computes the inverse of the matrix
    /// @return The inverse matrix
    /// @throws std::runtime_error if the matrix is singular (determinant is zero)
    /// @details Uses analytical inverse formulas for 2x2 and 3x3 matrices for performance.
    ///          For matrices larger than 3x3, uses Gauss-Jordan elimination.
    ///          Only available for square matrices (R == C).
    // Non-throwing inverse: writes result into `out` and returns true on success.
    [[nodiscard]] auto inverse(Mat& out) const noexcept -> bool
    {
        static_assert(R == C, "Inverse is only defined for square matrices.");

        // Fast paths for small matrices
        if constexpr (R == 2)
        {
            T det = data[0] * data[3] - data[1] * data[2];
            if (det == T(0))
            {
                return false;
            }
            out(0, 0) = data[3] / det;
            out(0, 1) = -data[1] / det;
            out(1, 0) = -data[2] / det;
            out(1, 1) = data[0] / det;
            return true;
        }
        else if constexpr (R == 3)
        {
            T det = data[0] * (data[4] * data[8] - data[5] * data[7]) -
                    data[1] * (data[3] * data[8] - data[5] * data[6]) +
                    data[2] * (data[3] * data[7] - data[4] * data[6]);
            if (det == T(0))
            {
                return false;
            }
            // Compute adjugate and divide by determinant
            out(0, 0) = (data[4] * data[8] - data[5] * data[7]) / det;
            out(0, 1) = (data[2] * data[7] - data[1] * data[8]) / det;
            out(0, 2) = (data[1] * data[5] - data[2] * data[4]) / det;
            out(1, 0) = (data[5] * data[6] - data[3] * data[8]) / det;
            out(1, 1) = (data[0] * data[8] - data[2] * data[6]) / det;
            out(1, 2) = (data[2] * data[3] - data[0] * data[5]) / det;
            out(2, 0) = (data[3] * data[7] - data[4] * data[6]) / det;
            out(2, 1) = (data[1] * data[6] - data[0] * data[7]) / det;
            out(2, 2) = (data[0] * data[4] - data[1] * data[3]) / det;
            return true;
        }
        else
        {
            // Generic Gauss-Jordan elimination for larger matrices
            // Create augmented matrix [A|I]
            T augmented[R][2 * C];
            for (std::size_t i = 0; i < R; ++i)
            {
                for (std::size_t j = 0; j < C; ++j)
                {
                    augmented[i][j] = data[i * C + j];
                    augmented[i][j + C] = (i == j) ? T(1) : T(0);
                }
            }

            // Forward elimination with partial pivoting
            for (std::size_t i = 0; i < R; ++i)
            {
                // Find pivot
                std::size_t pivot = i;
                T max_val = std::abs(augmented[i][i]);
                for (std::size_t k = i + 1; k < R; ++k)
                {
                    T abs_val = std::abs(augmented[k][i]);
                    if (abs_val > max_val)
                    {
                        max_val = abs_val;
                        pivot = k;
                    }
                }

                if (max_val == T(0))
                {
                    return false;
                }

                // Swap rows if needed
                if (pivot != i)
                {
                    for (std::size_t j = 0; j < 2 * C; ++j)
                    {
                        std::swap(augmented[i][j], augmented[pivot][j]);
                    }
                }

                // Scale pivot row
                T pivot_val = augmented[i][i];
                for (std::size_t j = 0; j < 2 * C; ++j)
                {
                    augmented[i][j] /= pivot_val;
                }

                // Eliminate column
                for (std::size_t k = 0; k < R; ++k)
                {
                    if (k != i)
                    {
                        T factor = augmented[k][i];
                        for (std::size_t j = 0; j < 2 * C; ++j)
                        {
                            augmented[k][j] -= factor * augmented[i][j];
                        }
                    }
                }
            }

            // Extract result from right half of augmented matrix
            for (std::size_t i = 0; i < R; ++i)
            {
                for (std::size_t j = 0; j < C; ++j)
                {
                    out(i, j) = augmented[i][j + C];
                }
            }
            return true;
        }
    }

    /// @brief Computes the inverse of the matrix
    /// @return The inverse matrix
    /// @throws std::runtime_error if the matrix is singular (determinant is zero) when exceptions are enabled
    [[nodiscard]] auto inverse() const -> Mat
    {
        auto result = try_inverse();
        if (!result.has_value())
        {
#if MICROLA_HAS_EXCEPTIONS
            throw std::runtime_error("Matrix is singular and cannot be inverted.");
#else
            return Mat::zero();
#endif
        }
        return *result;
    }

    /// @brief Computes the inverse in a non-throwing way
    /// @return Inverse matrix on success, std::nullopt if singular
    [[nodiscard]] auto try_inverse() const noexcept -> std::optional<Mat>
    {
        Mat result;
        if (!inverse(result))
        {
            return std::nullopt;
        }
        return result;
    }

    /// @brief Computes the inverse with condition number checking
    /// @param threshold Maximum acceptable condition number (default: 1e6)
    /// @return Inverse matrix on success, or an InverseError on failure
    /// @details Checks the condition number before attempting inversion to detect
    ///          ill-conditioned matrices. Values close to threshold indicate the
    ///          inverse may be numerically unstable.
    [[nodiscard]] auto inverse_checked(T threshold = T(1e6)) const noexcept -> std::variant<Mat, InverseError>
    {
        static_assert(R == C, "Inverse is only defined for square matrices.");

        // Quick determinant check first
        T det = determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return InverseError::Singular;
        }

        // Compute inverse
        auto inv_mat = try_inverse();
        if (!inv_mat.has_value())
        {
            return InverseError::Singular;
        }

        // Check condition number: κ(A) = ||A|| * ||A^-1||
        T norm_a = norm_inf();
        T norm_a_inv = inv_mat->norm_inf();
        T cond = norm_a * norm_a_inv;

        if (cond > threshold)
        {
            return InverseError::IllConditioned;
        }

        return *inv_mat;
    }

    /// @brief Computes the Moore-Penrose pseudoinverse
    /// @param tolerance Tolerance for considering singular values as zero (default: 1e-9)
    /// @return The pseudoinverse A+ such that A * A+ * A = A and A+ * A * A+ = A+
    /// @details The pseudoinverse is computed using SVD: A = U * S * V^T, then A+ = V * S+ * U^T
    ///          where S+ is computed by taking the reciprocal of non-zero singular values.
    ///          Works for both square and non-square matrices, and handles rank-deficient matrices.
    ///          For full-rank square matrices, this equals the regular inverse.
    /// @note This is useful for solving least-squares problems and working with non-square matrices.
    [[nodiscard]] auto pseudoinverse(T tolerance = 1e-9) const -> Mat<T, C, R>
    {
        // Compute SVD: A = U * S * V^T
        auto [U, S, V] = svd();

        // Compute S+ (pseudoinverse of S, which is R×C)
        Mat<T, C, R> s_plus;  // Note: dimensions are transposed
        for (std::size_t i = 0; i < std::min(R, C); ++i)
        {
            T sigma = S(i, i);
            if (std::abs(sigma) > tolerance)
            {
                s_plus(i, i) = static_cast<T>(1) / sigma;
            }
            else
            {
                s_plus(i, i) = static_cast<T>(0);
            }
        }

        // Compute A+ = V * S+ * U^T
        // First compute S+ * U^T
        Mat<T, C, R> temp;
        for (std::size_t i = 0; i < C; ++i)
        {
            for (std::size_t j = 0; j < R; ++j)
            {
                T sum = static_cast<T>(0);
                for (std::size_t k = 0; k < std::min(C, R); ++k)
                {
                    sum += s_plus(i, k) * U(j, k);  // U^T means we access U(j, k) not U(k, j)
                }
                temp(i, j) = sum;
            }
        }

        // Now compute V * temp
        Mat<T, C, R> result;
        for (std::size_t i = 0; i < C; ++i)
        {
            for (std::size_t j = 0; j < R; ++j)
            {
                T sum = static_cast<T>(0);
                for (std::size_t k = 0; k < C; ++k)
                {
                    sum += V(i, k) * temp(k, j);
                }
                result(i, j) = sum;
            }
        }

        return result;
    }

    /// @brief Equality comparison operator
    /// @param other The matrix to compare with
    /// @return true if all elements are equal, false otherwise
    /// @details Performs element-wise comparison of all matrix elements
    [[nodiscard]] constexpr auto operator==(const Mat& other) const noexcept -> bool
    {
        if (this == &other)
        {
            return true;
        }
        if constexpr (std::is_floating_point_v<T>)
        {
            // Use a tolerant element-wise comparison for floating-point matrices
            const T rel_tol = T(100) * std::numeric_limits<T>::epsilon();
            const T abs_tol = std::numeric_limits<T>::epsilon();
            for (std::size_t i = 0; i < R * C; ++i)
            {
                T a = data[i];
                T b = other.data[i];
                T diff = std::abs(a - b);
                if (diff <= abs_tol)
                {
                    continue;
                }
                T max_abs = std::max(std::abs(a), std::abs(b));
                if (diff > rel_tol * max_abs)
                {
                    return false;
                }
            }
            return true;
        }
        else
        {
            for (std::size_t i = 0; i < R * C; ++i)
            {
                if (data[i] != other.data[i])
                {
                    return false;
                }
            }
            return true;
        }
    }

    /// @brief Matrix inequality operator
    /// @param other The matrix to compare
    /// @return True if matrices are not equal, false otherwise
    /// @details Performs element-wise inequality comparison
    [[nodiscard]] constexpr auto operator!=(const Mat& other) const noexcept -> bool { return !(*this == other); }

    /// @brief Check whether a square matrix is approximately identity.
    /// @param epsilon Tolerance for floating-point comparisons.
    /// @return True if diagonal entries are approximately one and off-diagonal entries are approximately zero.
    template<std::size_t RR = R, std::size_t CC = C>
        requires(RR == CC)
    [[nodiscard]] auto is_identity(T epsilon = std::numeric_limits<T>::epsilon() *
                                               static_cast<T>(8)) const noexcept -> bool
    {
        for (std::size_t row = 0; row < R; ++row)
        {
            for (std::size_t col = 0; col < C; ++col)
            {
                const T expected = row == col ? T(1) : T(0);
                if constexpr (std::is_floating_point_v<T>)
                {
                    if (std::abs((*this)(row, col) - expected) > epsilon)
                    {
                        return false;
                    }
                }
                else if ((*this)(row, col) != expected)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// @brief Matrix addition operator
    /// @param other The matrix to add
    /// @return A new matrix containing the element-wise sum
    /// @details Performs element-wise addition: result[i][j] = this[i][j] + other[i][j]
    [[nodiscard]] constexpr auto operator+(const Mat& other) const noexcept -> Mat
    {
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Mat result;
            arm_matrix_instance_f32 src1, src2, dst;
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src1, R, C, const_cast<float*>(data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src2, R, C, const_cast<float*>(other.data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&dst, R, C, result.data);
            arm_mat_add_f32(&src1, &src2, &dst);
            return result;
        }
#endif
        Mat result;
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = data[i] + other.data[i];
        }
        return result;
    }

    /// @brief Matrix subtraction operator
    /// @param other The matrix to subtract
    /// @return A new matrix containing the element-wise difference
    /// @details Performs element-wise subtraction: result[i][j] = this[i][j] - other[i][j]
    [[nodiscard]] constexpr auto operator-(const Mat& other) const noexcept -> Mat
    {
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Mat result;
            arm_matrix_instance_f32 src1, src2, dst;
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src1, R, C, const_cast<float*>(data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src2, R, C, const_cast<float*>(other.data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&dst, R, C, result.data);
            arm_mat_sub_f32(&src1, &src2, &dst);
            return result;
        }
#endif
        Mat result;
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = data[i] - other.data[i];
        }
        return result;
    }

    /// @brief Matrix-matrix multiplication operator
    /// @tparam OtherC Number of columns in the other matrix
    /// @param other The matrix to multiply with (must have C rows)
    /// @return A new matrix of dimensions R x OtherC
    /// @details Performs standard matrix multiplication. The number of columns in this
    ///          matrix must equal the number of rows in the other matrix.
    ///          Includes SIMD optimizations for float matrices when available.
    template<std::size_t OtherC>
    [[nodiscard]] auto operator*(const Mat<T, C, OtherC>& other) const -> Mat<T, R, OtherC>
    {
        Mat<T, R, OtherC> result;

#ifdef CONFIG_MICROLA_RISCV
        // SIMD optimization for 4x4 float matrices using RISC-V V extension
        if constexpr (std::is_same_v<T, float> && (R == 4 && C == 4 && OtherC == 4))
        {
            /* cppcheck-suppress invalidPointerCast */
            const float* a = data;
            /* cppcheck-suppress invalidPointerCast */
            const float* b = other.data;
            /* cppcheck-suppress invalidPointerCast */
            float* r = result.data;
            size_t vl = vsetvl_e32m1(4);

            for (std::size_t i = 0; i < 4; ++i)
            {
                vfloat32m1_t a_row = vle32_v_f32m1(&a[i * 4], vl);
                for (std::size_t j = 0; j < 4; ++j)
                {
                    float b_col[4] = {b[j], b[4 + j], b[8 + j], b[12 + j]};
                    vfloat32m1_t b_col_v = vle32_v_f32m1(b_col, vl);
                    vfloat32m1_t prod = vfmul_vv_f32m1(a_row, b_col_v, vl);
                    vfloat32m1_t sum = vfredsum_vs_f32m1_f32m1(vundefined_f32m1(), prod,
                                                               vfmv_s_f_f32m1(vundefined_f32m1(), 0.0f, vl), vl);
                    r[i * 4 + j] = vfmv_f_s_f32m1_f32(sum);
                }
            }
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        // SIMD optimization for 4x4 float matrices (common in graphics)
        if constexpr (std::is_same_v<T, float> && (R == 4 && C == 4 && OtherC == 4))
        {
            /* cppcheck-suppress invalidPointerCast */
            const float* a = data;
            /* cppcheck-suppress invalidPointerCast */
            const float* b = other.data;
            /* cppcheck-suppress invalidPointerCast */
            float* r = result.data;

            // Process each row
            for (std::size_t i = 0; i < 4; ++i)
            {
                // Load row from A
                float32x4_t a_row = vld1q_f32(&a[i * 4]);

                // Compute dot products with columns of B
                for (std::size_t j = 0; j < 4; ++j)
                {
                    float32x4_t b_col = {b[j], b[4 + j], b[8 + j], b[12 + j]};
                    float32x4_t prod = vmulq_f32(a_row, b_col);

                    // Horizontal sum
                    float32x2_t sum = vadd_f32(vget_low_f32(prod), vget_high_f32(prod));
                    sum = vpadd_f32(sum, sum);
                    r[i * 4 + j] = vget_lane_f32(sum, 0);
                }
            }
            return result;
        }
#endif

#ifdef CONFIG_MICROLA_CMSIS
        // Use CMSIS-DSP for matrix multiplication if available
        if constexpr (std::is_same_v<T, float>)
        {
            arm_matrix_instance_f32 src1, src2, dst;
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src1, R, C, const_cast<float*>(data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&src2, C, OtherC, const_cast<float*>(other.data));
            /* cppcheck-suppress invalidPointerCast */
            arm_mat_init_f32(&dst, R, OtherC, result.data);
            arm_mat_mult_f32(&src1, &src2, &dst);
            return result;
        }
#endif

        // Generic implementation
        // Attempt a simple portable SIMD-friendly fast path for float when
        // operating on contiguous rows/columns using 4-wide kernels. This
        // processes the k dimension in blocks of 4 and accumulates using
        // the small 4-wide FMA helper. Falls back to scalar tail handling.
        if constexpr (std::is_same_v<T, float>)
        {
            /* cppcheck-suppress invalidPointerCast */
            const auto* a = data;
            /* cppcheck-suppress invalidPointerCast */
            const auto* b = other.data;
            /* cppcheck-suppress invalidPointerCast */
            auto* r = result.data;

            for (std::size_t i = 0; i < R; ++i)
            {
                const float* arow = a + i * C;
                for (std::size_t j = 0; j < OtherC; ++j)
                {
                    float acc4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                    std::size_t k = 0;

                    for (; k + 4 <= C; k += 4)
                    {
                        float a4[4];
                        float b4[4];
                        microla::simd::load_padded4(arow + k, 4, a4);
                        // Gather 4 elements from column j of B starting at row k
                        microla::simd::gather_strided_float(b, k * OtherC + j, OtherC, b4, 4);
                        microla::simd::fma4_accumulate(acc4, a4, b4);
                    }

                    // Horizontal sum of acc4
                    float sum = acc4[0] + acc4[1] + acc4[2] + acc4[3];

                    // Tail
                    for (; k < C; ++k)
                    {
                        sum += arow[k] * b[k * OtherC + j];
                    }

                    r[i * OtherC + j] = sum;
                }
            }
        }
        else
        {
            for (std::size_t i = 0; i < R; ++i)
            {
                for (std::size_t j = 0; j < OtherC; ++j)
                {
                    result.data[i * OtherC + j] = T(0);
                    for (std::size_t k = 0; k < C; ++k)
                    {
                        result.data[i * OtherC + j] += data[i * C + k] * other.data[k * OtherC + j];
                    }
                }
            }
        }
        return result;
    }

    /// @brief Matrix-vector multiplication operator
    /// @tparam VecSize Size of the vector (must equal C)
    /// @param vec The vector to multiply with
    /// @return A new vector of size R
    /// @details Performs matrix-vector multiplication. The matrix must have C columns
    ///          matching the vector size. Includes SIMD optimizations when available.
    template<std::size_t VecSize>
    [[nodiscard]] auto operator*(const Vec<T, VecSize>& vec) const -> Vec<T, R>
    {
        static_assert(C == VecSize, "Matrix-Vector multiplication requires compatible dimensions.");
        Vec<T, R> result;

#ifdef CONFIG_MICROLA_RISCV
        // SIMD optimization for 4x4 float matrix * vec4 using RISC-V V extension
        if constexpr (std::is_same_v<T, float> && (R == 4 && C == 4))
        {
            const float* m = data;
            const float* v = vec.data;
            float* r = result.data;
            size_t vl = vsetvl_e32m1(4);
            vfloat32m1_t vec_val = vle32_v_f32m1(v, vl);

            for (std::size_t i = 0; i < 4; ++i)
            {
                vfloat32m1_t row = vle32_v_f32m1(&m[i * 4], vl);
                vfloat32m1_t prod = vfmul_vv_f32m1(row, vec_val, vl);
                vfloat32m1_t sum =
                    vfredsum_vs_f32m1_f32m1(vundefined_f32m1(), prod, vfmv_s_f_f32m1(vundefined_f32m1(), 0.0f, vl), vl);
                r[i] = vfmv_f_s_f32m1_f32(sum);
            }
            return result;
        }
        // SIMD optimization for 3x3 float matrix * vec3 using RISC-V V extension
        else if constexpr (std::is_same_v<T, float> && (R == 3 && C == 3))
        {
            const float* m = data;
            const float* v = vec.data;
            float* r = result.data;
            size_t vl = vsetvl_e32m1(3);
            vfloat32m1_t vec_val = vle32_v_f32m1(v, vl);

            for (std::size_t i = 0; i < 3; ++i)
            {
                vfloat32m1_t row = vle32_v_f32m1(&m[i * 3], vl);
                vfloat32m1_t prod = vfmul_vv_f32m1(row, vec_val, vl);
                vfloat32m1_t sum =
                    vfredsum_vs_f32m1_f32m1(vundefined_f32m1(), prod, vfmv_s_f_f32m1(vundefined_f32m1(), 0.0f, vl), vl);
                r[i] = vfmv_f_s_f32m1_f32(sum);
            }
            return result;
        }
#endif
#ifdef CONFIG_MICROLA_NEON
        // SIMD optimization for 4x4 float matrix * vec4
        if constexpr (std::is_same_v<T, float> && (R == 4 && C == 4))
        {
            const float* m = data;
            const float* v = vec.data;
            float* r = result.data;

            float32x4_t vec_val = vld1q_f32(v);

            for (std::size_t i = 0; i < 4; ++i)
            {
                float32x4_t row = vld1q_f32(&m[i * 4]);
                float32x4_t prod = vmulq_f32(row, vec_val);

                // Horizontal sum
                float32x2_t sum = vadd_f32(vget_low_f32(prod), vget_high_f32(prod));
                sum = vpadd_f32(sum, sum);
                r[i] = vget_lane_f32(sum, 0);
            }
            return result;
        }
        // SIMD optimization for 3x3 float matrix * vec3
        else if constexpr (std::is_same_v<T, float> && (R == 3 && C == 3))
        {
            const float* m = data;
            const float* v = vec.data;
            float* r = result.data;

            // Extend vec3 to vec4 with 0 padding
            float vec_padded[4] = {v[0], v[1], v[2], 0.0f};
            float32x4_t vec_val = vld1q_f32(vec_padded);

            for (std::size_t i = 0; i < 3; ++i)
            {
                float row_padded[4] = {m[i * 3], m[i * 3 + 1], m[i * 3 + 2], 0.0f};
                float32x4_t row = vld1q_f32(row_padded);
                float32x4_t prod = vmulq_f32(row, vec_val);

                // Horizontal sum
                float32x2_t sum = vadd_f32(vget_low_f32(prod), vget_high_f32(prod));
                sum = vpadd_f32(sum, sum);
                r[i] = vget_lane_f32(sum, 0);
            }
            return result;
        }
#endif

#ifdef CONFIG_MICROLA_CMSIS
        // Use CMSIS-DSP for matrix-vector multiplication
        if constexpr (std::is_same_v<T, float>)
        {
            arm_matrix_instance_f32 mat, vec_mat, result_mat;
            arm_mat_init_f32(&mat, R, C, const_cast<float*>(data));
            arm_mat_init_f32(&vec_mat, C, 1, const_cast<float*>(vec.data));
            arm_mat_init_f32(&result_mat, R, 1, result.data);
            arm_mat_mult_f32(&mat, &vec_mat, &result_mat);
            return result;
        }
#endif

        // Generic implementation
        for (std::size_t i = 0; i < R; ++i)
        {
            result[i] = T(0);
            for (std::size_t j = 0; j < C; ++j)
            {
                result[i] += data[i * C + j] * vec[j];
            }
        }
        return result;
    }

    /// @brief Matrix-scalar multiplication operator
    /// @param scalar The scalar value to multiply all elements by
    /// @return A new matrix with all elements multiplied by the scalar
    /// @details Performs element-wise scalar multiplication: result[i][j] = this[i][j] * scalar
    [[nodiscard]] constexpr auto operator*(T scalar) const noexcept -> Mat
    {
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Mat result;
            arm_matrix_instance_f32 src, dst;
            arm_mat_init_f32(&src, R, C, const_cast<float*>(data));
            arm_mat_init_f32(&dst, R, C, result.data);
            arm_mat_scale_f32(&src, static_cast<float>(scalar), &dst);
            return result;
        }
#endif
        Mat result;
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = data[i] * scalar;
        }
        return result;
    }

    /// @brief Compound addition assignment operator
    /// @param other The matrix to add to this matrix
    /// @return Reference to this matrix after addition
    /// @details Performs in-place element-wise addition: this[i][j] += other[i][j]
    constexpr auto operator+=(const Mat& other) noexcept -> Mat&
    {
        for (std::size_t i = 0; i < R * C; ++i)
        {
            data[i] += other.data[i];
        }
        return *this;
    }

    /// @brief Compound subtraction assignment operator
    /// @param other The matrix to subtract from this matrix
    /// @return Reference to this matrix after subtraction
    /// @details Performs in-place element-wise subtraction: this[i][j] -= other[i][j]
    constexpr auto operator-=(const Mat& other) noexcept -> Mat&
    {
        for (std::size_t i = 0; i < R * C; ++i)
        {
            data[i] -= other.data[i];
        }
        return *this;
    }

    /// @brief Compound scalar multiplication assignment operator
    /// @param scalar The scalar value to multiply all elements by
    /// @return Reference to this matrix after multiplication
    /// @details Performs in-place element-wise scalar multiplication: this[i][j] *= scalar
    constexpr auto operator*=(T scalar) noexcept -> Mat&
    {
        for (std::size_t i = 0; i < R * C; ++i)
        {
            data[i] *= scalar;
        }
        return *this;
    }

    /// @brief Scalar division operator
    /// @param scalar The scalar value to divide all elements by
    /// @return A new matrix with all elements divided by the scalar
    /// @details Performs element-wise scalar division: result[i][j] = this[i][j] / scalar
    [[nodiscard]] constexpr auto operator/(T scalar) const noexcept -> Mat
    {
        Mat result;
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = data[i] / scalar;
        }
        return result;
    }

    /// @brief Compound scalar division assignment operator
    /// @param scalar The scalar value to divide all elements by
    /// @return Reference to this matrix after division
    /// @details Performs in-place element-wise scalar division: this[i][j] /= scalar
    constexpr auto operator/=(T scalar) noexcept -> Mat&
    {
        for (std::size_t i = 0; i < R * C; ++i)
        {
            data[i] /= scalar;
        }
        return *this;
    }

    /// @brief Computes the transpose of the matrix
    /// @return A new matrix of dimensions C x R that is the transpose of this matrix
    /// @details The transpose swaps rows and columns: result[j][i] = this[i][j]
    [[nodiscard]] constexpr auto transpose() const noexcept -> Mat<T, C, R>
    {
#ifdef CONFIG_MICROLA_CMSIS
        if constexpr (std::is_same_v<T, float>)
        {
            Mat<T, C, R> result;
            arm_matrix_instance_f32 src, dst;
            arm_mat_init_f32(&src, R, C, const_cast<float*>(data));
            arm_mat_init_f32(&dst, C, R, result.data);
            arm_mat_trans_f32(&src, &dst);
            return result;
        }
#endif
        Mat<T, C, R> result;
        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                result.data[j * R + i] = data[i * C + j];
            }
        }
        return result;
    }

    /// @brief Returns the total number of elements in the matrix
    /// @return The total number of elements (R * C)
    static constexpr auto size() noexcept -> std::size_t { return R * C; }

    /// @brief Returns the number of rows in the matrix
    /// @return The number of rows (R)
    // rows()/cols() helpers are declared earlier to avoid duplication

    /// @brief Creates a 3D rotation matrix around the X-axis
    /// @param angle The rotation angle in radians
    /// @return A 3x3 rotation matrix for rotation around the X-axis
    /// @details Only available for 3x3 matrices. Uses right-hand rule convention.
    ///          In C++26+, this function is constexpr when angle is known at compile-time.
    /// @note Requires R == 3 and C == 3
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation_x(T angle) -> Mat
    {
        static_assert(R == 3 && C == 3, "rotation_x is only defined for 3x3 matrices.");
        Mat result = identity();
        result(1, 1) = std::cos(angle);
        result(1, 2) = -std::sin(angle);
        result(2, 1) = std::sin(angle);
        result(2, 2) = std::cos(angle);
        return result;
    }

    /// @brief Creates a 3D rotation matrix around the Y-axis
    /// @param angle The rotation angle in radians
    /// @return A 3x3 rotation matrix for rotation around the Y-axis
    /// @details Only available for 3x3 matrices. Uses right-hand rule convention.
    ///          In C++26+, this function is constexpr when angle is known at compile-time.
    /// @note Requires R == 3 and C == 3
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation_y(T angle) -> Mat
    {
        static_assert(R == 3 && C == 3, "rotation_y is only defined for 3x3 matrices.");
        Mat result = identity();
        result(0, 0) = std::cos(angle);
        result(0, 2) = std::sin(angle);
        result(2, 0) = -std::sin(angle);
        result(2, 2) = std::cos(angle);
        return result;
    }

    /// @brief Creates a 3D rotation matrix around the Z-axis
    /// @param angle The rotation angle in radians
    /// @return A 3x3 rotation matrix for rotation around the Z-axis
    /// @details Only available for 3x3 matrices. Uses right-hand rule convention.
    ///          In C++26+, this function is constexpr when angle is known at compile-time.
    /// @note Requires R == 3 and C == 3
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation_z(T angle) -> Mat
    {
        static_assert(R == 3 && C == 3, "rotation_z is only defined for 3x3 matrices.");
        Mat result = identity();
        result(0, 0) = std::cos(angle);
        result(0, 1) = -std::sin(angle);
        result(1, 0) = std::sin(angle);
        result(1, 1) = std::cos(angle);
        return result;
    }

    /// @brief Creates a 2D rotation matrix
    /// @param angle The rotation angle in radians
    /// @return A 2x2 rotation matrix
    /// @details Creates a standard 2D rotation matrix using cos and sin.
    ///          Positive angles rotate counter-clockwise.
    ///          In C++26+, this function is constexpr when angle is known at compile-time.
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation(T angle) -> Mat<T, 2, 2>
    {
        Mat<T, 2, 2> result = identity();
        result(0, 0) = std::cos(angle);
        result(0, 1) = -std::sin(angle);
        result(1, 0) = std::sin(angle);
        result(1, 1) = std::cos(angle);
        return result;
    }

    /// @brief Creates a 3D rotation matrix from axis and angle (Rodrigues' formula)
    /// @param axis The rotation axis (should be normalized)
    /// @param angle The rotation angle in radians
    /// @return A 3x3 rotation matrix
    /// @details Implements Rodrigues' rotation formula for rotation around an arbitrary axis.
    ///          R = I + sin(θ)K + (1-cos(θ))K², where K is the skew-symmetric cross-product matrix.
    ///          In C++26+, this function is constexpr when parameters are known at compile-time.
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation_axis_angle(const Vec<T, 3>& axis, T angle) -> Mat<T, 3, 3>
    {
        static_assert(R == 3 && C == 3, "rotation_axis_angle is only defined for 3x3 matrices.");

        const T c = std::cos(angle);
        const T s = std::sin(angle);
        const T t = T(1) - c;

        // Normalize axis (in case it's not already)
        const Vec<T, 3> a = axis.normalized();

        // Rodrigues' formula
        Mat<T, 3, 3> result;
        result(0, 0) = t * a[0] * a[0] + c;
        result(0, 1) = t * a[0] * a[1] - s * a[2];
        result(0, 2) = t * a[0] * a[2] + s * a[1];

        result(1, 0) = t * a[0] * a[1] + s * a[2];
        result(1, 1) = t * a[1] * a[1] + c;
        result(1, 2) = t * a[1] * a[2] - s * a[0];

        result(2, 0) = t * a[0] * a[2] - s * a[1];
        result(2, 1) = t * a[1] * a[2] + s * a[0];
        result(2, 2) = t * a[2] * a[2] + c;

        return result;
    }

    /// @brief Creates a rotation matrix that rotates from one vector to another
    /// @param from The source vector (should be normalized)
    /// @param to The destination vector (should be normalized)
    /// @return A 3x3 rotation matrix
    /// @details Computes the rotation matrix that transforms the 'from' vector to the 'to' vector.
    ///          Uses the axis-angle approach: axis = from × to, angle = acos(from · to).
    ///          Handles the special cases of parallel and antiparallel vectors.
    ///          In C++26+, this function is constexpr when parameters are known at compile-time.
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto rotation_from_to(const Vec<T, 3>& from,
                                                                      const Vec<T, 3>& to) -> Mat<T, 3, 3>
    {
        static_assert(R == 3 && C == 3, "rotation_from_to is only defined for 3x3 matrices.");

        const Vec<T, 3> v1 = from.normalized();
        const Vec<T, 3> v2 = to.normalized();

        const T dot_product = v1.dot(v2);

        // Check if vectors are parallel (same direction)
        if (dot_product >= T(0.999999))
        {
            return identity();
        }

        // Check if vectors are antiparallel (opposite direction)
        if (dot_product <= T(-0.999999))
        {
            // Find an orthogonal vector to use as rotation axis (180° rotation)
            Vec<T, 3> ortho;
            if (std::abs(v1[0]) < std::abs(v1[1]) && std::abs(v1[0]) < std::abs(v1[2]))
            {
                ortho = Vec<T, 3>(T(1), T(0), T(0));
            }
            else if (std::abs(v1[1]) < std::abs(v1[2]))
            {
                ortho = Vec<T, 3>(T(0), T(1), T(0));
            }
            else
            {
                ortho = Vec<T, 3>(T(0), T(0), T(1));
            }

            const Vec<T, 3> axis = v1.cross(ortho).normalized();
            return rotation_axis_angle(axis, constants::pi<T>());
        }

        // General case: compute rotation axis and angle
        const Vec<T, 3> axis = v1.cross(v2).normalized();
        const T angle = std::acos(dot_product);

        return rotation_axis_angle(axis, angle);
    }

    /// @brief Creates a look-at rotation matrix
    /// @param target The target direction vector (not position)
    /// @param up The up direction vector
    /// @return A 3x3 rotation matrix
    /// @details Creates a rotation matrix for orienting towards a target with a specified up direction.
    ///          Commonly used in graphics applications for camera orientation.
    ///          Constructs an orthonormal basis from the forward (target), up, and right vectors.
    ///          In C++26+, this function is constexpr when parameters are known at compile-time.
    [[nodiscard]] static MICROLA_CONSTEXPR_TRIG auto look_at(const Vec<T, 3>& target,
                                                             const Vec<T, 3>& up) -> Mat<T, 3, 3>
    {
        static_assert(R == 3 && C == 3, "look_at is only defined for 3x3 matrices.");

        // Forward direction (normalized target)
        const Vec<T, 3> forward = target.normalized();

        // Right vector (perpendicular to forward and up)
        const Vec<T, 3> right = forward.cross(up).normalized();

        // Recalculate up vector to ensure orthogonality
        const Vec<T, 3> actual_up = right.cross(forward).normalized();

        // Build rotation matrix from orthonormal basis
        Mat<T, 3, 3> result;
        result(0, 0) = right[0];
        result(0, 1) = right[1];
        result(0, 2) = right[2];

        result(1, 0) = actual_up[0];
        result(1, 1) = actual_up[1];
        result(1, 2) = actual_up[2];

        result(2, 0) = forward[0];
        result(2, 1) = forward[1];
        result(2, 2) = forward[2];

        return result;
    }

    /// @brief Computes the trace of the matrix
    /// @return The sum of diagonal elements
    /// @details The trace is the sum of the diagonal elements: sum(M[i][i])
    ///          Only available for square matrices (R == C).
    [[nodiscard]] constexpr auto trace() const noexcept -> T
    {
        static_assert(R == C, "Trace is only defined for square matrices.");
        T sum = T(0);
        for (std::size_t i = 0; i < R; ++i)
        {
            sum += data[i * C + i];
        }
        return sum;
    }

    /// @brief Computes the Frobenius norm of the matrix
    /// @return The Frobenius norm (square root of sum of squared elements)
    /// @details The Frobenius norm is: ||A||_F = sqrt(sum(a_ij^2))
    ///          This is the matrix equivalent of the Euclidean vector norm.
    ///          Useful for measuring matrix "size" and convergence in iterative algorithms.
    ///          In C++26+, this function is constexpr when the matrix is known at compile-time.
    [[nodiscard]] MICROLA_CONSTEXPR_TRIG auto frobenius_norm() const noexcept -> T
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            const auto* src = data;
            auto n = R * C;
            float acc4[4] = {0.0F, 0.0F, 0.0F, 0.0F};
            std::size_t i = 0;
            for (; n - i >= 4; i += 4)
            {
                float tmp[4];
                microla::simd::load_padded4(src + i, 4, tmp);
                microla::simd::fma4_accumulate(acc4, tmp, tmp);
            }
            float sum = acc4[0] + acc4[1] + acc4[2] + acc4[3];
            for (; i < n; ++i)
            {
                sum += src[i] * src[i];
            }
            return std::sqrt(sum);
        }

        T sum = static_cast<T>(0);
        for (std::size_t i = 0; i < R * C; ++i)
        {
            sum += data[i] * data[i];
        }
        return std::sqrt(sum);
    }

    /// @brief Computes the infinity norm (maximum absolute row sum)
    /// @return The infinity norm ||A||_∞
    /// @details The infinity norm is: ||A||_∞ = max_i(sum_j(|a_ij|))
    ///          This represents the maximum absolute row sum.
    [[nodiscard]] auto norm_inf() const noexcept -> T
    {
        T max_sum = static_cast<T>(0);
        for (std::size_t i = 0; i < R; ++i)
        {
            T row_sum = static_cast<T>(0);
            for (std::size_t j = 0; j < C; ++j)
            {
                row_sum += std::abs((*this)(i, j));
            }
            max_sum = std::max(max_sum, row_sum);
        }
        return max_sum;
    }

    /// @brief Computes the 1-norm (maximum absolute column sum)
    /// @return The 1-norm ||A||_1
    /// @details The 1-norm is: ||A||_1 = max_j(sum_i(|a_ij|))
    ///          This represents the maximum absolute column sum.
    [[nodiscard]] auto norm_1() const noexcept -> T
    {
        T max_sum = static_cast<T>(0);
        for (std::size_t j = 0; j < C; ++j)
        {
            T col_sum = static_cast<T>(0);
            for (std::size_t i = 0; i < R; ++i)
            {
                col_sum += std::abs((*this)(i, j));
            }
            max_sum = std::max(max_sum, col_sum);
        }
        return max_sum;
    }

    /// @brief Alias for norm_inf() - computes the infinity norm
    /// @return The infinity norm ||A||_∞
    [[nodiscard]] auto infinity_norm() const noexcept -> T { return norm_inf(); }

    /// @brief Alias for norm_1() - computes the 1-norm
    /// @return The 1-norm ||A||_1
    [[nodiscard]] auto one_norm() const noexcept -> T { return norm_1(); }

    /// @brief Estimates the condition number using norm_inf
    /// @return The condition number κ(A) = ||A||_∞ * ||A^-1||_∞
    /// @details The condition number measures how sensitive the solution to Ax=b is
    ///          to perturbations in A or b. Values close to 1 indicate well-conditioned
    ///          matrices, while large values indicate ill-conditioned matrices.
    ///          Only available for square matrices (R == C).
    /// @note This requires computing the inverse, so it's relatively expensive.
    ///       For large matrices, consider iterative estimation methods.
    [[nodiscard]] auto condition_number() const -> T
    {
        static_assert(R == C, "Condition number is only defined for square matrices.");
        T norm_a = norm_inf();
        Mat a_inv = inverse();
        T norm_a_inv = a_inv.norm_inf();
        return norm_a * norm_a_inv;
    }

    /// @brief Scales a matrix by a scalar value (static method)
    /// @param m The matrix to scale
    /// @param scalar The scalar value
    /// @return A new matrix with all elements multiplied by the scalar
    /// @details Static helper function for scaling matrices: result[i][j] = m[i][j] * scalar
    [[nodiscard]] static constexpr auto scale(const Mat& m, T scalar) noexcept -> Mat
    {
        Mat result;
        for (std::size_t i = 0; i < R * C; ++i)
        {
            result.data[i] = m.data[i] * scalar;
        }
        return result;
    }

    /// @brief Extracts Euler angles from a rotation matrix
    /// @return A 3D vector containing [roll, pitch, yaw] in radians
    /// @details Extracts Euler angles using the convention:
    ///          - angles[0] = roll (rotation around X)
    ///          - angles[1] = pitch (rotation around Y)
    ///          - angles[2] = yaw (rotation around Z)
    ///          In C++26+, this function is constexpr when the matrix is known at compile-time.
    /// @note Only available for 3x3 matrices. Assumes the matrix is a valid rotation matrix.
    [[nodiscard]] MICROLA_CONSTEXPR_TRIG auto euler_angles() const -> Vec<T, 3>
    {
        static_assert(R == 3 && C == 3, "Euler angles are only defined for 3x3 matrices.");
        Vec<T, 3> angles;
        angles[0] = std::atan2(data[7], data[8]);  // Roll
        angles[1] = std::asin(-data[6]);           // Pitch
        angles[2] = std::atan2(data[3], data[0]);  // Yaw
        return angles;
    }

    /// @brief Performs LU decomposition with partial pivoting
    /// @return A tuple containing (L, U, P) where PA = LU
    /// @details Decomposes the matrix into:
    ///          - L: Lower triangular matrix with 1s on diagonal
    ///          - U: Upper triangular matrix
    ///          - P: Permutation matrix
    ///          Only available for square matrices (R == C).
    [[nodiscard]] auto lu() const -> std::tuple<Mat, Mat, Mat>
    {
        static_assert(R == C, "LU decomposition is only defined for square matrices.");
        Mat l = identity();
        Mat u = *this;
        Mat p = identity();

        for (std::size_t i = 0; i < R; ++i)
        {
            // Pivoting
            std::size_t pivot = i;
            for (std::size_t j = i + 1; j < R; ++j)
            {
                if (std::abs(u(j, i)) > std::abs(u(pivot, i)))
                {
                    pivot = j;
                }
            }
            if (pivot != i)
            {
                std::swap_ranges(u.data + i * C, u.data + (i + 1) * C, u.data + pivot * C);
                std::swap_ranges(p.data + i * C, p.data + (i + 1) * C, p.data + pivot * C);
            }

            // Gaussian elimination
            for (std::size_t j = i + 1; j < R; ++j)
            {
                T factor = u(j, i) / u(i, i);
                l(j, i) = factor;
                for (std::size_t k = i; k < C; ++k)
                {
                    u(j, k) -= factor * u(i, k);
                }
            }
        }
        return std::make_tuple(l, u, p);
    }

    /// @brief Performs Cholesky decomposition for symmetric positive-definite matrices
    /// @return Lower triangular matrix L such that A = L * L^T, or std::nullopt on failure
    /// @details Decomposes a symmetric positive-definite matrix into:
    ///          A = L * L^T where L is lower triangular.
    ///          This is more efficient than LU decomposition for this special case.
    ///          Only available for square matrices (R == C).
    /// @note Matrix must be symmetric and positive-definite for this decomposition to succeed.
    [[nodiscard]] auto cholesky() const noexcept -> std::optional<Mat>
    {
        static_assert(R == C, "Cholesky decomposition is only defined for square matrices.");

        Mat l;  // Initialize to zero

        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j <= i; ++j)
            {
                T sum = static_cast<T>(0);

                if (i == j)
                {
                    // Diagonal elements
                    for (std::size_t k = 0; k < j; ++k)
                    {
                        sum += l(j, k) * l(j, k);
                    }

                    T diag_val = (*this)(j, j) - sum;

                    // Check for positive-definiteness
                    if (diag_val <= static_cast<T>(0))
                    {
                        return std::nullopt;
                    }

                    l(j, j) = std::sqrt(diag_val);
                }
                else
                {
                    // Off-diagonal elements
                    for (std::size_t k = 0; k < j; ++k)
                    {
                        sum += l(i, k) * l(j, k);
                    }

                    l(i, j) = ((*this)(i, j) - sum) / l(j, j);
                }
            }
        }

        return l;
    }

    /// @brief Computes eigenvalues using the QR algorithm
    /// @param maxIterations Maximum number of QR iterations (default: 100)
    /// @param tolerance Convergence tolerance (default: 1e-6)
    /// @return A vector containing the eigenvalues
    /// @details Uses iterative QR decomposition to compute eigenvalues.
    ///          Only available for square matrices (R == C).
    /// @note Convergence is not guaranteed for all matrices
    /// @brief Computes eigenvalues using the QR algorithm into a caller-provided buffer
    /// @param out_buffer Pointer to a buffer with space for at least R elements
    /// @param out_size Size of the provided buffer
    /// @param maxIterations Maximum number of QR iterations (default: 100)
    /// @param tolerance Convergence tolerance (default: 1e-6)
    /// @return true on success, false on failure (e.g., out_size < R or no convergence)
    [[nodiscard]] auto eigenvalues_qr(T* out_buffer, std::size_t out_size, std::size_t maxIterations = 100,
                                      T tolerance = 1e-6) const noexcept -> bool
    {
        static_assert(R == C, "Eigenvalue computation is only defined for square matrices.");
        if (out_buffer == nullptr || out_size < R)
        {
            return false;
        }

        Mat a = *this;
        Mat q;
        Mat r_mat;  // Renamed R to R_mat to avoid shadowing the template parameter

        bool converged = (R <= 1);
        for (std::size_t iter = 0; iter < maxIterations; ++iter)
        {
            // QR decomposition
            std::tie(q, r_mat) = a.qr();

            // Update A
            a = r_mat * q;

            // Check for convergence
            converged = true;
            for (std::size_t i = 0; i < R - 1; ++i)
            {
                if (std::abs(a(i + 1, i)) > tolerance)
                {
                    converged = false;
                    break;
                }
            }
            if (converged)
            {
                break;
            }
        }

        if (!converged)
        {
            return false;
        }

        // Extract eigenvalues (diagonal elements)
        for (std::size_t i = 0; i < R; ++i)
        {
            out_buffer[i] = a(i, i);
        }
        return true;
    }

    MICROLA_DYNAMIC_ALLOC_ONLY(
        /// @brief Convenience wrapper that returns eigenvalues or an error
        [[nodiscard]] auto eigenvalues_qr(std::size_t maxIterations = 100, T tolerance = 1e-6)
            const->std::variant<std::vector<T>, EigenError> {
                std::vector<T> result(R);
                const bool ok = eigenvalues_qr(result.data(), result.size(), maxIterations, tolerance);
                if (!ok)
                {
                    return EigenError::NotConverged;
                }
                return result;
            })

    /// @brief Performs QR decomposition using Householder reflections
    /// @return A tuple containing (Q, R) where A = QR
    /// @details Decomposes the matrix into:
    ///          - Q: Orthogonal matrix (Q^T * Q = I)
    ///          - R: Upper triangular matrix
    ///          Only available for square matrices (R == C).
    [[nodiscard]] auto qr() const -> std::tuple<Mat, Mat>
    {
        static_assert(R == C, "QR decomposition is only defined for square matrices.");
        Mat q = identity();
        Mat r_mat = *this;

        for (std::size_t k = 0; k < std::min(R - 1, C); ++k)
        {
            // Compute column norm from k to end
            T norm = T(0);
            for (std::size_t i = k; i < R; ++i)
            {
                norm += r_mat(i, k) * r_mat(i, k);
            }
            norm = std::sqrt(norm);

            if (norm < std::numeric_limits<T>::epsilon())
            {
                continue;  // Skip zero columns
            }

            // Compute Householder vector
            T sign = (r_mat(k, k) >= T(0)) ? T(1) : T(-1);
            T u1 = r_mat(k, k) + sign * norm;

            // Store Householder vector in a temporary
            Vec<T, R> v;
            v[k] = u1;
            for (std::size_t i = k + 1; i < R; ++i)
            {
                v[i] = r_mat(i, k);
            }

            // Compute beta for Householder reflection: beta = 2 / (v^T * v)
            T v_norm_sq = u1 * u1;
            for (std::size_t i = k + 1; i < R; ++i)
            {
                v_norm_sq += v[i] * v[i];
            }
            T beta = T(2) / v_norm_sq;

            // Apply Householder transformation to R: R = (I - beta*v*v^T) * R
            for (std::size_t j = k; j < C; ++j)
            {
                T sum = T(0);
                for (std::size_t i = k; i < R; ++i)
                {
                    sum += v[i] * r_mat(i, j);
                }
                for (std::size_t i = k; i < R; ++i)
                {
                    r_mat(i, j) -= beta * sum * v[i];
                }
            }

            // Apply Householder transformation to Q: Q = Q * (I - beta*v*v^T)
            for (std::size_t i = 0; i < R; ++i)
            {
                T sum = T(0);
                for (std::size_t j = k; j < R; ++j)
                {
                    sum += q(i, j) * v[j];
                }
                for (std::size_t j = k; j < R; ++j)
                {
                    q(i, j) -= beta * sum * v[j];
                }
            }
        }

        return std::make_tuple(q, r_mat);
    }

    /// @brief Performs Singular Value Decomposition (SVD) using Jacobi algorithm
    /// @param maxIterations Maximum number of iterations (default: 100)
    /// @param tolerance Convergence tolerance (default: 1e-9)
    /// @return A tuple containing (U, S, V) where A = U * S * V^T
    /// @details Decomposes the matrix into:
    ///          - U: Left singular vectors (R×R orthogonal matrix)
    ///          - S: Singular values as diagonal matrix (R×C)
    ///          - V: Right singular vectors (C×C orthogonal matrix)
    ///          Uses the Jacobi algorithm which is stable for embedded systems.
    /// @note For non-square matrices, this uses the covariance method: A^T*A for tall matrices
    ///       or A*A^T for wide matrices. For best accuracy, prefer square or nearly-square matrices.
    [[nodiscard]] auto svd(std::size_t maxIterations = 100,
                           T tolerance = 1e-9) const -> std::tuple<Mat<T, R, R>, Mat<T, R, C>, Mat<T, C, C>>
    {
        // Initialize result matrices
        Mat<T, R, R> u = Mat<T, R, R>::identity();
        Mat<T, C, C> v = Mat<T, C, C>::identity();
        Mat<T, R, C> s = *this;

        if constexpr (R >= C)
        {
            // Tall or square matrix: compute V from A^T * A
            Mat<T, C, C> ata;
            for (std::size_t i = 0; i < C; ++i)
            {
                for (std::size_t j = 0; j < C; ++j)
                {
                    T sum = static_cast<T>(0);
                    for (std::size_t k = 0; k < R; ++k)
                    {
                        sum += (*this)(k, i) * (*this)(k, j);
                    }
                    ata(i, j) = sum;
                }
            }

            // Jacobi iterations for symmetric ATA
            for (std::size_t iter = 0; iter < maxIterations; ++iter)
            {
                T max_off_diag = static_cast<T>(0);
                std::size_t p = 0;
                std::size_t q = 1;

                // Find largest off-diagonal element
                for (std::size_t i = 0; i < C; ++i)
                {
                    for (std::size_t j = i + 1; j < C; ++j)
                    {
                        if (std::abs(ata(i, j)) > max_off_diag)
                        {
                            max_off_diag = std::abs(ata(i, j));
                            p = i;
                            q = j;
                        }
                    }
                }

                // Check convergence
                if (max_off_diag < tolerance)
                {
                    break;
                }

                // Compute Jacobi rotation
                T theta = (ata(q, q) - ata(p, p)) / (static_cast<T>(2) * ata(p, q));
                T t = static_cast<T>(1) / (std::abs(theta) + std::sqrt(static_cast<T>(1) + theta * theta));
                if (theta < static_cast<T>(0))
                {
                    t = -t;
                }

                T c = static_cast<T>(1) / std::sqrt(static_cast<T>(1) + t * t);
                T s = t * c;

                // Apply rotation to ATA
                for (std::size_t i = 0; i < C; ++i)
                {
                    if (i != p && i != q)
                    {
                        T aip = ata(i, p);
                        T aiq = ata(i, q);
                        ata(i, p) = c * aip - s * aiq;
                        ata(p, i) = ata(i, p);
                        ata(i, q) = s * aip + c * aiq;
                        ata(q, i) = ata(i, q);
                    }
                }

                T app = ata(p, p);
                T aqq = ata(q, q);
                T apq = ata(p, q);
                ata(p, p) = c * c * app - static_cast<T>(2) * s * c * apq + s * s * aqq;
                ata(q, q) = s * s * app + static_cast<T>(2) * s * c * apq + c * c * aqq;
                ata(p, q) = static_cast<T>(0);
                ata(q, p) = static_cast<T>(0);

                // Update V
                for (std::size_t i = 0; i < C; ++i)
                {
                    T vip = v(i, p);
                    T viq = v(i, q);
                    v(i, p) = c * vip - s * viq;
                    v(i, q) = s * vip + c * viq;
                }
            }

            // Extract singular values (sqrt of eigenvalues of A^T*A)
            for (std::size_t i = 0; i < C; ++i)
            {
                T sigma = std::sqrt(std::abs(ata(i, i)));

                // Compute U column: U[:, i] = A * V[:, i] / sigma
                if (sigma > tolerance)
                {
                    for (std::size_t j = 0; j < R; ++j)
                    {
                        T sum = static_cast<T>(0);
                        for (std::size_t k = 0; k < C; ++k)
                        {
                            sum += (*this)(j, k) * v(k, i);
                        }
                        u(j, i) = sum / sigma;
                    }
                }

                // Store singular value in S
                s(i, i) = sigma;
                for (std::size_t j = 0; j < C; ++j)
                {
                    if (i != j)
                    {
                        s(i, j) = static_cast<T>(0);
                    }
                }
            }

            // Fill remaining rows of S with zeros
            for (std::size_t i = C; i < R; ++i)
            {
                for (std::size_t j = 0; j < C; ++j)
                {
                    s(i, j) = static_cast<T>(0);
                }
            }
        }
        else
        {
            // Wide matrix: compute U from A * A^T
            Mat<T, R, R> aat;
            for (std::size_t i = 0; i < R; ++i)
            {
                for (std::size_t j = 0; j < R; ++j)
                {
                    T sum = static_cast<T>(0);
                    for (std::size_t k = 0; k < C; ++k)
                    {
                        sum += (*this)(i, k) * (*this)(j, k);
                    }
                    aat(i, j) = sum;
                }
            }

            // Jacobi iterations for symmetric AAT
            for (std::size_t iter = 0; iter < maxIterations; ++iter)
            {
                T max_off_diag = static_cast<T>(0);
                std::size_t p = 0;
                std::size_t q = 1;

                for (std::size_t i = 0; i < R; ++i)
                {
                    for (std::size_t j = i + 1; j < R; ++j)
                    {
                        if (std::abs(aat(i, j)) > max_off_diag)
                        {
                            max_off_diag = std::abs(aat(i, j));
                            p = i;
                            q = j;
                        }
                    }
                }

                if (max_off_diag < tolerance)
                {
                    break;
                }

                T theta = (aat(q, q) - aat(p, p)) / (static_cast<T>(2) * aat(p, q));
                T t = static_cast<T>(1) / (std::abs(theta) + std::sqrt(static_cast<T>(1) + theta * theta));
                if (theta < static_cast<T>(0))
                {
                    t = -t;
                }

                T c = static_cast<T>(1) / std::sqrt(static_cast<T>(1) + t * t);
                T s_rot = t * c;

                for (std::size_t i = 0; i < R; ++i)
                {
                    if (i != p && i != q)
                    {
                        T aip = aat(i, p);
                        T aiq = aat(i, q);
                        aat(i, p) = c * aip - s_rot * aiq;
                        aat(p, i) = aat(i, p);
                        aat(i, q) = s_rot * aip + c * aiq;
                        aat(q, i) = aat(i, q);
                    }
                }

                T app = aat(p, p);
                T aqq = aat(q, q);
                T apq = aat(p, q);
                aat(p, p) = c * c * app - static_cast<T>(2) * s_rot * c * apq + s_rot * s_rot * aqq;
                aat(q, q) = s_rot * s_rot * app + static_cast<T>(2) * s_rot * c * apq + c * c * aqq;
                aat(p, q) = static_cast<T>(0);
                aat(q, p) = static_cast<T>(0);

                // Update U
                for (std::size_t i = 0; i < R; ++i)
                {
                    T uip = u(i, p);
                    T uiq = u(i, q);
                    u(i, p) = c * uip - s_rot * uiq;
                    u(i, q) = s_rot * uip + c * uiq;
                }
            }

            // Extract singular values (sqrt of eigenvalues of A*A^T)
            for (std::size_t i = 0; i < R; ++i)
            {
                T sigma = std::sqrt(std::abs(aat(i, i)));

                // Compute V column: V[:, i] = A^T * U[:, i] / sigma
                if (sigma > tolerance)
                {
                    for (std::size_t j = 0; j < C; ++j)
                    {
                        T sum = static_cast<T>(0);
                        for (std::size_t k = 0; k < R; ++k)
                        {
                            sum += (*this)(k, j) * u(k, i);
                        }
                        v(j, i) = sum / sigma;
                    }
                }

                // Store singular value in S
                s(i, i) = sigma;
                for (std::size_t j = 0; j < C; ++j)
                {
                    if (i != j)
                    {
                        s(i, j) = static_cast<T>(0);
                    }
                }
            }

            // Fill remaining columns of S with zeros
            for (std::size_t i = R; i < R; ++i)
            {
                for (std::size_t j = 0; j < C; ++j)
                {
                    s(i, j) = static_cast<T>(0);
                }
            }
        }

        return std::make_tuple(u, s, v);
    }

    /// @brief Computes the rank of the matrix
    /// @return The rank (number of linearly independent rows/columns)
    /// @details Uses Gaussian elimination to determine the number of non-zero rows.
    ///          The rank represents the dimension of the matrix's column space.
    [[nodiscard]] constexpr auto rank() const -> std::size_t
    {
        Mat temp = *this;
        std::size_t r = 0;
        const T tol = T(100) * std::numeric_limits<T>::epsilon();

        for (std::size_t col = 0; col < C && r < R; ++col)
        {
            // Find pivot row with largest absolute value in column
            std::size_t pivot = r;
            T max_val = std::abs(temp(r, col));
            for (std::size_t k = r + 1; k < R; ++k)
            {
                T abs_val = std::abs(temp(k, col));
                if (abs_val > max_val)
                {
                    max_val = abs_val;
                    pivot = k;
                }
            }

            // Skip column if all values below tolerance
            if (max_val <= tol)
            {
                continue;
            }

            // Swap rows if needed
            if (pivot != r)
            {
                for (std::size_t j = col; j < C; ++j)
                {
                    std::swap(temp(r, j), temp(pivot, j));
                }
            }

            // Eliminate below
            for (std::size_t i = r + 1; i < R; ++i)
            {
                T factor = temp(i, col) / temp(r, col);
                for (std::size_t j = col; j < C; ++j)
                {
                    temp(i, j) -= factor * temp(r, j);
                }
            }
            ++r;
        }
        return r;
    }
};

// ============================================================================
// Identity Matrix Helper
// ============================================================================

/// @brief Helper struct for creating identity matrices (general case with loops)
template<typename T, std::size_t R, std::size_t C, typename Enable = void>
struct IdentityHelper
{
    static constexpr auto create() -> Mat<T, R, C>
    {
        static_assert(R == C, "Identity matrix is only defined for square matrices.");
        Mat<T, R, C> result;
        for (std::size_t i = 0; i < R; ++i)
        {
            for (std::size_t j = 0; j < C; ++j)
            {
                result(i, j) = (i == j) ? T(1) : T(0);
            }
        }
        return result;
    }
};

}  // namespace microla

namespace microla
{

/// @brief Type alias for square matrices
/// @tparam T The element type
/// @tparam N The dimension (N x N matrix)
/// @details Convenience alias for creating square matrices where rows == cols
template<typename T, std::size_t N>
using SquareMat = Mat<T, N, N>;

}  // namespace microla
