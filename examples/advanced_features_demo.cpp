// SPDX-License-Identifier: Apache-2.0
/// @file advanced_features_demo.cpp
/// @brief Demonstration of newly added MicroLA features
/// @details Shows usage of block operations, advanced decompositions, norms, and pseudoinverse
/// @author James Baldwin

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>

using namespace microla;

/// @brief Print a matrix with formatting
template<typename T, std::size_t R, std::size_t C>
void print_matrix(const char* name, const Mat<T, R, C>& m)
{
    std::cout << name << " (" << R << "x" << C << "):\n";
    for (std::size_t i = 0; i < R; ++i)
    {
        for (std::size_t j = 0; j < C; ++j)
        {
            std::cout << std::setw(10) << std::setprecision(4) << m(i, j) << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

auto main() -> int
{
    std::cout << "=== MicroLA Advanced Features Demo ===\n\n";

    // ========================================
    // 1. Block Operations
    // ========================================
    std::cout << "1. BLOCK OPERATIONS\n";
    std::cout << "-------------------\n";

    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();
    transform(0, 3) = 10.0F;  // Translation X
    transform(1, 3) = 20.0F;  // Translation Y
    transform(2, 3) = 30.0F;  // Translation Z

    print_matrix("4x4 Transformation Matrix", transform);

    // Extract 3x3 rotation block
    Mat<float, 3, 3> rotation = transform.block<3, 3>(0, 0);
    print_matrix("Extracted 3x3 Rotation Block", rotation);

    // Extract translation vector
    Mat<float, 3, 1> translation = transform.block<3, 1>(0, 3);
    print_matrix("Extracted Translation Vector", translation);

    // ========================================
    // 2. Cholesky Decomposition
    // ========================================
    std::cout << "2. CHOLESKY DECOMPOSITION\n";
    std::cout << "-------------------------\n";

    // Create a symmetric positive-definite matrix
    Mat<float, 3, 3> spd_matrix = {4.0F, 2.0F, 1.0F, 2.0F, 5.0F, 3.0F, 1.0F, 3.0F, 6.0F};

    print_matrix("Symmetric Positive-Definite Matrix", spd_matrix);

    auto l_result = spd_matrix.cholesky();
    if (!l_result.has_value())
    {
        std::cout << "Cholesky decomposition failed (matrix not positive-definite)\n\n";
        return 1;
    }
    Mat<float, 3, 3> l = *l_result;
    print_matrix("Cholesky Factor L (A = L*L^T)", l);

    // Verify: L * L^T should equal original matrix
    Mat<float, 3, 3> l_transpose = l.transpose();
    Mat<float, 3, 3> reconstructed = l * l_transpose;
    print_matrix("Reconstructed Matrix (L*L^T)", reconstructed);

    // ========================================
    // 3. Matrix Norms
    // ========================================
    std::cout << "3. MATRIX NORMS\n";
    std::cout << "---------------\n";

    Mat<float, 3, 3> test_matrix = {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F};

    print_matrix("Test Matrix", test_matrix);

    std::cout << "Frobenius Norm:    " << test_matrix.frobenius_norm() << "\n";
    std::cout << "Infinity Norm:     " << test_matrix.norm_inf() << "\n";
    std::cout << "1-Norm:            " << test_matrix.norm_1() << "\n\n";

    // ========================================
    // 4. Condition Number
    // ========================================
    std::cout << "4. CONDITION NUMBER\n";
    std::cout << "-------------------\n";

    // Well-conditioned matrix (identity)
    Mat<float, 3, 3> identity = Mat<float, 3, 3>::identity();
    std::cout << "Identity Matrix Condition Number: " << identity.condition_number() << "\n";

    // Moderately conditioned matrix
    Mat<float, 3, 3> moderate = {10.0F, 0.0F, 0.0F, 0.0F, 5.0F, 0.0F, 0.0F, 0.0F, 1.0F};
    std::cout << "Diagonal Matrix Condition Number: " << moderate.condition_number() << "\n\n";

    // ========================================
    // 5. Pseudoinverse (Least Squares)
    // ========================================
    std::cout << "5. PSEUDOINVERSE (Least Squares)\n";
    std::cout << "--------------------------------\n";

    // Overdetermined system: 5 equations, 3 unknowns
    Mat<float, 5, 3> a_tall = {1.0F, 0.0F, 0.0F, 1.0F, 1.0F, 0.0F, 1.0F, 2.0F,
                               1.0F, 1.0F, 3.0F, 4.0F, 1.0F, 4.0F, 9.0F};

    print_matrix("Overdetermined System Matrix (5x3)", a_tall);

    Mat<float, 3, 5> a_pinv = a_tall.pseudoinverse();
    print_matrix("Pseudoinverse (3x5)", a_pinv);

    // Verify pseudoinverse property: A * A+ * A ≈ A
    Mat<float, 5, 3> verification = a_tall * a_pinv * a_tall;
    print_matrix("Verification A*A+*A (should ≈ A)", verification);

    // ========================================
    // 6. SVD Decomposition
    // ========================================
    std::cout << "6. SVD DECOMPOSITION\n";
    std::cout << "--------------------\n";

    Mat<float, 3, 3> svd_test = {3.0F, 2.0F, 2.0F, 2.0F, 3.0F, -2.0F, 2.0F, -2.0F, 3.0F};

    print_matrix("Matrix for SVD", svd_test);

    auto svd_result = svd_test.svd();
    auto& u = std::get<0>(svd_result);
    auto& s = std::get<1>(svd_result);
    auto& v = std::get<2>(svd_result);

    print_matrix("U (Left Singular Vectors)", u);
    print_matrix("S (Singular Values)", s);
    print_matrix("V (Right Singular Vectors)", v);

    // Verify: U * S * V^T should equal original matrix
    Mat<float, 3, 3> v_transpose = v.transpose();
    Mat<float, 3, 3> svd_reconstructed = u * s * v_transpose;
    print_matrix("Reconstructed Matrix (U*S*V^T)", svd_reconstructed);

    // ========================================
    // 7. Memory Footprint Information
    // ========================================
    std::cout << "7. MEMORY FOOTPRINT INFORMATION\n";
    std::cout << "--------------------------------\n";

    std::cout << "Mat<float, 4, 4> size:      " << memory_info::matrix_size_bytes<float, 4, 4>() << " bytes\n";
    std::cout << "Mat<float, 4, 4> alignment: " << memory_info::matrix_alignment<float, 4, 4>() << " bytes\n";
    std::cout << "Vec<float, 3> size:         " << memory_info::vector_size_bytes<float, 3>() << " bytes\n";
    std::cout << "Quaternion<float> size:     " << memory_info::quaternion_size_bytes<float>() << " bytes\n";

    // Compile-time validation
    static_assert(memory_info::validate_matrix_stack_size<float, 4, 4>(), "4x4 float matrix fits in stack");
    std::cout << "\nCompile-time validation: 4x4 float matrix fits in stack ✓\n";

    // Check if a large matrix would exceed stack limit
    std::cout << "Mat<float, 100, 100> exceeds stack limit: "
              << (memory_info::matrix_exceeds_stack_limit<float, 100, 100>() ? "Yes" : "No") << "\n";

    std::cout << "\n=== Demo Complete ===\n";

    return 0;
}
