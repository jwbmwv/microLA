// SPDX-License-Identifier: MIT
/// @file advanced_features_demo.cpp
/// @brief Demonstration of newly added MicroLA features
/// @details Shows usage of block operations, advanced decompositions, norms, and pseudoinverse
/// @author James Baldwin

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>

using namespace microla;

/// @brief Print a matrix with formatting
template<typename T, std::uint32_t R, std::uint32_t C>
void print_matrix(const char* name, const Mat<T, R, C>& m)
{
    std::cout << name << " (" << R << "x" << C << "):\n";
    for (std::uint32_t i = 0; i < R; ++i)
    {
        for (std::uint32_t j = 0; j < C; ++j)
        {
            std::cout << std::setw(10) << std::setprecision(4) << m(i, j) << " ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

int main()
{
    std::cout << "=== MicroLA Advanced Features Demo ===\n\n";

    // ========================================
    // 1. Block Operations
    // ========================================
    std::cout << "1. BLOCK OPERATIONS\n";
    std::cout << "-------------------\n";

    Mat<float, 4, 4> transform = Mat<float, 4, 4>::identity();
    transform(0, 3) = 10.0f;  // Translation X
    transform(1, 3) = 20.0f;  // Translation Y
    transform(2, 3) = 30.0f;  // Translation Z

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
    Mat<float, 3, 3> spd_matrix = {4.0f, 2.0f, 1.0f, 2.0f, 5.0f, 3.0f, 1.0f, 3.0f, 6.0f};

    print_matrix("Symmetric Positive-Definite Matrix", spd_matrix);

    Mat<float, 3, 3> L = spd_matrix.cholesky();
    print_matrix("Cholesky Factor L (A = L*L^T)", L);

    // Verify: L * L^T should equal original matrix
    Mat<float, 3, 3> L_transpose = L.transpose();
    Mat<float, 3, 3> reconstructed = L * L_transpose;
    print_matrix("Reconstructed Matrix (L*L^T)", reconstructed);

    // ========================================
    // 3. Matrix Norms
    // ========================================
    std::cout << "3. MATRIX NORMS\n";
    std::cout << "---------------\n";

    Mat<float, 3, 3> test_matrix = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f};

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
    Mat<float, 3, 3> moderate = {10.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    std::cout << "Diagonal Matrix Condition Number: " << moderate.condition_number() << "\n\n";

    // ========================================
    // 5. Pseudoinverse (Least Squares)
    // ========================================
    std::cout << "5. PSEUDOINVERSE (Least Squares)\n";
    std::cout << "--------------------------------\n";

    // Overdetermined system: 5 equations, 3 unknowns
    Mat<float, 5, 3> A_tall = {1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 2.0f,
                               1.0f, 1.0f, 3.0f, 4.0f, 1.0f, 4.0f, 9.0f};

    print_matrix("Overdetermined System Matrix (5x3)", A_tall);

    Mat<float, 3, 5> A_pinv = A_tall.pseudoinverse();
    print_matrix("Pseudoinverse (3x5)", A_pinv);

    // Verify pseudoinverse property: A * A+ * A ≈ A
    Mat<float, 5, 3> verification = A_tall * A_pinv * A_tall;
    print_matrix("Verification A*A+*A (should ≈ A)", verification);

    // ========================================
    // 6. SVD Decomposition
    // ========================================
    std::cout << "6. SVD DECOMPOSITION\n";
    std::cout << "--------------------\n";

    Mat<float, 3, 3> svd_test = {3.0f, 2.0f, 2.0f, 2.0f, 3.0f, -2.0f, 2.0f, -2.0f, 3.0f};

    print_matrix("Matrix for SVD", svd_test);

    auto svd_result = svd_test.svd();
    auto& U = std::get<0>(svd_result);
    auto& S = std::get<1>(svd_result);
    auto& V = std::get<2>(svd_result);

    print_matrix("U (Left Singular Vectors)", U);
    print_matrix("S (Singular Values)", S);
    print_matrix("V (Right Singular Vectors)", V);

    // Verify: U * S * V^T should equal original matrix
    Mat<float, 3, 3> V_transpose = V.transpose();
    Mat<float, 3, 3> svd_reconstructed = U * S * V_transpose;
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

#if __cplusplus >= 201402L
    // Compile-time validation (C++14+)
    static_assert(memory_info::validate_matrix_stack_size<float, 4, 4>(), "4x4 float matrix fits in stack");
    std::cout << "\nCompile-time validation: 4x4 float matrix fits in stack ✓\n";
#else
    // C++11: Runtime check only
    std::cout << "\n4x4 float matrix fits in stack: "
              << (memory_info::validate_matrix_stack_size<float, 4, 4>() ? "Yes ✓" : "No") << "\n";
#endif

    // Check if a large matrix would exceed stack limit
    std::cout << "Mat<float, 100, 100> exceeds stack limit: "
              << (memory_info::matrix_exceeds_stack_limit<float, 100, 100>() ? "Yes" : "No") << "\n";

    std::cout << "\n=== Demo Complete ===\n";

    return 0;
}
