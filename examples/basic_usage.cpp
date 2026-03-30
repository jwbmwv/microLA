// SPDX-License-Identifier: Apache-2.0
/// @file basic_usage.cpp
/// @brief Comprehensive examples demonstrating MicroLA API usage
/// @details This file contains examples covering vector operations, matrix operations,
///          quaternion rotations, SLERP interpolation, and practical IMU sensor fusion.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#include <microla/microla.hpp>
#include <microla/quaternion.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace microla;

/// @brief Helper function to print Vec<float, 3>
/// @param name Descriptive label for the vector
/// @param v The vector to print
void print_vec3(const char* name, const Vec<float, 3>& v)
{
    std::cout << std::setw(12) << name << ": (" << std::setw(8) << std::fixed << std::setprecision(4)
              << v[Vec<float, 3>::X] << ", " << std::setw(8) << std::fixed << std::setprecision(4)
              << v[Vec<float, 3>::Y] << ", " << std::setw(8) << std::fixed << std::setprecision(4)
              << v[Vec<float, 3>::Z] << ")\n";
}

/// @brief Helper function to print Mat<float, R, C>
/// @tparam R Number of rows
/// @tparam C Number of columns
/// @param name Descriptive label for the matrix
/// @param m The matrix to print
template<std::size_t R, std::size_t C>
void print_mat(const char* name, const Mat<float, R, C>& m)
{
    std::cout << name << ":\n";
    for (std::size_t i = 0; i < R; i++)
    {
        std::cout << "  [";
        for (std::size_t j = 0; j < C; j++)
        {
            std::cout << std::setw(8) << std::fixed << std::setprecision(4) << m(i, j);
        }
        std::cout << " ]\n";
    }
}

/// @brief Helper function to print quaternion
/// @param name Descriptive label for the quaternion
/// @param q The quaternion to print
void print_quat(const char* name, const Quaternion<float>& q)
{
    std::cout << std::setw(12) << name << ": (x=" << std::setw(7) << std::fixed << std::setprecision(4) << q.x()
              << ", y=" << std::setw(7) << std::fixed << std::setprecision(4) << q.y() << ", z=" << std::setw(7)
              << std::fixed << std::setprecision(4) << q.z() << ", w=" << std::setw(7) << std::fixed
              << std::setprecision(4) << q.w() << ")\n";
}

/// @brief Main function demonstrating comprehensive MicroLA usage
/// @return 0 on success
auto main() -> int
{
    std::cout << "=== MicroLA Basic Usage Examples ===\n\n";

    constexpr float pi = constants::pi<float>();
    constexpr float radians_to_degrees = 180.0F / pi;

    // ==================== Vector Operations ====================
    std::cout << "--- Vector Operations ---\n";

    Vec<float, 3> v1(1.0F, 0.0F, 0.0F);
    Vec<float, 3> v2(0.0F, 1.0F, 0.0F);

    std::cout << "   Vec3 size: " << microla::Vec<float, 3>::size() << " elements\n";

    print_vec3("v1", v1);
    print_vec3("v2", v2);

    Vec<float, 3> sum = v1 + v2;
    print_vec3("v1 + v2", sum);

    float dot = v1.dot(v2);
    std::cout << "   v1 · v2: " << dot << "\n";

    Vec<float, 3> cross = v1.cross(v2);
    print_vec3("v1 × v2", cross);

    float angle = v1.angle(v2);
    std::cout << "   angle: " << angle << " rad (" << (angle * radians_to_degrees) << "°)\n\n";

    // ==================== Matrix Operations ====================
    std::cout << "--- Matrix Operations ---\n";

    // Create 90° rotation around Z-axis
    Mat<float, 3, 3> rz = Mat<float, 3, 3>::rotation_z(pi / 2.0F);

    std::cout << "   Mat3 dimensions: " << microla::Mat<float, 3, 3>::rows() << "x" << microla::Mat<float, 3, 3>::cols()
              << " (" << microla::Mat<float, 3, 3>::size() << " elements)\n";

    std::cout << "Rotation matrix (90° around Z):\n";
    for (int i = 0; i < 3; i++)
    {
        std::cout << "  [";
        for (int j = 0; j < 3; j++)
        {
            std::cout << std::setw(8) << std::fixed << std::setprecision(4) << rz[i][j];
        }
        std::cout << " ]\n";
    }

    Vec<float, 3> point(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated = rz * point;
    print_vec3("Original", point);
    print_vec3("Rotated", rotated);

    float det = rz.determinant();
    std::cout << "Determinant: " << det << "\n\n";

    // ==================== Quaternion Operations ====================
    std::cout << "--- Quaternion Operations ---\n";

    // Create quaternion for 45° rotation around Z-axis
    Vec<float, 3> axis(0.0F, 0.0F, 1.0F);
    float quat_angle = pi / 4.0F;  // 45 degrees
    Quaternion<float> q(axis, quat_angle);

    std::cout << "Quaternion (45° around Z): " << "w=" << q.w() << ", " << "x=" << q.x() << ", " << "y=" << q.y()
              << ", " << "z=" << q.z() << "\n";

    Vec<float, 3> test_point(1.0F, 0.0F, 0.0F);
    Vec<float, 3> quat_rotated = q.rotate(test_point);
    print_vec3("Original", test_point);
    print_vec3("Quat Rotated", quat_rotated);

    // ==================== SLERP Interpolation ====================
    std::cout << "\n--- SLERP Interpolation ---\n";

    Quaternion<float> q_start = Quaternion<float>::identity();
    Quaternion<float> q_end(axis, pi / 2.0F);  // 90 degrees

    std::cout << "Interpolating from 0° to 90°:\n";
    for (int step = 0; step <= 4; ++step)
    {
        const float t = static_cast<float>(step) * 0.25F;
        Quaternion<float> q_interp = q_start.slerp(q_end, t);
        Vec<float, 3> interp_point = q_interp.rotate(Vec<float, 3>(1.0F, 0.0F, 0.0F));

        std::cout << "  t=" << std::setw(4) << std::fixed << std::setprecision(2) << t << ": ";
        print_vec3("", interp_point);
    }

    // ==================== Accelerometer Example ====================
    std::cout << "\n--- Accelerometer Angle Calculation ---\n";

    Vec<float, 3> accel1(0.0F, 0.0F, 1.0F);      // Pointing up
    Vec<float, 3> accel2(0.707F, 0.0F, 0.707F);  // 45° tilt

    print_vec3("Accel 1", accel1);
    print_vec3("Accel 2", accel2);

    float accel_angle = accel1.angle(accel2);
    std::cout << "Angle between: " << accel_angle << " rad (" << (accel_angle * radians_to_degrees) << "°)\n";

    // Create rotation to align accel1 to accel2
    Mat<float, 3, 3> r_align = Mat<float, 3, 3>::rotation_from_to(accel1, accel2);
    Vec<float, 3> aligned = r_align * accel1;
    print_vec3("Aligned", aligned);

    // ==================== Advanced Vector Operations ====================
    std::cout << "\n--- Advanced Vector Operations ---\n";

    Vec<float, 3> a(3.0F, 4.0F, 0.0F);
    Vec<float, 3> b(1.0F, 0.0F, 0.0F);

    print_vec3("a", a);
    print_vec3("b", b);

    // Normalization and magnitude
    float mag_a = a.length();
    Vec<float, 3> a_norm = a.normalized();
    std::cout << " Magnitude: " << mag_a << "\n";
    print_vec3("Normalized", a_norm);

    // Projection and rejection
    Vec<float, 3> proj_ab = a.project(b);
    Vec<float, 3> rej_ab = a.reject(b);
    print_vec3("Project a→b", proj_ab);
    print_vec3("Reject a⊥b", rej_ab);

    // Verify orthogonality: proj + rej = original
    Vec<float, 3> sum_proj_rej = proj_ab + rej_ab;
    print_vec3("proj + rej", sum_proj_rej);

    // Signed angle
    Vec<float, 3> v_x(1.0F, 0.0F, 0.0F);
    Vec<float, 3> v_y(0.0F, 1.0F, 0.0F);
    Vec<float, 3> v_z(0.0F, 0.0F, 1.0F);

    float signed_ang = v_x.signed_angle(v_y, v_z);
    std::cout << "Signed angle (x→y, around z): " << signed_ang << " rad (" << (signed_ang * radians_to_degrees)
              << "°)\n";

    // ==================== Matrix Operations Extended ====================
    std::cout << "\n--- Extended Matrix Operations ---\n";

    // 2D rotation
    SquareMat<float, 2> r2d = SquareMat<float, 2>::rotation(pi / 6.0F);  // 30°
    print_mat("2D Rotation (30°)", r2d);

    Vec<float, 2> v2d(1.0F, 0.0F);
    Vec<float, 2> v2d_rot = r2d * v2d;
    std::cout << "2D vector rotated: (" << v2d_rot[0] << ", " << v2d_rot[1] << ")\n";

    // 3D rotations around each axis
    SquareMat<float, 3> rx_45 = SquareMat<float, 3>::rotation_x(pi / 4.0F);  // 45° around X
    SquareMat<float, 3> ry_45 = SquareMat<float, 3>::rotation_y(pi / 4.0F);  // 45° around Y
    SquareMat<float, 3> rz_45 = SquareMat<float, 3>::rotation_z(pi / 4.0F);  // 45° around Z

    print_mat("Rotation X (45°)", rx_45);
    print_mat("Rotation Y (45°)", ry_45);
    print_mat("Rotation Z (45°)", rz_45);

    // Rotation around arbitrary axis
    Vec<float, 3> arb_axis(1.0F, 1.0F, 1.0F);
    arb_axis = arb_axis.normalized();
    SquareMat<float, 3> r_arb = SquareMat<float, 3>::rotation_axis_angle(arb_axis, pi / 3.0F);  // 60°
    print_mat("Rotation around (1,1,1) axis (60°)", r_arb);

    // Look-at matrix
    Vec<float, 3> target(1.0F, 1.0F, 0.0F);
    Vec<float, 3> up(0.0F, 0.0F, 1.0F);
    SquareMat<float, 3> look = SquareMat<float, 3>::look_at(target, up);
    print_mat("Look-at matrix", look);

    // Transpose and inverse
    SquareMat<float, 3> rz_t = rz.transpose();
    SquareMat<float, 3> rz_inv = rz.inverse();
    print_mat("Rz Transpose", rz_t);
    print_mat("Rz Inverse", rz_inv);

    // Trace and determinant
    float trace = rz_45.trace();
    float det_45 = rz_45.determinant();
    std::cout << "       Trace: " << trace << "\n";
    std::cout << "Determinant: " << det_45 << "\n";

    // Euler angles
    Vec<float, 3> euler = rz_45.euler_angles();
    std::cout << "Euler angles (roll, pitch, yaw): (" << euler[0] << ", " << euler[1] << ", " << euler[2] << ") rad\n";

    // ==================== General Matrix Multiplication ====================
    std::cout << "\n--- General Matrix Operations ---\n";

    Mat<float, 3, 2> m_a;
    m_a[0][0] = 1.0F;
    m_a[0][1] = 2.0F;
    m_a[1][0] = 3.0F;
    m_a[1][1] = 4.0F;
    m_a[2][0] = 5.0F;
    m_a[2][1] = 6.0F;

    Mat<float, 2, 4> m_b;
    m_b[0][0] = 1.0F;
    m_b[0][1] = 2.0F;
    m_b[0][2] = 3.0F;
    m_b[0][3] = 4.0F;
    m_b[1][0] = 5.0F;
    m_b[1][1] = 6.0F;
    m_b[1][2] = 7.0F;
    m_b[1][3] = 8.0F;

    Mat<float, 3, 4> m_c = m_a * m_b;

    print_mat("A (3×2)", m_a);
    print_mat("B (2×4)", m_b);
    print_mat("C = A×B (3×4)", m_c);

    // ==================== Quaternion Operations Extended ====================
    std::cout << "\n--- Extended Quaternion Operations ---\n";

    // Create various quaternions
    Quaternion<float> q1(Vec<float, 3>(0.0F, 0.0F, 1.0F), pi / 2.0F);  // 90° around Z
    Quaternion<float> q2(Vec<float, 3>(1.0F, 0.0F, 0.0F), pi / 2.0F);  // 90° around X

    print_quat("q1 (90° Z)", q1);
    print_quat("q2 (90° X)", q2);

    // Quaternion multiplication (composition)
    Quaternion<float> q_combined = q1 * q2;
    print_quat("q1 * q2", q_combined);

    // Quaternion conjugate and inverse
    Quaternion<float> q1_conj = q1.conjugate();
    Quaternion<float> q1_inv = q1.inverse();
    print_quat("q1 conjugate", q1_conj);
    print_quat("q1 inverse", q1_inv);

    // Verify inverse: q * q^-1 = identity
    Quaternion<float> q_identity = q1 * q1_inv;
    print_quat("q1*q1^-1", q_identity);

    // Normalize
    Quaternion<float> q_scaled(0.5F, 0.5F, 0.5F, 0.5F);
    float q_norm = q_scaled.norm();
    Quaternion<float> q_normalized = q_scaled.normalized();
    std::cout << "   Norm before: " << q_norm << "\n";
    print_quat("Normalized", q_normalized);
    std::cout << "    Norm after: " << q_normalized.norm() << "\n";

    // Rotate vector with quaternion
    Vec<float, 3> vec_to_rot(1.0F, 0.0F, 0.0F);
    Vec<float, 3> rotated_vec = q1.rotate(vec_to_rot);
    print_vec3("Original", vec_to_rot);
    print_vec3("After q1", rotated_vec);

    // Convert to rotation matrix
    SquareMat<float, 3> r_from_q = q1.to_matrix();
    print_mat("Rotation matrix from q1", r_from_q);

    // ==================== SLERP and Interpolation ====================
    std::cout << "\n--- Advanced SLERP ---\n";

    Quaternion<float> q_start2 = Quaternion<float>::identity();
    Quaternion<float> q_end2(Vec<float, 3>(1.0F, 1.0F, 0.0F).normalized(), pi);  // 180° around diagonal

    std::cout << "Interpolating complex rotation:\n";
    for (int step = 0; step <= 5; ++step)
    {
        const float t = static_cast<float>(step) * 0.2F;
        Quaternion<float> q_interp = q_start2.slerp(q_end2, t);
        std::cout << "  t=" << std::setw(3) << std::fixed << std::setprecision(1) << t << ": ";
        print_quat("", q_interp);
    }

    // ==================== Cross Product Properties ====================
    std::cout << "\n--- Cross Product Properties ---\n";

    Vec<float, 3> u(1.0F, 0.0F, 0.0F);
    Vec<float, 3> v(0.0F, 1.0F, 0.0F);
    Vec<float, 3> w = u.cross(v);

    print_vec3("u", u);
    print_vec3("v", v);
    print_vec3("u × v", w);

    // Anti-commutativity: v × u = -(u × v)
    Vec<float, 3> w_rev = v.cross(u);
    print_vec3("v × u", w_rev);

    // Orthogonality: w ⊥ u and w ⊥ v
    float dot_wu = w.dot(u);
    float dot_wv = w.dot(v);
    std::cout << "  w · u = " << dot_wu << " (should be ~0)\n";
    std::cout << "  w · v = " << dot_wv << " (should be ~0)\n";

    // ==================== Practical IMU Tilt Alignment Example ====================
    std::cout << "\n--- IMU Tilt Alignment Example ---\n";

    // Simulate accelerometer reading (gravity vector in body frame)
    Vec<float, 3> accel_body(0.1F, 0.2F, 9.8F);  // Mostly pointing down with slight tilt
    accel_body = accel_body.normalized();

    // Reference gravity vector (world frame)
    Vec<float, 3> gravity_world(0.0F, 0.0F, 1.0F);

    print_vec3("Accel (body)", accel_body);
    print_vec3("Gravity (world)", gravity_world);

    // Calculate rotation to align body frame with world frame
    SquareMat<float, 3> r_body_to_world = SquareMat<float, 3>::rotation_from_to(accel_body, gravity_world);

    // Verify alignment
    Vec<float, 3> accel_rotated = r_body_to_world * accel_body;
    print_vec3("Accel (rotated)", accel_rotated);

    // Extract orientation angles
    Vec<float, 3> orientation = r_body_to_world.euler_angles();
    std::cout << "Roll, Pitch, Yaw: (" << (orientation[0] * radians_to_degrees) << "°, "
              << (orientation[1] * radians_to_degrees) << "°, " << (orientation[2] * radians_to_degrees) << "°)\n";

    std::cout << "\n=== All examples completed successfully! ===\n";

    return 0;
}
