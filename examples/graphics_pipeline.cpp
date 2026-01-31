// SPDX-License-Identifier: MIT
/// @file graphics_pipeline.cpp
/// @brief 3D graphics transformation example using MicroLA
/// @details Demonstrates basic transformations and rotations for 3D graphics
/// @copyright Copyright (c) 2026 James Baldwin

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace microla;

/// @brief Simple 3D mesh vertex
struct Vertex
{
    Vec<float, 3> position;
    Vec<float, 3> color;
};

/// @brief Apply transformation to a vertex
Vec<float, 3> transform_vertex(const Vertex& v, const Mat<float, 4, 4>& transform)
{
    // Homogeneous coordinates
    Vec<float, 4> pos_h{v.position[0], v.position[1], v.position[2], 1.0f};

    // Apply transform
    Vec<float, 4> transformed = transform * pos_h;

    // Return 3D position
    return Vec<float, 3>{transformed[0], transformed[1], transformed[2]};
}

int main()
{
    std::cout << "MicroLA - Graphics Transformation Example\n";
    std::cout << "==========================================\n\n";

    std::cout << std::fixed << std::setprecision(3);

    // Define a simple cube mesh (8 vertices)
    std::vector<Vertex> cube_vertices = {
        {{-1.0f, -1.0f, -1.0f}, {1.0f, 0.0f, 0.0f}},  // 0: Front-bottom-left (red)
        {{1.0f, -1.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},   // 1: Front-bottom-right (green)
        {{1.0f, 1.0f, -1.0f}, {0.0f, 0.0f, 1.0f}},    // 2: Front-top-right (blue)
        {{-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, 0.0f}},   // 3: Front-top-left (yellow)
        {{-1.0f, -1.0f, 1.0f}, {1.0f, 0.0f, 1.0f}},   // 4: Back-bottom-left (magenta)
        {{1.0f, -1.0f, 1.0f}, {0.0f, 1.0f, 1.0f}},    // 5: Back-bottom-right (cyan)
        {{1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}},     // 6: Back-top-right (white)
        {{-1.0f, 1.0f, 1.0f}, {0.5f, 0.5f, 0.5f}}     // 7: Back-top-left (gray)
    };

    std::cout << "=== 3D Transformations ===\n\n";

    // Identity matrix
    Mat<float, 4, 4> identity = Mat<float, 4, 4>::identity();
    std::cout << "Identity Matrix (4x4):\n";
    for (uint32_t i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (uint32_t j = 0; j < 4; ++j)
        {
            std::cout << std::setw(8) << identity(i, j);
        }
        std::cout << " ]\n";
    }

    // Rotation matrix (45° around Y-axis)
    float angle = deg_to_rad(45.0f);

    // Build a 4x4 rotation matrix from 3x3
    Mat<float, 3, 3> rot3x3 = Mat<float, 3, 3>::rotation_y(angle);
    Mat<float, 4, 4> rotation = Mat<float, 4, 4>::identity();

    // Copy 3x3 rotation into upper-left corner of 4x4
    for (uint32_t i = 0; i < 3; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            rotation(i, j) = rot3x3(i, j);
        }
    }

    std::cout << "\nRotation Matrix (45° around Y):\n";
    for (uint32_t i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (uint32_t j = 0; j < 4; ++j)
        {
            std::cout << std::setw(8) << rotation(i, j);
        }
        std::cout << " ]\n";
    }

    // Translation matrix
    Mat<float, 4, 4> translation = Mat<float, 4, 4>::identity();
    translation(0, 3) = 2.0f;   // X translation
    translation(1, 3) = 1.0f;   // Y translation
    translation(2, 3) = -5.0f;  // Z translation

    std::cout << "\nTranslation Matrix (2, 1, -5):\n";
    for (uint32_t i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (uint32_t j = 0; j < 4; ++j)
        {
            std::cout << std::setw(8) << translation(i, j);
        }
        std::cout << " ]\n";
    }

    // Combined transform: Translate then Rotate
    Mat<float, 4, 4> combined = rotation * translation;

    std::cout << "\nCombined Transform (Rotation * Translation):\n";
    for (uint32_t i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (uint32_t j = 0; j < 4; ++j)
        {
            std::cout << std::setw(8) << combined(i, j);
        }
        std::cout << " ]\n";
    }

    // Transform vertices
    std::cout << "\n=== Vertex Transformation ===\n";
    std::cout << "Original -> Rotated -> Translated -> Combined\n\n";

    for (size_t i = 0; i < 4; ++i)  // Show first 4 vertices
    {
        const Vertex& v = cube_vertices[i];
        Vec<float, 3> rotated = transform_vertex(v, rotation);
        Vec<float, 3> translated = transform_vertex(v, translation);
        Vec<float, 3> final_pos = transform_vertex(v, combined);

        std::cout << "Vertex " << i << ":\n";
        std::cout << "  Original:    (" << std::setw(6) << v.position[0] << ", " << std::setw(6) << v.position[1]
                  << ", " << std::setw(6) << v.position[2] << ")\n";
        std::cout << "  Rotated:     (" << std::setw(6) << rotated[0] << ", " << std::setw(6) << rotated[1] << ", "
                  << std::setw(6) << rotated[2] << ")\n";
        std::cout << "  Translated:  (" << std::setw(6) << translated[0] << ", " << std::setw(6) << translated[1]
                  << ", " << std::setw(6) << translated[2] << ")\n";
        std::cout << "  Combined:    (" << std::setw(6) << final_pos[0] << ", " << std::setw(6) << final_pos[1] << ", "
                  << std::setw(6) << final_pos[2] << ")\n\n";
    }

    // Demonstrate matrix decomposition
    std::cout << "=== Matrix Analysis ===\n\n";

    // Determinant
    float det = rotation.determinant();
    std::cout << "Rotation matrix determinant: " << det << " (should be ~1 for proper rotation)\n";

    // Trace
    float trace = rotation.trace();
    std::cout << "Rotation matrix trace: " << trace << "\n";

    // Frobenius norm
    float norm = rotation.frobenius_norm();
    std::cout << "Rotation matrix Frobenius norm: " << norm << "\n\n";

    // Animation example
    std::cout << "=== Animation Sequence ===\n";
    std::cout << "Rotating cube over time:\n\n";

    Vec<float, 3> test_vertex = cube_vertices[0].position;

    for (int frame = 0; frame <= 4; ++frame)
    {
        float t = frame / 4.0f;
        float anim_angle = t * constants::pi<float>() / 2.0f;  // 0° to 90°

        Mat<float, 3, 3> anim_rot3 = Mat<float, 3, 3>::rotation_y(anim_angle);
        Mat<float, 4, 4> anim_rot4 = Mat<float, 4, 4>::identity();

        for (uint32_t i = 0; i < 3; ++i)
        {
            for (uint32_t j = 0; j < 3; ++j)
            {
                anim_rot4(i, j) = anim_rot3(i, j);
            }
        }

        Vec<float, 3> animated_pos = transform_vertex(cube_vertices[0], anim_rot4);

        std::cout << "Frame " << frame << " (angle=" << std::setw(5) << rad_to_deg(anim_angle) << "°): ("
                  << std::setw(6) << animated_pos[0] << ", " << std::setw(6) << animated_pos[1] << ", " << std::setw(6)
                  << animated_pos[2] << ")\n";
    }

    return 0;
}
