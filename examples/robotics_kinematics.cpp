// SPDX-License-Identifier: Apache-2.0
/// @file robotics_kinematics.cpp
/// @brief Forward and inverse kinematics for a 2-link planar robot arm
/// @details Demonstrates practical robotics calculations using MicroLA
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.

#include <microla/microla.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace microla;

/// @brief 2-link planar robot arm (operates in XY plane)
class PlanarRobotArm
{
private:
    float link1_length;  // Length of first link (meters)
    float link2_length;  // Length of second link (meters)

public:
    PlanarRobotArm(float l1, float l2) : link1_length(l1), link2_length(l2) {}

    /// @brief Forward kinematics: joint angles -> end effector position
    /// @param theta1 Joint 1 angle (radians)
    /// @param theta2 Joint 2 angle (radians)
    /// @return End effector position in world frame
    [[nodiscard]] auto forward_kinematics(float theta1, float theta2) const -> Vec<float, 2>
    {
        // Position of link1 end
        float x1 = link1_length * std::cos(theta1);
        float y1 = link1_length * std::sin(theta1);

        // Position of link2 end (end effector)
        float x2 = x1 + link2_length * std::cos(theta1 + theta2);
        float y2 = y1 + link2_length * std::sin(theta1 + theta2);

        return Vec<float, 2>{x2, y2};
    }

    /// @brief Inverse kinematics: end effector position -> joint angles
    /// @param target Target end effector position
    /// @param elbow_up True for elbow-up solution, false for elbow-down
    /// @return Joint angles [theta1, theta2] (radians), or NaN if unreachable
    [[nodiscard]] auto inverse_kinematics(const Vec<float, 2>& target, bool elbow_up = true) const -> Vec<float, 2>
    {
        float x = target[0];
        float y = target[1];
        float r = std::sqrt(x * x + y * y);

        // Check if target is reachable
        if (r > link1_length + link2_length || r < std::abs(link1_length - link2_length))
        {
            std::cerr << "Target unreachable! Distance: " << r << " meters\n";
            return Vec<float, 2>{NAN, NAN};
        }

        // Law of cosines for theta2
        float cos_theta2 =
            (r * r - link1_length * link1_length - link2_length * link2_length) / (2.0F * link1_length * link2_length);
        cos_theta2 = clamp(cos_theta2, -1.0F, 1.0F);

        float theta2 = elbow_up ? std::acos(cos_theta2) : -std::acos(cos_theta2);

        // Solve for theta1
        float k1 = link1_length + link2_length * std::cos(theta2);
        float k2 = link2_length * std::sin(theta2);
        float theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        return Vec<float, 2>{theta1, theta2};
    }

    /// @brief Compute Jacobian matrix (relates joint velocities to end effector velocities)
    /// @param theta1 Joint 1 angle (radians)
    /// @param theta2 Joint 2 angle (radians)
    /// @return 2x2 Jacobian matrix
    [[nodiscard]] auto jacobian(float theta1, float theta2) const -> Mat<float, 2, 2>
    {
        float s1 = std::sin(theta1);
        float c1 = std::cos(theta1);
        float s12 = std::sin(theta1 + theta2);
        float c12 = std::cos(theta1 + theta2);

        Mat<float, 2, 2> j;
        j(0, 0) = -link1_length * s1 - link2_length * s12;
        j(0, 1) = -link2_length * s12;
        j(1, 0) = link1_length * c1 + link2_length * c12;
        j(1, 1) = link2_length * c12;

        return j;
    }

    /// @brief Get workspace boundary points (circle of reachable positions)
    /// @param num_points Number of boundary points to generate
    void print_workspace(int num_points = 16) const
    {
        std::cout << "\nWorkspace boundary (max reach = " << link1_length + link2_length << " m):\n";

        for (int i = 0; i < num_points; ++i)
        {
            float angle = (2.0F * constants::pi<float>() * static_cast<float>(i)) / static_cast<float>(num_points);
            float x = (link1_length + link2_length) * std::cos(angle);
            float y = (link1_length + link2_length) * std::sin(angle);
            std::cout << "  (" << std::setw(6) << x << ", " << std::setw(6) << y << ")\n";
        }
    }
};

auto main() -> int
{
    std::cout << "MicroLA - Robot Kinematics Example\n";
    std::cout << "=====================================\n\n";

    // Create 2-link robot with 1m links
    PlanarRobotArm robot(1.0F, 1.0F);

    std::cout << std::fixed << std::setprecision(4);

    // Test 1: Forward kinematics
    std::cout << "=== Forward Kinematics ===\n";
    float theta1 = deg_to_rad(45.0F);
    float theta2 = deg_to_rad(30.0F);

    Vec<float, 2> end_pos = robot.forward_kinematics(theta1, theta2);
    std::cout << "Joint angles: θ1=" << rad_to_deg(theta1) << "°, θ2=" << rad_to_deg(theta2) << "°\n";
    std::cout << "End effector: (" << end_pos[0] << ", " << end_pos[1] << ") meters\n";

    // Test 2: Inverse kinematics
    std::cout << "\n=== Inverse Kinematics ===\n";
    Vec<float, 2> target{1.5F, 1.0F};
    std::cout << "Target position: (" << target[0] << ", " << target[1] << ") meters\n";

    Vec<float, 2> angles_up = robot.inverse_kinematics(target, true);
    if (!std::isnan(angles_up[0]))
    {
        std::cout << "Elbow-up solution: θ1=" << rad_to_deg(angles_up[0]) << "°, θ2=" << rad_to_deg(angles_up[1])
                  << "°\n";

        // Verify
        Vec<float, 2> verify = robot.forward_kinematics(angles_up[0], angles_up[1]);
        std::cout << "  Verification: (" << verify[0] << ", " << verify[1] << ")\n";
    }

    Vec<float, 2> angles_down = robot.inverse_kinematics(target, false);
    if (!std::isnan(angles_down[0]))
    {
        std::cout << "Elbow-down solution: θ1=" << rad_to_deg(angles_down[0]) << "°, θ2=" << rad_to_deg(angles_down[1])
                  << "°\n";
    }

    // Test 3: Jacobian and velocity kinematics
    std::cout << "\n=== Jacobian & Velocity Kinematics ===\n";
    Mat<float, 2, 2> j = robot.jacobian(theta1, theta2);
    std::cout << "Jacobian at θ1=" << rad_to_deg(theta1) << "°, θ2=" << rad_to_deg(theta2) << "°:\n";
    std::cout << "  [" << j(0, 0) << "  " << j(0, 1) << "]\n";
    std::cout << "  [" << j(1, 0) << "  " << j(1, 1) << "]\n";

    // Joint velocities -> end effector velocity
    Vec<float, 2> joint_vel{deg_to_rad(10.0F), deg_to_rad(5.0F)};  // 10°/s, 5°/s
    Vec<float, 2> ee_vel = j * joint_vel;
    std::cout << "\nJoint velocities: " << rad_to_deg(joint_vel[0]) << "°/s, " << rad_to_deg(joint_vel[1]) << "°/s\n";
    std::cout << "End effector velocity: (" << ee_vel[0] << ", " << ee_vel[1] << ") m/s\n";

    // Test 4: Trajectory planning
    std::cout << "\n=== Trajectory Planning ===\n";
    Vec<float, 2> start{1.0F, 1.0F};
    Vec<float, 2> end{1.5F, 0.5F};
    std::cout << "Linear path from (" << start[0] << ", " << start[1] << ") to (" << end[0] << ", " << end[1] << "):\n";

    for (int i = 0; i <= 5; ++i)
    {
        float t = static_cast<float>(i) / 5.0F;
        Vec<float, 2> waypoint = start.lerp(end, t);
        Vec<float, 2> angles = robot.inverse_kinematics(waypoint, true);

        if (!std::isnan(angles[0]))
        {
            std::cout << "  t=" << t << ": pos(" << waypoint[0] << ", " << waypoint[1] << ") -> joints("
                      << rad_to_deg(angles[0]) << "°, " << rad_to_deg(angles[1]) << "°)\n";
        }
    }

    // Test 5: Workspace visualization
    robot.print_workspace();

    return 0;
}
