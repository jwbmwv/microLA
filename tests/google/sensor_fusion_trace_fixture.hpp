// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

#include <microla/sensor_fusion.hpp>

namespace microla::test_fixture
{

struct Imu9TraceFrame
{
    float timestamp_s;
    Vec<float, 3> gyro_rad_s;
    Vec<float, 3> accel_m_s2;
    Vec<float, 3> magnetic_field;
    fusion::StatusFlag required_primary_flag;
    fusion::StatusFlag required_secondary_flag;
};

inline const std::array<Imu9TraceFrame, 6> imu9_ekf_lifecycle_trace{{
    {0.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
     Vec<float, 3>(50.0F, 0.0F, 0.0F), fusion::StatusFlag::none, fusion::StatusFlag::none},
    {0.1F, Vec<float, 3>(0.0F, 0.0F, 1.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
     Vec<float, 3>(50.0F, 0.0F, 0.0F), fusion::StatusFlag::none, fusion::StatusFlag::none},
    {0.2F, Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
     Vec<float, 3>(0.0F, -50.0F, 0.0F), fusion::StatusFlag::mag_rejected, fusion::StatusFlag::propagation_only},
    {0.3F, Vec<float, 3>(100.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>()),
     Vec<float, 3>(50.0F, 0.0F, 0.0F), fusion::StatusFlag::gyro_rejected, fusion::StatusFlag::high_rotation_rate},
    {0.4F, Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(50.0F, 0.0F, 0.0F),
     fusion::StatusFlag::freefall_detected, fusion::StatusFlag::accel_rejected},
    {0.5F, Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, -50.0F), Vec<float, 3>(50.0F, 0.0F, 0.0F),
     fusion::StatusFlag::high_linear_acceleration, fusion::StatusFlag::accel_rejected},
}};

struct Imu6TraceFrame
{
    float timestamp_s;
    Vec<float, 3> gyro_rad_s;
    Vec<float, 3> accel_m_s2;
};

inline const std::array<Imu6TraceFrame, 3> imu6_timing_trace{{
    {1.0F, Vec<float, 3>(0.0F, 0.0F, 0.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())},
    {0.5F, Vec<float, 3>(0.0F, 0.0F, 1.0F), Vec<float, 3>(0.0F, -constants::gravity<float>(), 0.0F)},
    {0.01F, Vec<float, 3>(0.0F, 0.0F, 1.0F), Vec<float, 3>(0.0F, 0.0F, -constants::gravity<float>())},
}};

}  // namespace microla::test_fixture