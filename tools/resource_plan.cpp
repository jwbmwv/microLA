// SPDX-License-Identifier: Apache-2.0

#include <microla/sensor_fusion.hpp>

#include <cstddef>
#include <iostream>

namespace
{

template<typename Type>
void write_type(const char* name, bool has_next)
{
    std::cout << "    \"" << name << "\": {\"bytes\": " << sizeof(Type) << ", \"alignment\": "
              << alignof(Type) << "}";
    if (has_next)
    {
        std::cout << ',';
    }
    std::cout << '\n';
}

}  // namespace

int main()
{
    using namespace microla;
    using namespace microla::fusion;

    using MahonyOrientation = OrientationEstimator<float, DefaultImu9MahonyConfig<float>>;
    using MekfOrientation = OrientationEstimator<float, DefaultImu9EkfConfig<float>>;
    using MahonyRelative = RelativeAngleEstimator<float,
                                                   DefaultImu9MahonyConfig<float>,
                                                   DefaultImu9MahonyConfig<float>,
                                                   DefaultRelativeAngleConfig<float>>;
    using MekfRelative = RelativeAngleEstimator<float,
                                                 DefaultImu9EkfConfig<float>,
                                                 DefaultImu9EkfConfig<float>,
                                                 DefaultRelativeAngleConfig<float>>;

    std::cout << "{\n";
    std::cout << "  \"schema_version\": 1,\n";
    std::cout << "  \"abi\": {\"pointer_bytes\": " << sizeof(void*) << ", \"size_t_bytes\": "
              << sizeof(std::size_t) << "},\n";
    std::cout << "  \"types\": {\n";
    write_type<Vec<float, 3>>("vec3f", true);
    write_type<Quaternion<float>>("quaternionf", true);
    write_type<SensorCalibration<float>>("sensor_calibration", true);
    write_type<OrientationEstimate<float>>("orientation_estimate", true);
    write_type<RelativeAngleResult<float>>("relative_angle_result", true);
    write_type<MahonyOrientation>("orientation_mahony_imu9", true);
    write_type<MekfOrientation>("orientation_mekf_imu9", true);
    write_type<MahonyRelative>("relative_mahony_imu9_pair", true);
    write_type<MekfRelative>("relative_mekf_imu9_pair", false);
    std::cout << "  }\n";
    std::cout << "}\n";
}
