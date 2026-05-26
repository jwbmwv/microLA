# MicroLA Cookbook

Practical recipes for the current MicroLA C++20 API.

This cookbook intentionally focuses on patterns backed by the shipped headers and example programs. Application-specific camera models, particle filters, and other domain code are better kept in your project unless they are promoted into maintained examples.

## Table of Contents

1. [Embedded C++20 Checklist](#embedded-c20-checklist)
2. [IMU Sensor Fusion](#imu-sensor-fusion)
3. [Planar Robot Kinematics](#planar-robot-kinematics)
4. [Kalman Filtering](#kalman-filtering)
5. [4x4 Transform Assembly](#4x4-transform-assembly)
6. [Collision Detection](#collision-detection)
7. [Interpolation](#interpolation)
8. [Allocation-Sensitive Matrix Diagnostics](#allocation-sensitive-matrix-diagnostics)

---

## Embedded C++20 Checklist

For allocation-sensitive or embedded builds, prefer these patterns:

- Use fixed-size `Vec`, `Mat`, and `Quaternion` types with dimensions known at compile time.
- Prefer non-throwing APIs such as `try_inverse()` and `inverse_checked()`.
- Prefer caller-provided buffers for algorithms that offer them, such as `eigenvalues_qr(buffer, size)`.
- Precompute constant transforms once and reuse them instead of rebuilding them in hot loops.
- Keep application-level estimators and control logic outside the library unless they are generic enough to become maintained examples.

---

## IMU Sensor Fusion

The maintained sensor-fusion example now uses `microla::fusion::RelativeAngleEstimator` directly. That keeps the example aligned with the shipped public API instead of a hand-rolled complementary filter.

```cpp
#include <microla/sensor_fusion.hpp>

using namespace microla;
using namespace microla::fusion;

struct AnchorConfig : DefaultImu9MahonyConfig<float>
{
};

struct FollowerConfig : DefaultImu6MahonyConfig<float>
{
    static constexpr float drift_rate_without_heading_rad_per_s = 0.03F;
};

struct JointConfig : DefaultRelativeAngleConfig<float>
{
    static constexpr PrimaryScalarOutput default_output = PrimaryScalarOutput::hinge_twist;
    static constexpr bool apply_reference_pose = true;
};

using JointEstimator = RelativeAngleEstimator<float, AnchorConfig, FollowerConfig, JointConfig>;

JointEstimator estimator;
estimator.update_left(anchor_sample);
estimator.update_right(follower_sample);
estimator.capture_reference_pose();

auto hinge = estimator.compute();
auto tilt = estimator.compute_scalar(PrimaryScalarOutput::tilt_angle);
```

Use this pattern when the application cares about the angle between two entities rather than only one body-frame orientation. See [docs/SENSOR_FUSION.md](SENSOR_FUSION.md) for the complete set of tuning knobs and observability rules.

---

## Planar Robot Kinematics

The maintained robotics example models a 2-link planar arm. This keeps the math fixed-size and C++20-friendly while still covering forward kinematics, inverse kinematics, and the Jacobian.

```cpp
#include <microla/microla.hpp>

#include <cmath>

using namespace microla;

using Vec2f = Vec<float, 2>;
using Mat2f = Mat<float, 2, 2>;

class PlanarRobotArm
{
private:
    float link1_length;
    float link2_length;

public:
    PlanarRobotArm(float l1, float l2) : link1_length(l1), link2_length(l2) {}

    [[nodiscard]] auto forward_kinematics(float theta1, float theta2) const -> Vec2f
    {
        float x1 = link1_length * std::cos(theta1);
        float y1 = link1_length * std::sin(theta1);
        float x2 = x1 + link2_length * std::cos(theta1 + theta2);
        float y2 = y1 + link2_length * std::sin(theta1 + theta2);
        return Vec2f{x2, y2};
    }

    [[nodiscard]] auto inverse_kinematics(const Vec2f& target, bool elbow_up = true) const -> Vec2f
    {
        float x = target[0];
        float y = target[1];
        float r = std::sqrt(x * x + y * y);

        if (r > link1_length + link2_length || r < std::abs(link1_length - link2_length))
        {
            return Vec2f{NAN, NAN};
        }

        float cos_theta2 =
            (r * r - link1_length * link1_length - link2_length * link2_length) / (2.0F * link1_length * link2_length);
        cos_theta2 = clamp(cos_theta2, -1.0F, 1.0F);

        float theta2 = elbow_up ? std::acos(cos_theta2) : -std::acos(cos_theta2);
        float k1 = link1_length + link2_length * std::cos(theta2);
        float k2 = link2_length * std::sin(theta2);
        float theta1 = std::atan2(y, x) - std::atan2(k2, k1);

        return Vec2f{theta1, theta2};
    }

    [[nodiscard]] auto jacobian(float theta1, float theta2) const -> Mat2f
    {
        float s1 = std::sin(theta1);
        float c1 = std::cos(theta1);
        float s12 = std::sin(theta1 + theta2);
        float c12 = std::cos(theta1 + theta2);

        Mat2f j;
        j(0, 0) = -link1_length * s1 - link2_length * s12;
        j(0, 1) = -link2_length * s12;
        j(1, 0) = link1_length * c1 + link2_length * c12;
        j(1, 1) = link2_length * c12;
        return j;
    }
};
```

---

## Kalman Filtering

MicroLA ships a fixed-size linear Kalman filter with Joseph-form covariance updates. Keep the matrices fixed-size and configure them once.

```cpp
#include <microla/kalman.hpp>

using namespace microla;

using Vec1f = Vec<float, 1>;
using Vec2f = Vec<float, 2>;
using Mat2f = Mat<float, 2, 2>;
using Mat1x2f = Mat<float, 1, 2>;

KalmanFilter<float, 2, 1> kf;

float dt = 0.1F;
Mat2f F = Mat2f::identity();
F(0, 1) = dt;
kf.set_state_transition(F);

Mat1x2f H;
H(0, 0) = 1.0F;
H(0, 1) = 0.0F;
kf.set_measurement_matrix(H);

kf.set_process_noise(Mat2f::identity() * 0.01F);

Mat<float, 1, 1> R;
R(0, 0) = 0.25F;
kf.set_measurement_noise(R);

kf.set_state(Vec2f{0.0F, 1.0F});

kf.predict();

Vec1f measurement{5.2F};
bool accepted = kf.update(measurement);
float nis = kf.compute_nis(measurement);
Vec2f state = kf.get_state();
```

---

## 4x4 Transform Assembly

The graphics example builds 4x4 transforms from fixed-size 3x3 rotation blocks plus translation terms. This is often the simplest C++20 path when you want predictable code generation.

```cpp
#include <microla/microla.hpp>

using namespace microla;

using Vec3f = Vec<float, 3>;
using Vec4f = Vec<float, 4>;
using Mat3f = Mat<float, 3, 3>;
using Mat4f = Mat<float, 4, 4>;

Mat3f rot3 = Mat3f::rotation_y(deg_to_rad(45.0F));
Mat4f world = Mat4f::identity();

for (std::size_t row = 0; row < 3; ++row)
{
    for (std::size_t col = 0; col < 3; ++col)
    {
        world(row, col) = rot3(row, col);
    }
}

world(0, 3) = 2.0F;
world(1, 3) = 1.0F;
world(2, 3) = -5.0F;

Vec4f point_h{1.0F, 1.0F, 0.0F, 1.0F};
Vec4f transformed = world * point_h;
Vec3f point_world{transformed[0], transformed[1], transformed[2]};
```

---

## Collision Detection

Use the geometry types in `microla::geometry` directly instead of introducing duplicate application structs.

```cpp
#include <microla/geometry.hpp>

using namespace microla;

using Vec3f = Vec<float, 3>;

geometry::AABB<float> box({-1.0F, -1.0F, -1.0F}, {1.0F, 1.0F, 1.0F});
auto grown = geometry::AABB<float>::from_center_extents(Vec3f{0.0F, 0.0F, 0.0F}, Vec3f{1.5F, 1.5F, 1.5F});

bool overlaps = box.intersects(grown);
bool inside = box.contains(Vec3f{0.5F, 0.5F, 0.5F});
float surface_area = box.surface_area();
float volume = box.volume();

geometry::Sphere<float> sphere(Vec3f{0.0F, 0.0F, 0.0F}, 1.0F);
bool sphere_hits_box = sphere.intersects(box);

geometry::Ray<float> ray(Vec3f{0.0F, 0.0F, 5.0F}, Vec3f{0.0F, 0.0F, -1.0F});
auto hit = box.intersect(ray);
if (hit.has_value())
{
    Vec3f entry = ray.at(hit->first);
    Vec3f exit = ray.at(hit->second);
}
```

---

## Interpolation

MicroLA already provides member functions for common spline-style interpolation on fixed-size vectors.

```cpp
#include <microla/vector.hpp>

using namespace microla;

using Vec3f = Vec<float, 3>;

Vec3f p0{-1.0F, 0.0F, 0.0F};
Vec3f p1{0.0F, 0.0F, 0.0F};
Vec3f p2{1.0F, 1.0F, 0.0F};
Vec3f p3{2.0F, 1.0F, 0.0F};

Vec3f t1{1.0F, 0.0F, 0.0F};
Vec3f t2{1.0F, 1.0F, 0.0F};

Vec3f hermite = p1.cubic_hermite(p2, t1, t2, 0.5F);
Vec3f catmull = p1.catmull_rom(p2, p0, p3, 0.5F);
Vec3f linear = p1.lerp(p2, 0.5F);
```

---

## Allocation-Sensitive Matrix Diagnostics

The library exposes convenience wrappers that return `std::optional`, `std::variant`, or `std::vector`, but embedded code can stay on the caller-owned side of the API.

```cpp
#include <microla/matrix.hpp>

#include <variant>

using namespace microla;

using Mat3f = Mat<float, 3, 3>;

Mat3f A{
    4.0F, 1.0F, 0.0F,
    1.0F, 3.0F, 0.0F,
    0.0F, 0.0F, 2.0F
};

auto inv = A.inverse_checked();
if (std::holds_alternative<Mat3f>(inv))
{
    Mat3f inv_a = std::get<Mat3f>(inv);
}

float eigenvalues[3] = {};
bool converged = A.eigenvalues_qr(eigenvalues, 3);
if (converged)
{
    float lambda0 = eigenvalues[0];
    float lambda1 = eigenvalues[1];
    float lambda2 = eigenvalues[2];
}

auto maybe_inverse = A.try_inverse();
if (maybe_inverse.has_value())
{
    Mat3f inv_a = *maybe_inverse;
}
```

---

## Additional Resources

- **[API_Documentation.md](API_Documentation.md)** - Complete API reference
- **[PERFORMANCE.md](../PERFORMANCE.md)** - Performance benchmarks
- **[examples/](../examples/)** - Maintained example programs
- **[QUICK_REFERENCE.md](../QUICK_REFERENCE.md)** - Quick syntax guide

## Contributing Recipes

If you add a new cookbook entry, keep it tied to the current public API:

- Make the snippet compile against the shipped headers.
- Prefer fixed-size types and non-allocating overloads when an embedded-friendly option exists.
- Keep domain-specific layers separate from the library unless they are promoted into maintained examples.
- Link to the example or test file that exercises the same pattern when possible.

---

**Date**: March 6, 2026
**MicroLA Version**: 0.0.1
