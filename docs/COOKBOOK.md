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

## Extended Kalman Filtering

For nonlinear systems, use the Extended Kalman Filter (EKF) which linearizes dynamics via Jacobians. MicroLA provides a header-only EKF with function pointers for custom state transition and measurement models.

### Example 1: Constant Turn Rate and Velocity (CTRV) Model

Track a vehicle with nonlinear motion model (position + velocity + turn rate).

```cpp
#include <microla/extended_kalman.hpp>
#include <cmath>

using namespace microla;

// State: [x, y, vx, vy, omega] - position, velocity, turn rate
using State5 = Vec<float, 5>;
using Meas2 = Vec<float, 2>;
using Mat5 = Mat<float, 5, 5>;
using Mat2x5 = Mat<float, 2, 5>;

// Nonlinear state transition: x_k = f(x_k-1, dt)
auto ctrv_state_transition(const State5& x, float dt) -> State5
{
    float px = x[0], py = x[1], vx = x[2], vy = x[3], omega = x[4];
    
    State5 x_pred;
    // Position update with turn rate
    if (std::abs(omega) > 1e-4f)
    {
        float v = std::sqrt(vx * vx + vy * vy);
        float yaw = std::atan2(vy, vx);
        x_pred[0] = px + (v / omega) * (std::sin(yaw + omega * dt) - std::sin(yaw));
        x_pred[1] = py + (v / omega) * (-std::cos(yaw + omega * dt) + std::cos(yaw));
    }
    else
    {
        // Straight-line motion
        x_pred[0] = px + vx * dt;
        x_pred[1] = py + vy * dt;
    }
    
    // Velocity stays constant (constant velocity model)
    x_pred[2] = vx;
    x_pred[3] = vy;
    x_pred[4] = omega;
    
    return x_pred;
}

// Jacobian of state transition: F = ∂f/∂x
auto ctrv_state_jacobian(const State5& x, float dt) -> Mat5
{
    Mat5 F = Mat5::identity();
    
    float vx = x[2], vy = x[3], omega = x[4];
    float v = std::sqrt(vx * vx + vy * vy);
    float yaw = std::atan2(vy, vx);
    
    if (std::abs(omega) > 1e-4f)
    {
        // Partial derivatives for turn rate motion
        float sin_yaw = std::sin(yaw);
        float cos_yaw = std::cos(yaw);
        float sin_yaw_dt = std::sin(yaw + omega * dt);
        float cos_yaw_dt = std::cos(yaw + omega * dt);
        
        // ∂px/∂vx, ∂px/∂vy, ∂px/∂omega
        F(0, 2) = (vx / (v * omega)) * (sin_yaw_dt - sin_yaw);
        F(0, 3) = (vy / (v * omega)) * (sin_yaw_dt - sin_yaw);
        F(0, 4) = -(v / (omega * omega)) * (sin_yaw_dt - sin_yaw) 
                  + (v / omega) * cos_yaw_dt * dt;
        
        // ∂py/∂vx, ∂py/∂vy, ∂py/∂omega
        F(1, 2) = (vx / (v * omega)) * (-cos_yaw_dt + cos_yaw);
        F(1, 3) = (vy / (v * omega)) * (-cos_yaw_dt + cos_yaw);
        F(1, 4) = -(v / (omega * omega)) * (-cos_yaw_dt + cos_yaw) 
                  + (v / omega) * sin_yaw_dt * dt;
    }
    else
    {
        // Straight-line Jacobian
        F(0, 2) = dt;
        F(1, 3) = dt;
    }
    
    return F;
}

// Measurement model: measure only position [x, y]
auto ctrv_measurement(const State5& x) -> Meas2
{
    return Meas2{x[0], x[1]};
}

// Measurement Jacobian: H = ∂h/∂x
auto ctrv_measurement_jacobian(const State5& x) -> Mat2x5
{
    Mat2x5 H;
    H(0, 0) = 1.0f;  // ∂h_x/∂x
    H(1, 1) = 1.0f;  // ∂h_y/∂y
    // All other entries are zero (position doesn't depend on velocity or omega)
    return H;
}

void run_ctrv_filter()
{
    // Create EKF
    ExtendedKalmanFilter<float, 5, 2> ekf;
    
    // Set dynamics
    ekf.set_state_transition(ctrv_state_transition, ctrv_state_jacobian);
    ekf.set_measurement_model(ctrv_measurement, ctrv_measurement_jacobian);
    
    // Configure noise
    Mat5 Q = Mat5::identity() * 0.1f;  // Process noise
    Mat<float, 2, 2> R = Mat<float, 2, 2>::identity() * 0.5f;  // Measurement noise
    ekf.set_process_noise(Q);
    ekf.set_measurement_noise(R);
    
    // Initialize state
    State5 x0{0.0f, 0.0f, 5.0f, 0.0f, 0.1f};  // Starting at origin, moving with turn
    ekf.set_state(x0);
    
    // Filter loop
    float dt = 0.1f;
    for (int i = 0; i < 100; ++i)
    {
        // Predict
        ekf.predict(dt);
        
        // Update with measurement
        Meas2 z{/* GPS reading */};
        bool accepted = ekf.update_joseph(z);  // Use Joseph form for stability
        
        if (accepted)
        {
            State5 state = ekf.get_state();
            float pos_x = state[0];
            float pos_y = state[1];
            float heading = std::atan2(state[3], state[2]);
        }
    }
}
```

### Example 2: Range-Bearing Tracking (Radar/Sonar)

Track an object using nonlinear polar measurements (range and bearing angle).

```cpp
#include <microla/extended_kalman.hpp>
#include <cmath>

using namespace microla;

// State: [x, y, vx, vy] - 2D position and velocity
using State4 = Vec<float, 4>;
using Meas2 = Vec<float, 2>;
using Mat4 = Mat<float, 4, 4>;
using Mat2x4 = Mat<float, 2, 4>;

// Linear state transition (constant velocity)
auto radar_state_transition(const State4& x, float dt) -> State4
{
    State4 x_pred;
    x_pred[0] = x[0] + x[2] * dt;  // x + vx * dt
    x_pred[1] = x[1] + x[3] * dt;  // y + vy * dt
    x_pred[2] = x[2];               // vx (constant)
    x_pred[3] = x[3];               // vy (constant)
    return x_pred;
}

// Jacobian for linear dynamics
auto radar_state_jacobian(const State4& x, float dt) -> Mat4
{
    Mat4 F = Mat4::identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
}

// Nonlinear measurement: [range, bearing] from position
auto radar_measurement(const State4& x) -> Meas2
{
    float px = x[0], py = x[1];
    float range = std::sqrt(px * px + py * py);
    float bearing = std::atan2(py, px);
    return Meas2{range, bearing};
}

// Measurement Jacobian: H = ∂h/∂x
auto radar_measurement_jacobian(const State4& x) -> Mat2x4
{
    float px = x[0], py = x[1];
    float range = std::sqrt(px * px + py * py);
    float range_sq = px * px + py * py;
    
    Mat2x4 H;
    // ∂range/∂x, ∂range/∂y
    H(0, 0) = px / range;
    H(0, 1) = py / range;
    H(0, 2) = 0.0f;
    H(0, 3) = 0.0f;
    
    // ∂bearing/∂x, ∂bearing/∂y
    H(1, 0) = -py / range_sq;
    H(1, 1) = px / range_sq;
    H(1, 2) = 0.0f;
    H(1, 3) = 0.0f;
    
    return H;
}

void run_radar_tracking()
{
    ExtendedKalmanFilter<float, 4, 2> ekf;
    
    ekf.set_state_transition(radar_state_transition, radar_state_jacobian);
    ekf.set_measurement_model(radar_measurement, radar_measurement_jacobian);
    
    // Radar noise characteristics
    Mat4 Q = Mat4::identity() * 0.01f;  // Low process noise (nearly constant velocity)
    Mat<float, 2, 2> R;
    R(0, 0) = 1.0f;    // Range noise (meters)
    R(1, 1) = 0.01f;   // Bearing noise (radians)
    ekf.set_process_noise(Q);
    ekf.set_measurement_noise(R);
    
    // Initialize state
    State4 x0{100.0f, 100.0f, -5.0f, 3.0f};  // Start position + velocity
    ekf.set_state(x0);
    
    // Tracking loop
    float dt = 0.1f;
    for (int i = 0; i < 200; ++i)
    {
        ekf.predict(dt);
        
        // Radar measurement: [range, bearing]
        Meas2 z{/* range, bearing from sensor */};
        
        // Compute NIS for outlier detection
        float nis = ekf.compute_nis(z);
        if (nis < 10.0f)  // Chi-squared threshold for 2 DOF
        {
            ekf.update_joseph(z);
        }
        
        State4 state = ekf.get_state();
        float tracked_x = state[0];
        float tracked_y = state[1];
    }
}
```

### Example 3: Attitude Estimation with Quaternions

Track 3D orientation using gyroscope integration and accelerometer/magnetometer updates.

```cpp
#include <microla/extended_kalman.hpp>
#include <microla/quaternion.hpp>
#include <cmath>

using namespace microla;

// State: [qw, qx, qy, qz, wx, wy, wz] - quaternion + angular velocity
using State7 = Vec<float, 7>;
using Meas6 = Vec<float, 6>;  // Accelerometer (3) + Magnetometer (3)
using Mat7 = Mat<float, 7, 7>;
using Mat6x7 = Mat<float, 6, 7>;

// Quaternion integration from gyroscope
auto attitude_state_transition(const State7& x, float dt) -> State7
{
    Quaternion<float> q(x[0], x[1], x[2], x[3]);
    Vec<float, 3> omega{x[4], x[5], x[6]};
    
    // Integrate quaternion with angular velocity
    float omega_mag = omega.length();
    if (omega_mag > 1e-6f)
    {
        float half_angle = 0.5f * omega_mag * dt;
        Vec<float, 3> axis = omega.normalized();
        Quaternion<float> delta(
            std::cos(half_angle),
            axis[0] * std::sin(half_angle),
            axis[1] * std::sin(half_angle),
            axis[2] * std::sin(half_angle)
        );
        q = q * delta;
        q = q.normalized();
    }
    
    State7 x_pred;
    x_pred[0] = q.w();
    x_pred[1] = q.x();
    x_pred[2] = q.y();
    x_pred[3] = q.z();
    x_pred[4] = x[4];  // Angular velocity assumed constant
    x_pred[5] = x[5];
    x_pred[6] = x[6];
    
    return x_pred;
}

// Simplified attitude Jacobian
auto attitude_state_jacobian(const State7& x, float dt) -> Mat7
{
    Mat7 F = Mat7::identity();
    
    // Quaternion linearization around current state
    float wx = x[4], wy = x[5], wz = x[6];
    float dt2 = 0.5f * dt;
    
    // ∂q/∂ω (simplified)
    F(0, 4) = -dt2 * x[1];  // ∂qw/∂wx
    F(0, 5) = -dt2 * x[2];  // ∂qw/∂wy
    F(0, 6) = -dt2 * x[3];  // ∂qw/∂wz
    
    F(1, 4) = dt2 * x[0];   // ∂qx/∂wx
    F(2, 5) = dt2 * x[0];   // ∂qy/∂wy
    F(3, 6) = dt2 * x[0];   // ∂qz/∂wz
    
    return F;
}

// Measurement prediction: rotate reference vectors by quaternion
auto attitude_measurement(const State7& x) -> Meas6
{
    Quaternion<float> q(x[0], x[1], x[2], x[3]);
    
    // Expected accelerometer reading (gravity in body frame)
    Vec<float, 3> gravity_world{0.0f, 0.0f, 9.81f};
    Vec<float, 3> accel_pred = q.inverse().rotate(gravity_world);
    
    // Expected magnetometer reading (north in body frame)
    Vec<float, 3> mag_world{1.0f, 0.0f, 0.0f};
    Vec<float, 3> mag_pred = q.inverse().rotate(mag_world);
    
    Meas6 h;
    h[0] = accel_pred[0];
    h[1] = accel_pred[1];
    h[2] = accel_pred[2];
    h[3] = mag_pred[0];
    h[4] = mag_pred[1];
    h[5] = mag_pred[2];
    
    return h;
}

// Measurement Jacobian (numerical approximation or analytical)
auto attitude_measurement_jacobian(const State7& x) -> Mat6x7
{
    // For quaternions, numerical Jacobian is often more practical
    Mat6x7 H;
    float eps = 1e-5f;
    
    Meas6 h0 = attitude_measurement(x);
    
    for (std::size_t j = 0; j < 7; ++j)
    {
        State7 x_pert = x;
        x_pert[j] += eps;
        Meas6 h_pert = attitude_measurement(x_pert);
        
        for (std::size_t i = 0; i < 6; ++i)
        {
            H(i, j) = (h_pert[i] - h0[i]) / eps;
        }
    }
    
    return H;
}
```

### EKF Best Practices

1. **Always use Joseph form** for covariance updates (`update_joseph()` instead of `update()`) in production code - it's numerically more stable
2. **Validate measurements** with Normalized Innovation Squared (NIS) before updating
3. **Tune process noise Q** based on model uncertainty, not to compensate for poor dynamics
4. **Use numerical Jacobians** when analytical derivatives are complex or error-prone
5. **Normalize quaternions** after state transitions to prevent drift
6. **Check filter divergence** by monitoring innovation magnitude and covariance growth



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
**MicroLA Version**: 0.0.2
