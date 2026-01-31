# MicroLA Cookbook

Practical patterns and recipes for common linear algebra tasks in embedded systems and real-time applications.

## Table of Contents

1. [IMU Sensor Fusion](#imu-sensor-fusion)
2. [Camera Calibration & Transforms](#camera-calibration--transforms)
3. [Robot Forward/Inverse Kinematics](#robot-forwardinverse-kinematics)
4. [Particle Filter State Estimation](#particle-filter-state-estimation)
5. [3D Graphics Pipeline](#3d-graphics-pipeline)
6. [Kalman Filter Implementation](#kalman-filter-implementation)
7. [Orientation Tracking](#orientation-tracking)
8. [Coordinate Frame Transformations](#coordinate-frame-transformations)
9. [Collision Detection](#collision-detection)
10. [Path Planning & Interpolation](#path-planning--interpolation)

---

## IMU Sensor Fusion

### Complementary Filter for Attitude Estimation

Fuse accelerometer and gyroscope data to estimate orientation:

```cpp
#include <microla/microla.hpp>
#include <microla/quaternion.hpp>

using namespace microla;

class ComplementaryFilter {
private:
    Quaternion<float> orientation;
    float alpha = 0.98f;  // Gyro weight (0.98 = 98% gyro, 2% accel)
    
public:
    ComplementaryFilter() : orientation(Quaternion<float>::identity()) {}
    
    // Update at fixed rate (e.g., 100 Hz = 0.01s)
    void update(const Vec3f& accel, const Vec3f& gyro, float dt) {
        // 1. Integrate gyroscope (dead reckoning)
        Vec3f gyro_rad = gyro * constants::deg2rad<float>;
        Quaternion<float> gyro_delta = Quaternion<float>::from_axis_angle(
            gyro_rad.normalized(), gyro_rad.length() * dt
        );
        Quaternion<float> gyro_orientation = orientation * gyro_delta;
        
        // 2. Compute accelerometer-based orientation (assumes static or low dynamics)
        Vec3f accel_norm = accel.normalized();
        Vec3f gravity(0.0f, 0.0f, -1.0f);  // Gravity in world frame
        
        // Find rotation from gravity to measured acceleration
        Vec3f axis = gravity.cross(accel_norm);
        float angle = std::acos(std::clamp(gravity.dot(accel_norm), -1.0f, 1.0f));
        
        Quaternion<float> accel_orientation = Quaternion<float>::identity();
        if (axis.length() > 1e-6f) {
            accel_orientation = Quaternion<float>::from_axis_angle(axis.normalized(), angle);
        }
        
        // 3. Complementary fusion
        orientation = Quaternion<float>::slerp(accel_orientation, gyro_orientation, alpha);
        orientation = orientation.normalized();
    }
    
    Quaternion<float> get_orientation() const { return orientation; }
    Vec3f get_euler_angles() const { return orientation.to_euler(); }
};

// Usage
ComplementaryFilter filter;
Vec3f accel(0.1f, 0.2f, -9.8f);  // m/s²
Vec3f gyro(0.5f, -0.3f, 0.1f);    // deg/s
float dt = 0.01f;                  // 100 Hz

filter.update(accel, gyro, dt);
Vec3f euler = filter.get_euler_angles();  // Roll, Pitch, Yaw
```

### Madgwick Filter (AHRS)

More sophisticated 9-DOF fusion with magnetometer:

```cpp
class MadgwickFilter {
private:
    Quaternion<float> q;
    float beta = 0.1f;  // Convergence rate
    
public:
    MadgwickFilter() : q(Quaternion<float>::identity()) {}
    
    void update(const Vec3f& accel, const Vec3f& gyro, const Vec3f& mag, float dt) {
        // Normalize measurements
        Vec3f a = accel.normalized();
        Vec3f m = mag.normalized();
        Vec3f w = gyro * constants::deg2rad<float>;
        
        // Compute gradient (objective function from accelerometer and magnetometer)
        Vec3f v(2.0f * (q.x() * q.z() - q.w() * q.y()),
                2.0f * (q.w() * q.x() + q.y() * q.z()),
                q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
        
        Vec3f error = v.cross(a);
        
        // Gradient descent step
        Vec3f q_dot_w = (q * Quaternion<float>(w.x(), w.y(), w.z(), 0.0f)).imaginary() * 0.5f;
        Vec3f q_dot_err = error * (-beta);
        
        // Integrate quaternion derivative
        Quaternion<float> q_dot(q_dot_w.x() + q_dot_err.x(),
                                 q_dot_w.y() + q_dot_err.y(),
                                 q_dot_w.z() + q_dot_err.z(),
                                 -q.x() * w.x() - q.y() * w.y() - q.z() * w.z());
        
        q = q + q_dot * dt;
        q = q.normalized();
    }
    
    Quaternion<float> get_orientation() const { return q; }
};
```

---

## Camera Calibration & Transforms

### Pinhole Camera Model

```cpp
struct CameraIntrinsics {
    float fx, fy;  // Focal lengths
    float cx, cy;  // Principal point
    
    // Project 3D point to 2D image coordinates
    Vec2f project(const Vec3f& point_3d) const {
        if (std::abs(point_3d.z()) < 1e-6f) {
            return Vec2f(cx, cy);  // Point at camera origin
        }
        float x = point_3d.x() / point_3d.z();
        float y = point_3d.y() / point_3d.z();
        return Vec2f(fx * x + cx, fy * y + cy);
    }
    
    // Back-project 2D pixel + depth to 3D
    Vec3f unproject(const Vec2f& pixel, float depth) const {
        float x = (pixel.x() - cx) * depth / fx;
        float y = (pixel.y() - cy) * depth / fy;
        return Vec3f(x, y, depth);
    }
};

// Extrinsic parameters (camera pose)
struct CameraPose {
    Quaternion<float> rotation;
    Vec3f translation;
    
    // Transform point from world to camera frame
    Vec3f world_to_camera(const Vec3f& world_point) const {
        return rotation.rotate(world_point - translation);
    }
    
    // Transform point from camera to world frame
    Vec3f camera_to_world(const Vec3f& camera_point) const {
        return rotation.conjugate().rotate(camera_point) + translation;
    }
};

// Full camera model
class Camera {
    CameraIntrinsics intrinsics;
    CameraPose pose;
    
public:
    Camera(float fx, float fy, float cx, float cy)
        : intrinsics{fx, fy, cx, cy}, pose{Quaternion<float>::identity(), Vec3f()} {}
    
    // Project world point to pixel
    Vec2f world_to_pixel(const Vec3f& world_point) const {
        Vec3f camera_point = pose.world_to_camera(world_point);
        return intrinsics.project(camera_point);
    }
    
    // Set camera pose
    void set_pose(const Quaternion<float>& rot, const Vec3f& pos) {
        pose.rotation = rot;
        pose.translation = pos;
    }
};

// Usage
Camera cam(800.0f, 800.0f, 320.0f, 240.0f);  // 640×480 image
cam.set_pose(Quaternion<float>::identity(), Vec3f(0, 0, 0));
Vec3f world_point(1.0f, 2.0f, 5.0f);
Vec2f pixel = cam.world_to_pixel(world_point);
```

### Stereo Vision

```cpp
struct StereoCamera {
    Camera left, right;
    float baseline;  // Distance between cameras
    
    // Compute depth from disparity
    float disparity_to_depth(float disparity) const {
        if (disparity < 1e-6f) return std::numeric_limits<float>::infinity();
        return (left.intrinsics.fx * baseline) / disparity;
    }
    
    // Triangulate 3D point from stereo pair
    Vec3f triangulate(const Vec2f& left_pixel, const Vec2f& right_pixel) const {
        float disparity = left_pixel.x() - right_pixel.x();
        float depth = disparity_to_depth(disparity);
        return left.intrinsics.unproject(left_pixel, depth);
    }
};
```

---

## Robot Forward/Inverse Kinematics

### 6-DOF Manipulator (Denavit-Hartenberg Parameters)

```cpp
struct DHParameter {
    float a;      // Link length
    float alpha;  // Link twist
    float d;      // Link offset
    float theta;  // Joint angle
};

class Robot6DOF {
private:
    std::array<DHParameter, 6> dh_params;
    
    // Compute transformation matrix from DH parameters
    Mat4f dh_transform(const DHParameter& dh) const {
        float ct = std::cos(dh.theta);
        float st = std::sin(dh.theta);
        float ca = std::cos(dh.alpha);
        float sa = std::sin(dh.alpha);
        
        return Mat4f({
            ct,    -st * ca,  st * sa,   dh.a * ct,
            st,     ct * ca, -ct * sa,   dh.a * st,
            0.0f,   sa,       ca,        dh.d,
            0.0f,   0.0f,     0.0f,      1.0f
        });
    }
    
public:
    Robot6DOF(const std::array<DHParameter, 6>& params) : dh_params(params) {}
    
    // Forward kinematics: joint angles → end-effector pose
    Mat4f forward_kinematics(const std::array<float, 6>& joint_angles) {
        Mat4f transform = Mat4f::identity();
        
        for (size_t i = 0; i < 6; ++i) {
            DHParameter dh = dh_params[i];
            dh.theta += joint_angles[i];  // Add joint angle to base theta
            transform = transform * dh_transform(dh);
        }
        
        return transform;
    }
    
    // Extract position and orientation from transform
    Vec3f get_position(const Mat4f& transform) const {
        return Vec3f(transform(0, 3), transform(1, 3), transform(2, 3));
    }
    
    Quaternion<float> get_orientation(const Mat4f& transform) const {
        Mat3f rotation;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation(i, j) = transform(i, j);
            }
        }
        return Quaternion<float>::from_rotation_matrix(rotation);
    }
};

// Usage
std::array<DHParameter, 6> robot_params = {{
    {0.0f, constants::pi<float> / 2.0f, 0.3f, 0.0f},
    {0.5f, 0.0f, 0.0f, 0.0f},
    {0.5f, 0.0f, 0.0f, 0.0f},
    {0.0f, constants::pi<float> / 2.0f, 0.3f, 0.0f},
    {0.0f, -constants::pi<float> / 2.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.1f, 0.0f}
}};

Robot6DOF robot(robot_params);
std::array<float, 6> joints = {0.0f, 0.5f, -0.5f, 0.0f, 0.0f, 0.0f};
Mat4f end_effector = robot.forward_kinematics(joints);
Vec3f position = robot.get_position(end_effector);
```

### Simple Inverse Kinematics (2-Link Planar Arm)

```cpp
struct TwoLinkIK {
    float L1, L2;  // Link lengths
    
    // Solve for joint angles given end-effector position
    bool solve(const Vec2f& target, float& theta1, float& theta2) const {
        float x = target.x();
        float y = target.y();
        float r = std::sqrt(x * x + y * y);
        
        // Check if target is reachable
        if (r > L1 + L2 || r < std::abs(L1 - L2)) {
            return false;  // Out of reach
        }
        
        // Law of cosines
        float cos_theta2 = (r * r - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
        theta2 = std::acos(std::clamp(cos_theta2, -1.0f, 1.0f));
        
        // Solve for theta1
        float k1 = L1 + L2 * std::cos(theta2);
        float k2 = L2 * std::sin(theta2);
        theta1 = std::atan2(y, x) - std::atan2(k2, k1);
        
        return true;
    }
};

// Usage
TwoLinkIK arm{0.5f, 0.4f};
float theta1, theta2;
if (arm.solve(Vec2f(0.7f, 0.3f), theta1, theta2)) {
    // Solution found
}
```

---

## Particle Filter State Estimation

```cpp
#include <random>

template<int STATE_DIM>
struct Particle {
    Vec<float, STATE_DIM> state;
    float weight;
};

template<int STATE_DIM, int NUM_PARTICLES>
class ParticleFilter {
private:
    std::array<Particle<STATE_DIM>, NUM_PARTICLES> particles;
    std::mt19937 rng;
    
public:
    ParticleFilter() : rng(std::random_device{}()) {
        // Initialize particles uniformly
        for (auto& p : particles) {
            p.weight = 1.0f / NUM_PARTICLES;
        }
    }
    
    // Prediction step (motion model)
    void predict(const Vec<float, STATE_DIM>& control, float noise_std) {
        std::normal_distribution<float> noise(0.0f, noise_std);
        
        for (auto& p : particles) {
            // Apply motion model with noise
            p.state = p.state + control;
            for (int i = 0; i < STATE_DIM; ++i) {
                p.state[i] += noise(rng);
            }
        }
    }
    
    // Update step (measurement model)
    void update(const Vec<float, STATE_DIM>& measurement, float sensor_std) {
        float weight_sum = 0.0f;
        
        // Compute likelihood of each particle
        for (auto& p : particles) {
            Vec<float, STATE_DIM> diff = p.state - measurement;
            float dist_sq = diff.dot(diff);
            p.weight *= std::exp(-dist_sq / (2.0f * sensor_std * sensor_std));
            weight_sum += p.weight;
        }
        
        // Normalize weights
        for (auto& p : particles) {
            p.weight /= weight_sum;
        }
        
        // Resample if effective sample size is low
        float eff_sample_size = 1.0f / compute_weight_variance();
        if (eff_sample_size < NUM_PARTICLES / 2.0f) {
            resample();
        }
    }
    
    // Estimate current state (weighted mean)
    Vec<float, STATE_DIM> estimate() const {
        Vec<float, STATE_DIM> mean;
        for (const auto& p : particles) {
            mean = mean + p.state * p.weight;
        }
        return mean;
    }
    
private:
    float compute_weight_variance() const {
        float sum_sq = 0.0f;
        for (const auto& p : particles) {
            sum_sq += p.weight * p.weight;
        }
        return sum_sq;
    }
    
    void resample() {
        std::array<Particle<STATE_DIM>, NUM_PARTICLES> new_particles;
        std::discrete_distribution<> dist(
            particles.begin(), particles.end(),
            [](const auto& p) { return p.weight; }
        );
        
        for (auto& np : new_particles) {
            np = particles[dist(rng)];
            np.weight = 1.0f / NUM_PARTICLES;
        }
        
        particles = new_particles;
    }
};

// Usage: 2D position tracking
ParticleFilter<2, 1000> pf;
pf.predict(Vec2f(1.0f, 0.5f), 0.1f);           // Move with noise
pf.update(Vec2f(5.2f, 3.1f), 0.5f);            // GPS measurement
Vec2f position = pf.estimate();
```

---

## 3D Graphics Pipeline

### Vertex Transformation Pipeline

```cpp
class GraphicsPipeline {
private:
    Mat4f model, view, projection;
    Mat4f mvp;  // Cached model-view-projection
    
public:
    void set_model(const Mat4f& m) {
        model = m;
        mvp = projection * view * model;
    }
    
    void set_view(const Vec3f& eye, const Vec3f& target, const Vec3f& up) {
        Vec3f z = (eye - target).normalized();
        Vec3f x = up.cross(z).normalized();
        Vec3f y = z.cross(x);
        
        view = Mat4f({
            x.x(), x.y(), x.z(), -x.dot(eye),
            y.x(), y.y(), y.z(), -y.dot(eye),
            z.x(), z.y(), z.z(), -z.dot(eye),
            0.0f,  0.0f,  0.0f,  1.0f
        });
        mvp = projection * view * model;
    }
    
    void set_perspective(float fov_y, float aspect, float near, float far) {
        float tan_half_fov = std::tan(fov_y * 0.5f);
        
        projection = Mat4f({
            1.0f / (aspect * tan_half_fov), 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f / tan_half_fov, 0.0f, 0.0f,
            0.0f, 0.0f, -(far + near) / (far - near), -2.0f * far * near / (far - near),
            0.0f, 0.0f, -1.0f, 0.0f
        });
        mvp = projection * view * model;
    }
    
    // Transform vertex from model space to clip space
    Vec4f transform_vertex(const Vec3f& vertex) const {
        Vec4f v(vertex.x(), vertex.y(), vertex.z(), 1.0f);
        return mvp * v;
    }
    
    // Perspective divide (clip space → NDC)
    Vec3f to_ndc(const Vec4f& clip) const {
        if (std::abs(clip.w()) < 1e-6f) return Vec3f(0, 0, 0);
        return Vec3f(clip.x() / clip.w(), clip.y() / clip.w(), clip.z() / clip.w());
    }
    
    // Transform normal (use inverse transpose of model matrix)
    Vec3f transform_normal(const Vec3f& normal) const {
        Mat3f normal_matrix = model.to_mat3().inverse().transpose();
        return (normal_matrix * normal).normalized();
    }
};

// Usage
GraphicsPipeline pipeline;
pipeline.set_model(Mat4f::translation(Vec3f(0, 0, -5)));
pipeline.set_view(Vec3f(0, 2, 5), Vec3f(0, 0, 0), Vec3f(0, 1, 0));
pipeline.set_perspective(60.0f * constants::deg2rad<float>, 16.0f / 9.0f, 0.1f, 100.0f);

Vec3f vertex(1.0f, 1.0f, 0.0f);
Vec4f clip = pipeline.transform_vertex(vertex);
Vec3f ndc = pipeline.to_ndc(clip);
```

### Lighting Calculations

```cpp
struct Light {
    Vec3f position;
    Vec3f color;
    float intensity;
};

// Phong shading
Vec3f phong_lighting(const Vec3f& vertex_pos, const Vec3f& normal,
                     const Vec3f& view_pos, const Light& light,
                     const Vec3f& material_color, float shininess) {
    // Ambient
    Vec3f ambient = material_color * 0.1f;
    
    // Diffuse
    Vec3f light_dir = (light.position - vertex_pos).normalized();
    float diff = std::max(normal.dot(light_dir), 0.0f);
    Vec3f diffuse = material_color * light.color * diff * light.intensity;
    
    // Specular
    Vec3f view_dir = (view_pos - vertex_pos).normalized();
    Vec3f reflect_dir = normal * (2.0f * normal.dot(light_dir)) - light_dir;
    float spec = std::pow(std::max(view_dir.dot(reflect_dir), 0.0f), shininess);
    Vec3f specular = light.color * spec * light.intensity;
    
    return ambient + diffuse + specular;
}
```

---

## Kalman Filter Implementation

### Standard Kalman Filter

```cpp
template<int STATE_DIM, int MEAS_DIM>
class KalmanFilter {
private:
    Vec<float, STATE_DIM> x;                          // State estimate
    SquareMat<float, STATE_DIM> P;                    // Covariance estimate
    SquareMat<float, STATE_DIM> F;                    // State transition
    Mat<float, MEAS_DIM, STATE_DIM> H;                // Measurement matrix
    SquareMat<float, STATE_DIM> Q;                    // Process noise covariance
    SquareMat<float, MEAS_DIM> R;                     // Measurement noise covariance
    
public:
    KalmanFilter() {
        P = SquareMat<float, STATE_DIM>::identity();
        F = SquareMat<float, STATE_DIM>::identity();
        Q = SquareMat<float, STATE_DIM>::identity() * 0.01f;
        R = SquareMat<float, MEAS_DIM>::identity() * 0.1f;
    }
    
    // Prediction step
    void predict() {
        // x = F * x (no control input)
        x = F * x;
        
        // P = F * P * F^T + Q
        P = F * P * F.transpose() + Q;
    }
    
    // Update step with measurement
    void update(const Vec<float, MEAS_DIM>& z) {
        // Innovation (measurement residual)
        Vec<float, MEAS_DIM> y = z - H * x;
        
        // Innovation covariance
        Mat<float, MEAS_DIM, MEAS_DIM> S = H * P * H.transpose() + R;
        
        // Kalman gain
        Mat<float, STATE_DIM, MEAS_DIM> K = P * H.transpose() * S.inverse();
        
        // Update state estimate
        x = x + K * y;
        
        // Update covariance (Joseph form for numerical stability)
        SquareMat<float, STATE_DIM> I = SquareMat<float, STATE_DIM>::identity();
        SquareMat<float, STATE_DIM> IKH = I - K * H;
        P = IKH * P * IKH.transpose() + K * R * K.transpose();
    }
    
    Vec<float, STATE_DIM> get_state() const { return x; }
    SquareMat<float, STATE_DIM> get_covariance() const { return P; }
    
    void set_state_transition(const SquareMat<float, STATE_DIM>& F_new) { F = F_new; }
    void set_measurement_matrix(const Mat<float, MEAS_DIM, STATE_DIM>& H_new) { H = H_new; }
};

// Usage: 1D position + velocity tracking
KalmanFilter<2, 1> kf;  // 2 states (pos, vel), 1 measurement (pos)

// Set up state transition (position += velocity * dt)
float dt = 0.1f;
Mat2f F({1.0f, dt, 0.0f, 1.0f});
kf.set_state_transition(F);

// Measurement matrix (measure position only)
Mat<float, 1, 2> H({1.0f, 0.0f});
kf.set_measurement_matrix(H);

// Run filter
kf.predict();
kf.update(Vec<float, 1>(5.2f));  // Position measurement
Vec2f state = kf.get_state();    // [position, velocity]
```

---

## Orientation Tracking

### Gyroscope Integration with Drift Compensation

```cpp
class OrientationTracker {
private:
    Quaternion<float> orientation;
    Vec3f gyro_bias;
    float bias_alpha = 0.001f;  // Bias estimation rate
    
public:
    OrientationTracker() : orientation(Quaternion<float>::identity()), gyro_bias() {}
    
    void update(const Vec3f& gyro, const Vec3f& accel, float dt) {
        // Estimate gyro bias during static periods
        if (accel.length() > 9.0f && accel.length() < 10.5f) {  // Near 1g
            gyro_bias = gyro_bias * (1.0f - bias_alpha) + gyro * bias_alpha;
        }
        
        // Remove bias
        Vec3f gyro_corrected = (gyro - gyro_bias) * constants::deg2rad<float>;
        
        // Integrate angular velocity
        float angle = gyro_corrected.length() * dt;
        if (angle > 1e-6f) {
            Vec3f axis = gyro_corrected.normalized();
            Quaternion<float> delta = Quaternion<float>::from_axis_angle(axis, angle);
            orientation = orientation * delta;
        }
        
        // Gravity-based drift correction
        Vec3f expected_gravity = orientation.conjugate().rotate(Vec3f(0, 0, -9.81f));
        Vec3f gravity_error = expected_gravity.cross(accel);
        float error_magnitude = gravity_error.length();
        
        if (error_magnitude > 1e-6f) {
            Vec3f correction_axis = gravity_error.normalized();
            float correction_angle = error_magnitude * dt * 0.1f;  // Slow correction
            Quaternion<float> correction = Quaternion<float>::from_axis_angle(
                correction_axis, correction_angle
            );
            orientation = orientation * correction;
        }
        
        orientation = orientation.normalized();
    }
    
    Quaternion<float> get_orientation() const { return orientation; }
    Vec3f get_gyro_bias() const { return gyro_bias; }
};
```

---

## Coordinate Frame Transformations

### Common Aerospace Transformations

```cpp
// NED (North-East-Down) to ENU (East-North-Up)
Mat3f ned_to_enu() {
    return Mat3f({
        0.0f,  1.0f,  0.0f,
        1.0f,  0.0f,  0.0f,
        0.0f,  0.0f, -1.0f
    });
}

// Body frame to world frame transformation
Vec3f body_to_world(const Vec3f& body_vec, const Quaternion<float>& orientation) {
    return orientation.rotate(body_vec);
}

// Geographic to ECEF (Earth-Centered Earth-Fixed)
Vec3f geographic_to_ecef(float lat, float lon, float alt) {
    const float a = 6378137.0f;         // WGS84 semi-major axis
    const float f = 1.0f / 298.257223563f;  // Flattening
    const float e_sq = f * (2.0f - f);  // Eccentricity squared
    
    float sin_lat = std::sin(lat);
    float cos_lat = std::cos(lat);
    float sin_lon = std::sin(lon);
    float cos_lon = std::cos(lon);
    
    float N = a / std::sqrt(1.0f - e_sq * sin_lat * sin_lat);
    
    return Vec3f(
        (N + alt) * cos_lat * cos_lon,
        (N + alt) * cos_lat * sin_lon,
        (N * (1.0f - e_sq) + alt) * sin_lat
    );
}
```

---

## Collision Detection

### AABB (Axis-Aligned Bounding Box)

```cpp
struct AABB {
    Vec3f min, max;
    
    bool intersects(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }
    
    bool contains(const Vec3f& point) const {
        return (point.x() >= min.x() && point.x() <= max.x()) &&
               (point.y() >= min.y() && point.y() <= max.y()) &&
               (point.z() >= min.z() && point.z() <= max.z());
    }
    
    Vec3f center() const { return (min + max) * 0.5f; }
    Vec3f extents() const { return (max - min) * 0.5f; }
};
```

### Sphere Collision

```cpp
struct Sphere {
    Vec3f center;
    float radius;
    
    bool intersects(const Sphere& other) const {
        Vec3f diff = center - other.center;
        float dist_sq = diff.dot(diff);
        float radius_sum = radius + other.radius;
        return dist_sq <= radius_sum * radius_sum;
    }
    
    bool contains(const Vec3f& point) const {
        Vec3f diff = point - center;
        return diff.dot(diff) <= radius * radius;
    }
};
```

### Ray-Sphere Intersection

```cpp
struct Ray {
    Vec3f origin, direction;
    
    std::optional<float> intersect_sphere(const Sphere& sphere) const {
        Vec3f oc = origin - sphere.center;
        float b = direction.dot(oc);
        float c = oc.dot(oc) - sphere.radius * sphere.radius;
        float discriminant = b * b - c;
        
        if (discriminant < 0.0f) {
            return std::nullopt;  // No intersection
        }
        
        float t = -b - std::sqrt(discriminant);
        if (t < 0.0f) {
            t = -b + std::sqrt(discriminant);
        }
        
        return (t >= 0.0f) ? std::optional<float>(t) : std::nullopt;
    }
};
```

---

## Path Planning & Interpolation

### Cubic Hermite Spline

```cpp
Vec3f hermite_spline(const Vec3f& p0, const Vec3f& p1,
                     const Vec3f& v0, const Vec3f& v1, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    
    float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    float h10 = t3 - 2.0f * t2 + t;
    float h01 = -2.0f * t3 + 3.0f * t2;
    float h11 = t3 - t2;
    
    return p0 * h00 + v0 * h10 + p1 * h01 + v1 * h11;
}
```

### Catmull-Rom Spline

```cpp
Vec3f catmull_rom(const Vec3f& p0, const Vec3f& p1,
                  const Vec3f& p2, const Vec3f& p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    
    return p0 * (-0.5f * t3 + t2 - 0.5f * t) +
           p1 * (1.5f * t3 - 2.5f * t2 + 1.0f) +
           p2 * (-1.5f * t3 + 2.0f * t2 + 0.5f * t) +
           p3 * (0.5f * t3 - 0.5f * t2);
}
```

### Bezier Curve

```cpp
Vec3f quadratic_bezier(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, float t) {
    float u = 1.0f - t;
    return p0 * (u * u) + p1 * (2.0f * u * t) + p2 * (t * t);
}

Vec3f cubic_bezier(const Vec3f& p0, const Vec3f& p1,
                   const Vec3f& p2, const Vec3f& p3, float t) {
    float u = 1.0f - t;
    float u2 = u * u;
    float u3 = u2 * u;
    float t2 = t * t;
    float t3 = t2 * t;
    
    return p0 * u3 + p1 * (3.0f * u2 * t) + p2 * (3.0f * u * t2) + p3 * t3;
}
```

---

## Additional Resources

- **[API_Documentation.md](API_Documentation.md)** - Complete API reference
- **[PERFORMANCE.md](../PERFORMANCE.md)** - Performance benchmarks
- **[examples/](../examples/)** - More complete example programs
- **[QUICK_REFERENCE.md](../QUICK_REFERENCE.md)** - Quick syntax guide

## Contributing Recipes

Have a useful pattern? Submit a PR to add it to this cookbook! Please include:
- Clear code example with comments
- Usage demonstration
- Performance notes (if applicable)
- References to relevant literature

---

**Last Updated**: January 31, 2026  
**MicroLA Version**: 1.2.0
