// SPDX-License-Identifier: Apache-2.0
/// @file extended_kalman.hpp
/// @brief Extended Kalman filter for nonlinear systems
/// @details Header-only EKF implementation using linearization via Jacobians
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

#include "matrix.hpp"
#include "vector.hpp"
#include "compiler_features.hpp"
#include <cmath>

namespace microla
{

/// @brief Extended Kalman filter for nonlinear systems
/// @tparam T Numeric type (typically float or double)
/// @tparam STATE_DIM Number of state variables
/// @tparam MEAS_DIM Number of measurements
/// @details Extends standard Kalman filter to nonlinear systems by linearizing
///          the process and measurement models around current state estimate.
///          User must provide functions for nonlinear state transition and measurement,
///          as well as their Jacobian matrices.
template<typename T = float, std::size_t StateDim = 2, std::size_t MeasDim = 1>
class ExtendedKalmanFilter
{
public:
    using StateVec = Vec<T, StateDim>;
    using MeasVec = Vec<T, MeasDim>;
    using StateMat = Mat<T, StateDim, StateDim>;
    using MeasMat = Mat<T, MeasDim, MeasDim>;
    using MeasToStateMat = Mat<T, MeasDim, StateDim>;
    using StateToMeasMat = Mat<T, StateDim, MeasDim>;

    /// @brief Function pointer type for state transition: x_k = f(x_k-1, u_k, dt)
    using StateTransitionFunc = StateVec (*)(const StateVec&, T);

    /// @brief Function pointer type for measurement model: z = h(x)
    using MeasurementFunc = MeasVec (*)(const StateVec&);

    /// @brief Function pointer type for state Jacobian: F = ∂f/∂x
    using StateJacobianFunc = StateMat (*)(const StateVec&, T);

    /// @brief Function pointer type for measurement Jacobian: H = ∂h/∂x
    using MeasurementJacobianFunc = MeasToStateMat (*)(const StateVec&);

private:
    StateVec x;  ///< State estimate vector
    StateMat P;  ///< State covariance matrix
    StateMat Q;  ///< Process noise covariance
    MeasMat R;   ///< Measurement noise covariance

    StateTransitionFunc f;               ///< Nonlinear state transition function
    MeasurementFunc h;                   ///< Nonlinear measurement function
    StateJacobianFunc F_jacobian;        ///< State transition Jacobian
    MeasurementJacobianFunc H_jacobian;  ///< Measurement Jacobian

public:
    /// @brief Constructor with function pointers
    /// @param state_func Nonlinear state transition function
    /// @param meas_func Nonlinear measurement function
    /// @param state_jac State transition Jacobian function
    /// @param meas_jac Measurement Jacobian function
    /// @param process_noise Process noise covariance
    /// @param measurement_noise Measurement noise covariance
    ExtendedKalmanFilter(StateTransitionFunc state_func, MeasurementFunc meas_func, StateJacobianFunc state_jac,
                         MeasurementJacobianFunc meas_jac, const StateMat& process_noise,
                         const MeasMat& measurement_noise) noexcept
        : x(), P(StateMat::identity()), Q(process_noise), R(measurement_noise), f(state_func), h(meas_func),
          F_jacobian(state_jac), H_jacobian(meas_jac)
    {
    }

    /// @brief Default constructor (requires function pointers to be set later)
    ExtendedKalmanFilter() noexcept
        : x(), P(StateMat::identity()), Q(StateMat::identity() * T(0.01)), R(MeasMat::identity() * T(0.1)), f(nullptr),
          h(nullptr), F_jacobian(nullptr), H_jacobian(nullptr)
    {
    }

    /// @brief Prediction step using nonlinear state transition
    /// @param dt Time step
    /// @details Propagates state through nonlinear dynamics:
    ///          x̂ₖ|ₖ₋₁ = f(x̂ₖ₋₁|ₖ₋₁, dt)
    ///          F = ∂f/∂x evaluated at x̂ₖ₋₁|ₖ₋₁
    ///          Pₖ|ₖ₋₁ = F * Pₖ₋₁|ₖ₋₁ * F^T + Q
    void predict(T dt) noexcept
    {
        if (!f || !F_jacobian)
        {
            return;
        }

        // Propagate state through nonlinear dynamics
        StateVec x_pred = f(x, dt);

        // Compute Jacobian at current state
        StateMat f = F_jacobian(x, dt);

        // Update state
        x = x_pred;

        // Propagate covariance
        P = f * P * f.transpose() + Q;
    }

    /// @brief Update step using nonlinear measurement model
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    /// @details Linearizes measurement model around predicted state:
    ///          ẑ = h(x̂ₖ|ₖ₋₁)
    ///          H = ∂h/∂x evaluated at x̂ₖ|ₖ₋₁
    ///          yₖ = z - ẑ (innovation)
    ///          Sₖ = H * Pₖ|ₖ₋₁ * H^T + R
    ///          Kₖ = Pₖ|ₖ₋₁ * H^T * Sₖ⁻¹
    ///          x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ * yₖ
    ///          Pₖ|ₖ = (I - Kₖ * H) * Pₖ|ₖ₋₁
    auto update(const MeasVec& z) noexcept -> bool
    {
        if (!h || !H_jacobian)
        {
            return false;
        }

        // Predict measurement from current state estimate
        MeasVec z_pred = h(x);

        // Compute measurement Jacobian at current state
        MeasToStateMat h = H_jacobian(x);

        // Innovation (measurement residual)
        MeasVec y = z - z_pred;

        // Innovation covariance
        MeasMat s = h * P * h.transpose() + R;

        // Check if innovation covariance is invertible
        T det = s.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;
        }

        // Kalman gain
        MeasMat s_inv;
        if (!s.inverse(s_inv))
        {
            return false;
        }
        StateToMeasMat k = P * h.transpose() * s_inv;

        // Update state estimate
        x = x + k * y;

        // Update covariance (simplified form)
        StateMat i = StateMat::identity();
        P = (i - k * h) * P;

        return true;
    }

    /// @brief Update step with Joseph form for improved numerical stability
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    auto update_joseph(const MeasVec& z) noexcept -> bool
    {
        if (!h || !H_jacobian)
        {
            return false;
        }

        MeasVec z_pred = h(x);
        MeasToStateMat h = H_jacobian(x);
        MeasVec y = z - z_pred;
        MeasMat s = h * P * h.transpose() + R;

        T det = s.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;
        }

        MeasMat s_inv;
        if (!s.inverse(s_inv))
        {
            return false;
        }
        StateToMeasMat k = P * h.transpose() * s_inv;
        x = x + k * y;

        // Joseph form covariance update
        StateMat i = StateMat::identity();
        StateMat ikh = i - k * h;
        P = ikh * P * ikh.transpose() + k * R * k.transpose();

        return true;
    }

    // ==================== Accessors ====================

    [[nodiscard]] auto get_state() const noexcept -> const StateVec& { return x; }
    [[nodiscard]] auto get_covariance() const noexcept -> const StateMat& { return P; }
    [[nodiscard]] auto get_state(std::size_t index) const noexcept -> T { return x[index]; }
    [[nodiscard]] auto get_variance(std::size_t index) const noexcept -> T { return P(index, index); }
    [[nodiscard]] auto get_std_dev(std::size_t index) const noexcept -> T { return std::sqrt(P(index, index)); }

    // ==================== Setters ====================

    void set_state(const StateVec& state) noexcept { x = state; }
    void set_covariance(const StateMat& covariance) noexcept { P = covariance; }
    void set_process_noise(const StateMat& Q_in) noexcept { Q = Q_in; }
    void set_measurement_noise(const MeasMat& R_in) noexcept { R = R_in; }

    void set_state_transition(StateTransitionFunc f_in, StateJacobianFunc F_jac) noexcept
    {
        f = f_in;
        F_jacobian = F_jac;
    }

    void set_measurement_model(MeasurementFunc h_in, MeasurementJacobianFunc H_jac) noexcept
    {
        h = h_in;
        H_jacobian = H_jac;
    }

    // ==================== Utility Methods ====================

    void reset() noexcept
    {
        x = StateVec();
        P = StateMat::identity();
    }

    void reset(const StateVec& x0, const StateMat& P0) noexcept
    {
        x = x0;
        P = P0;
    }

    /// @brief Compute innovation for current state
    [[nodiscard]] auto compute_innovation(const MeasVec& z) const noexcept -> MeasVec
    {
        if (!h)
        {
            return MeasVec();
        }
        return z - h(x);
    }

    /// @brief Compute normalized innovation squared (NIS) for outlier detection
    [[nodiscard]] auto compute_nis(const MeasVec& z) const noexcept -> T
    {
        if (!h || !H_jacobian)
        {
            return T(0);
        }

        MeasVec y = z - h(x);
        MeasToStateMat h = H_jacobian(x);
        MeasMat s = h * P * h.transpose() + R;
        MeasMat s_inv;
        if (!s.inverse(s_inv))
        {
            return T(0);
        }

        T nis = T(0);
        for (std::size_t i = 0; i < MeasDim; ++i)
        {
            for (std::size_t j = 0; j < MeasDim; ++j)
            {
                nis += y[i] * s_inv(i, j) * y[j];
            }
        }
        return nis;
    }
};

// ==================== Common EKF Configurations ====================

/// @brief EKF for 2D position tracking with nonlinear dynamics
template<typename T = float>
using EKF2D = ExtendedKalmanFilter<T, 2, 2>;

/// @brief EKF for 3D position and velocity
template<typename T = float>
using EKF6D = ExtendedKalmanFilter<T, 6, 3>;

/// @brief EKF for attitude estimation (quaternion + angular velocity)
template<typename T = float>
using EKFAttitude = ExtendedKalmanFilter<T, 7, 6>;

}  // namespace microla
