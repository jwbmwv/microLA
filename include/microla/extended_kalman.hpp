// SPDX-License-Identifier: MIT
/// @file extended_kalman.hpp
/// @brief Extended Kalman filter for nonlinear systems
/// @details Header-only EKF implementation using linearization via Jacobians
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_EXTENDED_KALMAN_HPP_
#define MICROLA_EXTENDED_KALMAN_HPP_

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
template<typename T = float, std::size_t STATE_DIM = 2, std::size_t MEAS_DIM = 1>
class ExtendedKalmanFilter
{
public:
    using StateVec = Vec<T, STATE_DIM>;
    using MeasVec = Vec<T, MEAS_DIM>;
    using StateMat = Mat<T, STATE_DIM, STATE_DIM>;
    using MeasMat = Mat<T, MEAS_DIM, MEAS_DIM>;
    using MeasToStateMat = Mat<T, MEAS_DIM, STATE_DIM>;
    using StateToMeasMat = Mat<T, STATE_DIM, MEAS_DIM>;

    /// @brief Function pointer type for state transition: x_k = f(x_k-1, u_k, dt)
    using StateTransitionFunc = StateVec (*)(const StateVec&, T);

    /// @brief Function pointer type for measurement model: z = h(x)
    using MeasurementFunc = MeasVec (*)(const StateVec&);

    /// @brief Function pointer type for state Jacobian: F = ∂f/∂x
    using StateJacobianFunc = StateMat (*)(const StateVec&, T);

    /// @brief Function pointer type for measurement Jacobian: H = ∂h/∂x
    using MeasurementJacobianFunc = MeasToStateMat (*)(const StateVec&);

private:
    StateVec x_;                         ///< State estimate vector
    StateMat P_;                         ///< State covariance matrix
    StateMat Q_;                         ///< Process noise covariance
    MeasMat R_;                          ///< Measurement noise covariance
    
    StateTransitionFunc f_;              ///< Nonlinear state transition function
    MeasurementFunc h_;                  ///< Nonlinear measurement function
    StateJacobianFunc F_jacobian_;       ///< State transition Jacobian
    MeasurementJacobianFunc H_jacobian_; ///< Measurement Jacobian

public:
    /// @brief Constructor with function pointers
    /// @param state_func Nonlinear state transition function
    /// @param meas_func Nonlinear measurement function
    /// @param state_jac State transition Jacobian function
    /// @param meas_jac Measurement Jacobian function
    /// @param process_noise Process noise covariance
    /// @param measurement_noise Measurement noise covariance
    ExtendedKalmanFilter(
        StateTransitionFunc state_func,
        MeasurementFunc meas_func,
        StateJacobianFunc state_jac,
        MeasurementJacobianFunc meas_jac,
        const StateMat& process_noise,
        const MeasMat& measurement_noise) noexcept
        : x_()
        , P_(StateMat::identity())
        , Q_(process_noise)
        , R_(measurement_noise)
        , f_(state_func)
        , h_(meas_func)
        , F_jacobian_(state_jac)
        , H_jacobian_(meas_jac)
    {
    }

    /// @brief Default constructor (requires function pointers to be set later)
    ExtendedKalmanFilter() noexcept
        : x_()
        , P_(StateMat::identity())
        , Q_(StateMat::identity() * T(0.01))
        , R_(MeasMat::identity() * T(0.1))
        , f_(nullptr)
        , h_(nullptr)
        , F_jacobian_(nullptr)
        , H_jacobian_(nullptr)
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
        if (!f_ || !F_jacobian_)
            return;

        // Propagate state through nonlinear dynamics
        StateVec x_pred = f_(x_, dt);

        // Compute Jacobian at current state
        StateMat F = F_jacobian_(x_, dt);

        // Update state
        x_ = x_pred;

        // Propagate covariance
        P_ = F * P_ * F.transpose() + Q_;
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
    bool update(const MeasVec& z) noexcept
    {
        if (!h_ || !H_jacobian_)
            return false;

        // Predict measurement from current state estimate
        MeasVec z_pred = h_(x_);

        // Compute measurement Jacobian at current state
        MeasToStateMat H = H_jacobian_(x_);

        // Innovation (measurement residual)
        MeasVec y = z - z_pred;

        // Innovation covariance
        MeasMat S = H * P_ * H.transpose() + R_;

        // Check if innovation covariance is invertible
        T det = S.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;
        }

        // Kalman gain
        StateToMeasMat K = P_ * H.transpose() * S.inverse();

        // Update state estimate
        x_ = x_ + K * y;

        // Update covariance (simplified form)
        StateMat I = StateMat::identity();
        P_ = (I - K * H) * P_;

        return true;
    }

    /// @brief Update step with Joseph form for improved numerical stability
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    bool update_joseph(const MeasVec& z) noexcept
    {
        if (!h_ || !H_jacobian_)
            return false;

        MeasVec z_pred = h_(x_);
        MeasToStateMat H = H_jacobian_(x_);
        MeasVec y = z - z_pred;
        MeasMat S = H * P_ * H.transpose() + R_;

        T det = S.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;
        }

        StateToMeasMat K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;

        // Joseph form covariance update
        StateMat I = StateMat::identity();
        StateMat IKH = I - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();

        return true;
    }

    // ==================== Accessors ====================

    const StateVec& get_state() const noexcept { return x_; }
    const StateMat& get_covariance() const noexcept { return P_; }
    T get_state(std::size_t index) const noexcept { return x_[static_cast<std::uint32_t>(index)]; }
    T get_variance(std::size_t index) const noexcept { return P_(static_cast<std::uint32_t>(index), static_cast<std::uint32_t>(index)); }
    T get_std_dev(std::size_t index) const noexcept { return std::sqrt(P_(static_cast<std::uint32_t>(index), static_cast<std::uint32_t>(index))); }

    // ==================== Setters ====================

    void set_state(const StateVec& x) noexcept { x_ = x; }
    void set_covariance(const StateMat& P) noexcept { P_ = P; }
    void set_process_noise(const StateMat& Q) noexcept { Q_ = Q; }
    void set_measurement_noise(const MeasMat& R) noexcept { R_ = R; }

    void set_state_transition(StateTransitionFunc f, StateJacobianFunc F_jac) noexcept
    {
        f_ = f;
        F_jacobian_ = F_jac;
    }

    void set_measurement_model(MeasurementFunc h, MeasurementJacobianFunc H_jac) noexcept
    {
        h_ = h;
        H_jacobian_ = H_jac;
    }

    // ==================== Utility Methods ====================

    void reset() noexcept
    {
        x_ = StateVec();
        P_ = StateMat::identity();
    }

    void reset(const StateVec& x0, const StateMat& P0) noexcept
    {
        x_ = x0;
        P_ = P0;
    }

    /// @brief Compute innovation for current state
    MeasVec compute_innovation(const MeasVec& z) const noexcept
    {
        if (!h_)
            return MeasVec();
        return z - h_(x_);
    }

    /// @brief Compute normalized innovation squared (NIS) for outlier detection
    T compute_nis(const MeasVec& z) const noexcept
    {
        if (!h_ || !H_jacobian_)
            return T(0);

        MeasVec y = z - h_(x_);
        MeasToStateMat H = H_jacobian_(x_);
        MeasMat S = H * P_ * H.transpose() + R_;
        MeasMat S_inv = S.inverse();

        T nis = T(0);
        for (std::size_t i = 0; i < MEAS_DIM; ++i)
        {
            for (std::size_t j = 0; j < MEAS_DIM; ++j)
            {
                nis += y[static_cast<std::uint32_t>(i)] * S_inv(static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)) * y[static_cast<std::uint32_t>(j)];
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

#endif  // MICROLA_EXTENDED_KALMAN_HPP_
