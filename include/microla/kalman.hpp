// SPDX-License-Identifier: MIT
/// @file kalman.hpp
/// @brief Standard Kalman filter implementation for embedded systems
/// @details Header-only Kalman filter with compile-time dimensions for optimal performance
/// @copyright Copyright (c) 2026 James Baldwin
/// @author James Baldwin

#pragma once

#ifndef MICROLA_KALMAN_HPP_
#define MICROLA_KALMAN_HPP_

#include "matrix.hpp"
#include "vector.hpp"
#include "compiler_features.hpp"

namespace microla
{

/// @brief Standard Kalman filter for linear systems
/// @tparam T Numeric type (typically float or double)
/// @tparam STATE_DIM Number of state variables
/// @tparam MEAS_DIM Number of measurements
/// @details Implements the optimal recursive estimator for linear Gaussian systems.
///          Uses Joseph form covariance update for numerical stability.
///          All dimensions known at compile time for zero-overhead abstractions.
template<typename T = float, std::size_t STATE_DIM = 2, std::size_t MEAS_DIM = 1>
class KalmanFilter
{
public:
    using StateVec = Vec<T, STATE_DIM>;
    using MeasVec = Vec<T, MEAS_DIM>;
    using StateMat = Mat<T, STATE_DIM, STATE_DIM>;
    using MeasMat = Mat<T, MEAS_DIM, MEAS_DIM>;
    using MeasToStateMat = Mat<T, MEAS_DIM, STATE_DIM>;
    using StateToMeasMat = Mat<T, STATE_DIM, MEAS_DIM>;

private:
    StateVec x_;        ///< State estimate vector
    StateMat P_;        ///< State covariance matrix
    StateMat F_;        ///< State transition matrix
    MeasToStateMat H_;  ///< Measurement matrix (maps state to measurement)
    StateMat Q_;        ///< Process noise covariance
    MeasMat R_;         ///< Measurement noise covariance

public:
    /// @brief Default constructor with identity initialization
    KalmanFilter() noexcept
        : x_(), P_(StateMat::identity()), F_(StateMat::identity()), H_(), Q_(StateMat::identity() * T(0.01)),
          R_(MeasMat::identity() * T(0.1))
    {
    }

    /// @brief Constructor with custom noise covariances
    /// @param process_noise Process noise covariance (Q)
    /// @param measurement_noise Measurement noise covariance (R)
    KalmanFilter(const StateMat& process_noise, const MeasMat& measurement_noise) noexcept
        : x_(), P_(StateMat::identity()), F_(StateMat::identity()), H_(), Q_(process_noise), R_(measurement_noise)
    {
    }

    /// @brief Prediction step (time update)
    /// @details Propagates state and covariance forward in time:
    ///          x̂ₖ|ₖ₋₁ = F * x̂ₖ₋₁|ₖ₋₁
    ///          Pₖ|ₖ₋₁ = F * Pₖ₋₁|ₖ₋₁ * F^T + Q
    void predict() noexcept
    {
        // Predict state: x = F * x
        x_ = F_ * x_;

        // Predict covariance: P = F * P * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /// @brief Prediction step with control input
    /// @param u Control input vector
    /// @param B Control matrix (maps control to state)
    /// @details Extended prediction: x̂ₖ|ₖ₋₁ = F * x̂ₖ₋₁|ₖ₋₁ + B * u
    template<std::size_t CTRL_DIM>
    void predict(const Vec<T, CTRL_DIM>& u, const Mat<T, STATE_DIM, CTRL_DIM>& B) noexcept
    {
        // Predict state with control: x = F * x + B * u
        x_ = F_ * x_ + B * u;

        // Predict covariance: P = F * P * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /// @brief Update step (measurement update) with innovation validation
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    /// @details Incorporates measurement to refine state estimate:
    ///          yₖ = z - H * x̂ₖ|ₖ₋₁ (innovation)
    ///          Sₖ = H * Pₖ|ₖ₋₁ * H^T + R (innovation covariance)
    ///          Kₖ = Pₖ|ₖ₋₁ * H^T * Sₖ⁻¹ (Kalman gain)
    ///          x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ * yₖ
    ///          Pₖ|ₖ = (I - Kₖ * H) * Pₖ|ₖ₋₁ * (I - Kₖ * H)^T + Kₖ * R * Kₖ^T (Joseph form)
    bool update(const MeasVec& z) noexcept
    {
        // Innovation (measurement residual): y = z - H * x
        MeasVec y = z - H_ * x_;

        // Innovation covariance: S = H * P * H^T + R
        MeasMat S = H_ * P_ * H_.transpose() + R_;

        // Check if innovation covariance is invertible
        T det = S.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;  // Singular innovation covariance, reject measurement
        }

        // Kalman gain: K = P * H^T * S^(-1)
        StateToMeasMat K = P_ * H_.transpose() * S.inverse();

        // Update state estimate: x = x + K * y
        x_ = x_ + K * y;

        // Update covariance using Joseph form for numerical stability
        // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        StateMat I = StateMat::identity();
        StateMat IKH = I - K * H_;
        P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();

        return true;
    }

    /// @brief Simplified update without Joseph form (faster but less stable)
    /// @param z Measurement vector
    /// @details Uses simplified covariance update: P = (I - K*H) * P
    ///          Faster but may lose positive definiteness in extreme cases
    bool update_simple(const MeasVec& z) noexcept
    {
        MeasVec y = z - H_ * x_;
        MeasMat S = H_ * P_ * H_.transpose() + R_;

        T det = S.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;
        }

        StateToMeasMat K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;

        // Simplified covariance update
        StateMat I = StateMat::identity();
        P_ = (I - K * H_) * P_;

        return true;
    }

    // ==================== Accessors ====================

    /// @brief Get current state estimate
    const StateVec& get_state() const noexcept { return x_; }

    /// @brief Get current covariance estimate
    const StateMat& get_covariance() const noexcept { return P_; }

    /// @brief Get specific state variable
    T get_state(std::size_t index) const noexcept { return x_[static_cast<std::uint32_t>(index)]; }

    /// @brief Get state variance for specific variable
    T get_variance(std::size_t index) const noexcept
    {
        return P_(static_cast<std::uint32_t>(index), static_cast<std::uint32_t>(index));
    }

    /// @brief Get state standard deviation
    T get_std_dev(std::size_t index) const noexcept
    {
        return std::sqrt(P_(static_cast<std::uint32_t>(index), static_cast<std::uint32_t>(index)));
    }

    // ==================== Setters ====================

    /// @brief Set state estimate
    void set_state(const StateVec& x) noexcept { x_ = x; }

    /// @brief Set covariance matrix
    void set_covariance(const StateMat& P) noexcept { P_ = P; }

    /// @brief Set state transition matrix
    void set_state_transition(const StateMat& F) noexcept { F_ = F; }

    /// @brief Set measurement matrix
    void set_measurement_matrix(const MeasToStateMat& H) noexcept { H_ = H; }

    /// @brief Set process noise covariance
    void set_process_noise(const StateMat& Q) noexcept { Q_ = Q; }

    /// @brief Set measurement noise covariance
    void set_measurement_noise(const MeasMat& R) noexcept { R_ = R; }

    // ==================== Matrix Accessors (for advanced users) ====================

    /// @brief Get state transition matrix
    const StateMat& get_F() const noexcept { return F_; }

    /// @brief Get measurement matrix
    const MeasToStateMat& get_H() const noexcept { return H_; }

    /// @brief Get process noise covariance
    const StateMat& get_Q() const noexcept { return Q_; }

    /// @brief Get measurement noise covariance
    const MeasMat& get_R() const noexcept { return R_; }

    // ==================== Utility Methods ====================

    /// @brief Reset filter to initial state
    void reset() noexcept
    {
        x_ = StateVec();
        P_ = StateMat::identity();
    }

    /// @brief Reset with specific initial state and covariance
    void reset(const StateVec& x0, const StateMat& P0) noexcept
    {
        x_ = x0;
        P_ = P0;
    }

    /// @brief Compute innovation (measurement residual)
    MeasVec compute_innovation(const MeasVec& z) const noexcept { return z - H_ * x_; }

    /// @brief Compute innovation covariance
    MeasMat compute_innovation_covariance() const noexcept { return H_ * P_ * H_.transpose() + R_; }

    /// @brief Compute normalized innovation squared (NIS) for measurement validation
    /// @details NIS follows chi-squared distribution with MEAS_DIM degrees of freedom.
    ///          Use for outlier detection: if NIS > threshold, reject measurement.
    T compute_nis(const MeasVec& z) const noexcept
    {
        MeasVec y = compute_innovation(z);
        MeasMat S = compute_innovation_covariance();
        MeasMat S_inv = S.inverse();

        // NIS = y^T * S^(-1) * y
        T nis = T(0);
        for (std::size_t i = 0; i < MEAS_DIM; ++i)
        {
            for (std::size_t j = 0; j < MEAS_DIM; ++j)
            {
                nis += y[static_cast<std::uint32_t>(i)] *
                       S_inv(static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)) *
                       y[static_cast<std::uint32_t>(j)];
            }
        }
        return nis;
    }
};

// ==================== Type Aliases for Common Use Cases ====================

/// @brief 1D position tracking (position only)
template<typename T = float>
using KalmanFilter1D = KalmanFilter<T, 1, 1>;

/// @brief 2D position + velocity tracking (constant velocity model)
template<typename T = float>
using KalmanFilter2D = KalmanFilter<T, 2, 1>;

/// @brief 4D position + velocity in 2D space
template<typename T = float>
using KalmanFilter4D = KalmanFilter<T, 4, 2>;

/// @brief 6D position + velocity in 3D space
template<typename T = float>
using KalmanFilter6D = KalmanFilter<T, 6, 3>;

}  // namespace microla

#endif  // MICROLA_KALMAN_HPP_
