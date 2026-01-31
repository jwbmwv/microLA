// SPDX-License-Identifier: Apache-2.0
/// @file kalman.hpp
/// @brief Standard Kalman filter implementation for embedded systems
/// @details Header-only Kalman filter with compile-time dimensions for optimal performance
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

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
template<typename T = float, std::size_t StateDim = 2, std::size_t MeasDim = 1>
class KalmanFilter
{
public:
    using StateVec = Vec<T, StateDim>;
    using MeasVec = Vec<T, MeasDim>;
    using StateMat = Mat<T, StateDim, StateDim>;
    using MeasMat = Mat<T, MeasDim, MeasDim>;
    using MeasToStateMat = Mat<T, MeasDim, StateDim>;
    using StateToMeasMat = Mat<T, StateDim, MeasDim>;

private:
    StateVec x;        ///< State estimate vector
    StateMat P;        ///< State covariance matrix
    StateMat F;        ///< State transition matrix
    MeasToStateMat H;  ///< Measurement matrix (maps state to measurement)
    StateMat Q;        ///< Process noise covariance
    MeasMat R;         ///< Measurement noise covariance

public:
    /// @brief Default constructor with identity initialization
    KalmanFilter() noexcept
        : x(), P(StateMat::identity()), F(StateMat::identity()), H(), Q(StateMat::identity() * T(0.01)),
          R(MeasMat::identity() * T(0.1))
    {
    }

    /// @brief Constructor with custom noise covariances
    /// @param process_noise Process noise covariance (Q)
    /// @param measurement_noise Measurement noise covariance (R)
    KalmanFilter(const StateMat& process_noise, const MeasMat& measurement_noise) noexcept
        : x(), P(StateMat::identity()), F(StateMat::identity()), H(), Q(process_noise), R(measurement_noise)
    {
    }

    /// @brief Prediction step (time update)
    /// @details Propagates state and covariance forward in time:
    ///          x̂ₖ|ₖ₋₁ = F * x̂ₖ₋₁|ₖ₋₁
    ///          Pₖ|ₖ₋₁ = F * Pₖ₋₁|ₖ₋₁ * F^T + Q
    void predict() noexcept
    {
        // Predict state: x = F * x
        x = F * x;

        // Predict covariance: P = F * P * F^T + Q
        P = F * P * F.transpose() + Q;
    }

    /// @brief Prediction step with control input
    /// @param u Control input vector
    /// @param B Control matrix (maps control to state)
    /// @details Extended prediction: x̂ₖ|ₖ₋₁ = F * x̂ₖ₋₁|ₖ₋₁ + B * u
    template<std::size_t CtrlDim>
    void predict(const Vec<T, CtrlDim>& u, const Mat<T, StateDim, CtrlDim>& B) noexcept
    {
        // Predict state with control: x = F * x + B * u
        x = F * x + B * u;

        // Predict covariance: P = F * P * F^T + Q
        P = F * P * F.transpose() + Q;
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
    auto update(const MeasVec& z) noexcept -> bool
    {
        // Innovation (measurement residual): y = z - H * x
        MeasVec y = z - H * x;

        // Innovation covariance: S = H * P * H^T + R
        MeasMat s = H * P * H.transpose() + R;

        // Check if innovation covariance is invertible
        T det = s.determinant();
        if (std::abs(det) < std::numeric_limits<T>::epsilon())
        {
            return false;  // Singular innovation covariance, reject measurement
        }

        // Kalman gain: K = P * H^T * S^(-1)
        MeasMat s_inv;
        if (!s.inverse(s_inv))
        {
            return false;
        }
        StateToMeasMat k = P * H.transpose() * s_inv;

        // Update state estimate: x = x + K * y
        x = x + k * y;

        // Update covariance using Joseph form for numerical stability
        // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
        StateMat i = StateMat::identity();
        StateMat ikh = i - k * H;
        P = ikh * P * ikh.transpose() + k * R * k.transpose();

        return true;
    }

    /// @brief Simplified update without Joseph form (faster but less stable)
    /// @param z Measurement vector
    /// @details Uses simplified covariance update: P = (I - K*H) * P
    ///          Faster but may lose positive definiteness in extreme cases
    auto update_simple(const MeasVec& z) noexcept -> bool
    {
        MeasVec y = z - H * x;
        MeasMat s = H * P * H.transpose() + R;

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
        StateToMeasMat k = P * H.transpose() * s_inv;
        x = x + k * y;

        // Simplified covariance update
        StateMat i = StateMat::identity();
        P = (i - k * H) * P;

        return true;
    }

    // ==================== Accessors ====================

    /// @brief Get current state estimate
    [[nodiscard]] auto get_state() const noexcept -> const StateVec& { return x; }

    /// @brief Get current covariance estimate
    [[nodiscard]] auto get_covariance() const noexcept -> const StateMat& { return P; }

    /// @brief Get specific state variable
    [[nodiscard]] auto get_state(std::size_t index) const noexcept -> T { return x[index]; }

    /// @brief Get state variance for specific variable
    [[nodiscard]] auto get_variance(std::size_t index) const noexcept -> T { return P(index, index); }

    /// @brief Get state standard deviation
    [[nodiscard]] auto get_std_dev(std::size_t index) const noexcept -> T { return std::sqrt(P(index, index)); }

    // ==================== Setters ====================

    /// @brief Set state estimate
    void set_state(const StateVec& state) noexcept { x = state; }

    /// @brief Set covariance matrix
    void set_covariance(const StateMat& covariance) noexcept { P = covariance; }

    /// @brief Set state transition matrix
    void set_state_transition(const StateMat& F_in) noexcept { F = F_in; }

    /// @brief Set measurement matrix
    void set_measurement_matrix(const MeasToStateMat& H_in) noexcept { H = H_in; }

    /// @brief Set process noise covariance
    void set_process_noise(const StateMat& Q_in) noexcept { Q = Q_in; }

    /// @brief Set measurement noise covariance
    void set_measurement_noise(const MeasMat& R_in) noexcept { R = R_in; }

    // ==================== Matrix Accessors (for advanced users) ====================

    /// @brief Get state transition matrix
    [[nodiscard]] auto get_f() const noexcept -> const StateMat& { return F; }

    /// @brief Get measurement matrix
    [[nodiscard]] auto get_h() const noexcept -> const MeasToStateMat& { return H; }

    /// @brief Get process noise covariance
    [[nodiscard]] auto get_q() const noexcept -> const StateMat& { return Q; }

    /// @brief Get measurement noise covariance
    [[nodiscard]] auto get_r() const noexcept -> const MeasMat& { return R; }

    // ==================== Utility Methods ====================

    /// @brief Reset filter to initial state
    void reset() noexcept
    {
        x = StateVec();
        P = StateMat::identity();
    }

    /// @brief Reset with specific initial state and covariance
    void reset(const StateVec& x0, const StateMat& P0) noexcept
    {
        x = x0;
        P = P0;
    }

    /// @brief Compute innovation (measurement residual)
    [[nodiscard]] auto compute_innovation(const MeasVec& z) const noexcept -> MeasVec { return z - H * x; }

    /// @brief Compute innovation covariance
    [[nodiscard]] auto compute_innovation_covariance() const noexcept -> MeasMat { return H * P * H.transpose() + R; }

    /// @brief Compute normalized innovation squared (NIS) for measurement validation
    /// @details NIS follows chi-squared distribution with MEAS_DIM degrees of freedom.
    ///          Use for outlier detection: if NIS > threshold, reject measurement.
    [[nodiscard]] auto compute_nis(const MeasVec& z) const noexcept -> T
    {
        MeasVec y = compute_innovation(z);
        MeasMat s = compute_innovation_covariance();
        MeasMat s_inv;
        if (!s.inverse(s_inv))
        {
            return T(0);
        }

        // NIS = y^T * S^(-1) * y
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
