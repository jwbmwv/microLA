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
/// @tparam StateDim Number of state variables.
/// @tparam MeasDim Number of measurements.
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
    using ResidualFunc = MeasVec (*)(const MeasVec&, const MeasVec&);

private:
    StateVec x;  ///< State estimate vector
    StateMat P;  ///< State covariance matrix
    StateMat Q;  ///< Process noise covariance
    MeasMat R;   ///< Measurement noise covariance

    StateTransitionFunc f;               ///< Nonlinear state transition function
    MeasurementFunc h;                   ///< Nonlinear measurement function
    StateJacobianFunc F_jacobian;        ///< State transition Jacobian
    MeasurementJacobianFunc H_jacobian;  ///< Measurement Jacobian
    ResidualFunc residual = nullptr;

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
        if (!f || !F_jacobian || !std::isfinite(dt) || !x.is_finite() || !P.is_finite() || !Q.is_finite())
        {
            return;
        }

        const StateVec predicted_state = f(x, dt);
        const StateMat jacobian = F_jacobian(x, dt);
        if (!predicted_state.is_finite() || !jacobian.is_finite())
        {
            return;
        }
        const StateMat predicted_covariance = jacobian * P * jacobian.transpose() + Q;
        if (predicted_covariance.is_finite())
        {
            x = predicted_state;
            P = predicted_covariance;
        }
    }

    /// @brief Update step using nonlinear measurement model
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    /// @retval true Finite corrected state and covariance were committed.
    /// @retval false Missing callbacks, invalid data, failed inversion, or non-finite results; estimates are unchanged.
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
        if (!h || !H_jacobian || !z.is_finite() || !x.is_finite() || !P.is_finite() || !R.is_finite())
        {
            return false;
        }

        // Predict measurement from current state estimate
        MeasVec z_pred = h(x);

        // Compute measurement Jacobian at current state
        MeasToStateMat h = H_jacobian(x);

        // Innovation (measurement residual)
        MeasVec y = residual ? residual(z, z_pred) : z - z_pred;
        if (!y.is_finite() || !h.is_finite())
        {
            return false;
        }

        // Innovation covariance
        MeasMat s = h * P * h.transpose() + R;

        // Kalman gain
        MeasMat s_inv;
        if (!s.inverse_checked(s_inv))
        {
            return false;
        }
        StateToMeasMat k = P * h.transpose() * s_inv;

        // Update state estimate
        const StateVec next_state = x + k * y;

        // Update covariance (simplified form)
        StateMat i = StateMat::identity();
        const StateMat next_covariance = (i - k * h) * P;
        if (!next_state.is_finite() || !next_covariance.is_finite())
        {
            return false;
        }
        x = next_state;
        P = next_covariance;

        return true;
    }

    /// @brief Update step with Joseph form for improved numerical stability
    /// @param z Measurement vector
    /// @return True if update was applied, false if measurement rejected
    /// @retval true Finite corrected state and Joseph covariance were committed.
    /// @retval false Missing callbacks, invalid data, failed inversion, or non-finite results; estimates are unchanged.
    auto update_joseph(const MeasVec& z) noexcept -> bool
    {
        if (!h || !H_jacobian || !z.is_finite() || !x.is_finite() || !P.is_finite() || !R.is_finite())
        {
            return false;
        }

        MeasVec z_pred = h(x);
        MeasToStateMat h = H_jacobian(x);
        MeasVec y = residual ? residual(z, z_pred) : z - z_pred;
        if (!y.is_finite() || !h.is_finite())
        {
            return false;
        }
        MeasMat s = h * P * h.transpose() + R;

        MeasMat s_inv;
        if (!s.inverse_checked(s_inv))
        {
            return false;
        }
        StateToMeasMat k = P * h.transpose() * s_inv;
        const StateVec next_state = x + k * y;

        // Joseph form covariance update
        StateMat i = StateMat::identity();
        StateMat ikh = i - k * h;
        const StateMat next_covariance = ikh * P * ikh.transpose() + k * R * k.transpose();
        if (!next_state.is_finite() || !next_covariance.is_finite())
        {
            return false;
        }
        x = next_state;
        P = next_covariance;

        return true;
    }

    // ==================== Accessors ====================

    /// @brief Read the current state estimate.
    /// @return Const reference to the stored state vector.
    [[nodiscard]] auto get_state() const noexcept -> const StateVec& { return x; }
    /// @brief Read the current state covariance.
    /// @return Const reference to the stored covariance matrix.
    [[nodiscard]] auto get_covariance() const noexcept -> const StateMat& { return P; }
    /// @brief Read one state component.
    /// @param index State component index, less than StateDim.
    /// @return Value of the selected state component.
    [[nodiscard]] auto get_state(std::size_t index) const noexcept -> T { return x[index]; }
    /// @brief Read one state variance.
    /// @param index State component index, less than StateDim.
    /// @return Corresponding covariance diagonal entry.
    [[nodiscard]] auto get_variance(std::size_t index) const noexcept -> T { return P(index, index); }
    /// @brief Read one state standard deviation.
    /// @param index State component index, less than StateDim.
    /// @return Square root of the selected variance; negative variance produces NaN.
    [[nodiscard]] auto get_std_dev(std::size_t index) const noexcept -> T { return std::sqrt(P(index, index)); }

    // ==================== Setters ====================

    /// @brief Replace the state estimate without validation.
    /// @param state State vector to store.
    void set_state(const StateVec& state) noexcept { x = state; }
    /// @brief Replace the state covariance without validation.
    /// @param covariance State covariance to store.
    void set_covariance(const StateMat& covariance) noexcept { P = covariance; }
    /// @brief Set additive process noise per prediction.
    /// @param Q_in Process-noise covariance.
    void set_process_noise(const StateMat& Q_in) noexcept { Q = Q_in; }
    /// @brief Set measurement noise.
    /// @param R_in Measurement-noise covariance.
    void set_measurement_noise(const MeasMat& R_in) noexcept { R = R_in; }

    /// @brief Set the nonlinear transition and its Jacobian callbacks.
    /// @param f_in Callback mapping state and time step to predicted state; null disables prediction.
    /// @param F_jac Callback evaluating the transition Jacobian at the prior state and time step.
    void set_state_transition(StateTransitionFunc f_in, StateJacobianFunc F_jac) noexcept
    {
        f = f_in;
        F_jacobian = F_jac;
    }

    /// @brief Set the nonlinear measurement and its Jacobian callbacks.
    /// @param h_in Callback mapping state to predicted measurements; null disables updates.
    /// @param H_jac Callback evaluating the measurement Jacobian at the current state.
    void set_measurement_model(MeasurementFunc h_in, MeasurementJacobianFunc H_jac) noexcept
    {
        h = h_in;
        H_jacobian = H_jac;
    }

    // ==================== Utility Methods ====================

    /// @brief Set a custom innovation calculation, for example to wrap angles.
    /// @param function Callback accepting measured and predicted vectors; null restores measured - predicted.
    void set_residual_function(ResidualFunc function) noexcept { residual = function; }

    /// @brief Reset to zero state and identity covariance, retaining callbacks and noise settings.
    void reset() noexcept
    {
        x = StateVec();
        P = StateMat::identity();
    }

    /// @brief Reset the estimates, retaining callbacks and noise settings.
    /// @param x0 Initial state estimate.
    /// @param P0 Initial state covariance.
    void reset(const StateVec& x0, const StateMat& P0) noexcept
    {
        x = x0;
        P = P0;
    }

    /// @brief Compute innovation for current state
    /// @param z Measurement vector.
    /// @return Custom residual or z - h(x); a zero vector if no measurement callback is configured.
    [[nodiscard]] auto compute_innovation(const MeasVec& z) const noexcept -> MeasVec
    {
        if (!h)
        {
            return MeasVec();
        }
        const MeasVec predicted = h(x);
        return residual ? residual(z, predicted) : z - predicted;
    }

    /// @brief Compute normalized innovation squared (NIS) for outlier detection
    /// @param z Measurement vector to evaluate.
    /// @return Nonnegative NIS, or infinity for missing callbacks, invalid data, failed inversion, or invalid results.
    [[nodiscard]] auto compute_nis(const MeasVec& z) const noexcept -> T
    {
        if (!h || !H_jacobian || !z.is_finite() || !x.is_finite() || !P.is_finite() || !R.is_finite())
        {
            return std::numeric_limits<T>::infinity();
        }

        MeasVec y = compute_innovation(z);
        MeasToStateMat h = H_jacobian(x);
        MeasMat s = h * P * h.transpose() + R;
        MeasMat s_inv;
        if (!y.is_finite() || !s.inverse_checked(s_inv))
        {
            return std::numeric_limits<T>::infinity();
        }

        T nis = T(0);
        for (std::size_t i = 0; i < MeasDim; ++i)
        {
            for (std::size_t j = 0; j < MeasDim; ++j)
            {
                nis += y[i] * s_inv(i, j) * y[j];
            }
        }
        return std::isfinite(nis) && nis >= T(0) ? nis : std::numeric_limits<T>::infinity();
    }
};

// ==================== Common EKF Configurations ====================

/// @brief EKF for 2D position tracking with nonlinear dynamics
/// @tparam T Floating-point type for state and covariance calculations.
template<typename T = float>
using EKF2D = ExtendedKalmanFilter<T, 2, 2>;

/// @brief EKF for 3D position and velocity
/// @tparam T Floating-point type for state and covariance calculations.
template<typename T = float>
using EKF6D = ExtendedKalmanFilter<T, 6, 3>;

/// @brief EKF for attitude estimation (quaternion + angular velocity)
/// @tparam T Floating-point type for state and covariance calculations.
template<typename T = float>
using EKFAttitude = ExtendedKalmanFilter<T, 7, 6>;

}  // namespace microla
