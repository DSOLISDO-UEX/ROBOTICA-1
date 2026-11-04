#include "person_tracker.h"
#include <cmath>
#include <algorithm>

void PersonTracker::reset()
{
    initialized_ = false;
    x_.setZero();
    P_.setZero();
    last_meas_time_ = -1.0;
    consecutive_rejections_ = 0;
}

void PersonTracker::ego_compensate(double adv, double rot, double dt_ego)
{
    if (!initialized_ || dt_ego <= 0.0) return;

    // Bridge: positive rot turns counter-clockwise (-x); tracker wants positive toward +x
    const double phi = -rot * dt_ego;
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    Eigen::Matrix2d Rm;
    Rm << c, -s,
          s,  c;

    Eigen::Vector2d t(0.0, adv * dt_ego);

    x_.head<2>() = Rm * (x_.head<2>() - t);
    x_.tail<2>() = Rm * x_.tail<2>();

    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    T.block<2, 2>(0, 0) = Rm;
    T.block<2, 2>(2, 2) = Rm;

    P_ = T * P_ * T.transpose();
    symmetrise_P();
}

void PersonTracker::predict(double dt)
{
    if (!initialized_ || dt <= 0.0) return;

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    const double q11 = Q_INTENSITY * (dt3 / 3.0);
    const double q12 = Q_INTENSITY * (dt2 / 2.0);
    const double q22 = Q_INTENSITY * dt;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    // Per-axis white-noise acceleration for X (indices 0, 2) and Y (indices 1, 3)
    Q(0, 0) = q11; Q(0, 2) = q12;
    Q(2, 0) = q12; Q(2, 2) = q22;

    Q(1, 1) = q11; Q(1, 3) = q12;
    Q(3, 1) = q12; Q(3, 3) = q22;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
    symmetrise_P();
}

void PersonTracker::process_measurement(double a, double d, double t_now)
{
    const double dt_raw = t_now - last_meas_time_;

    // Reinitialise conditions: first call, gap > 1s, or negative gap
    if (!initialized_ || dt_raw > 1.0 || dt_raw < 0.0)
    {
        init_from_measurement(a, d, t_now);
        return;
    }

    // Time step clamp applied after reinitialisation checks
    const double dt = std::clamp(dt_raw, 0.001, 0.5);
    predict(dt);

    Eigen::Vector2d z(d * std::sin(a), d * std::cos(a));
    Eigen::Matrix2d R = compute_R(a, d);

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d y = z - H * x_;
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix2d S_inv = S.inverse();

    // Mahalanobis gate (2 dof, 99%)
    const double d2 = y.transpose() * S_inv * y;
    if (d2 > MAHALANOBIS_GATE)
    {
        consecutive_rejections_++;
        if (consecutive_rejections_ >= 5)
        {
            init_from_measurement(a, d, t_now);
        }
        else
        {
            last_meas_time_ = t_now;
        }
        return;
    }

    consecutive_rejections_ = 0;
    last_meas_time_ = t_now;

    Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S_inv;
    x_ = x_ + K * y;

    // Joseph form covariance update
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d I_KH = I - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
    symmetrise_P();
}

bool PersonTracker::step_without_measurement(double dt_ego, double t_now)
{
    if (initialized_ && (t_now - last_meas_time_) <= 1.0)
    {
        const double dt = std::clamp(dt_ego, 0.001, 0.5);
        predict(dt);
        return true;
    }
    reset();
    return false;
}

std::tuple<double, double> PersonTracker::get_polar_estimate() const
{
    const double px = x_(0);
    const double py = x_(1);
    return {std::atan2(px, py), std::hypot(px, py)};
}

double PersonTracker::get_radial_velocity() const
{
    const auto [a_hat, d_hat] = get_polar_estimate();
    if (d_hat < 1e-3) return 0.0;
    return x_(2) * std::sin(a_hat) + x_(3) * std::cos(a_hat);
}

bool PersonTracker::is_initialized() const
{
    return initialized_;
}

Eigen::Matrix2d PersonTracker::compute_R(double a, double d)
{
    Eigen::Matrix2d J;
    J << std::sin(a),  d * std::cos(a),
         std::cos(a), -d * std::sin(a);

    Eigen::Matrix2d Sigma;
    Sigma << SIGMA_D * SIGMA_D, 0.0,
             0.0, SIGMA_A * SIGMA_A;

    return J * Sigma * J.transpose();
}

void PersonTracker::init_from_measurement(double a, double d, double t_now)
{
    const double zx = d * std::sin(a);
    const double zy = d * std::cos(a);
    x_ << zx, zy, 0.0, 0.0;

    P_.setZero();
    P_.block<2, 2>(0, 0) = compute_R(a, d);
    P_(2, 2) = 1000.0 * 1000.0;
    P_(3, 3) = 1000.0 * 1000.0;

    initialized_ = true;
    last_meas_time_ = t_now;
    consecutive_rejections_ = 0;
}

void PersonTracker::symmetrise_P()
{
    P_ = 0.5 * (P_ + P_.transpose());
}