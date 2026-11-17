#include "personTracker.h"

#include <algorithm>
#include <cmath>

namespace tracker_params
{
    constexpr double kSigmaAngle = 0.05;               // Angular measurement noise standard deviation (rad)
    constexpr double kSigmaDistance = 150.0;           // Distance measurement noise standard deviation (mm)
    constexpr double kQIntensity = 500.0 * 500.0;      // Continuous acceleration spectral density (mm/s^2)^2
    constexpr double kMahalanobisGate = 9.21;          // Chi-square gating threshold (2 DOF, 99%)
    constexpr double kMaxDropoutTime = 2.0;            // Maximum dropout time before tracker reset (s)
    constexpr int kMaxConsecutiveRejections = 5;       // Maximum consecutive measurement rejections
}

void PersonTracker::Reset()
{
    is_initialized_ = false;
    state_.setZero();
    covariance_.setZero();
    last_meas_time_ = -1.0;
    consecutive_rejections_ = 0;
}

void PersonTracker::EgoCompensate(double adv_speed, double rot_speed, double dt_ego)
{
    if (!is_initialized_ || dt_ego <= 0.0) return;

    // Positive rot_speed rotates robot counter-clockwise (+Z); relative scene rotates clockwise (-phi)
    const double phi = rot_speed * dt_ego;
    const double cos_phi = std::cos(phi);
    const double sin_phi = std::sin(phi);

    Eigen::Matrix2d rot_matrix;
    rot_matrix <<  cos_phi,  sin_phi,
                  -sin_phi,  cos_phi;

    const Eigen::Vector2d translation(0.0, adv_speed * dt_ego);

    state_.head<2>() = rot_matrix * (state_.head<2>() - translation);
    state_.tail<2>() = rot_matrix * state_.tail<2>();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
    transform.block<2, 2>(0, 0) = rot_matrix;
    transform.block<2, 2>(2, 2) = rot_matrix;

    covariance_ = transform * covariance_ * transform.transpose();
    SymmetrizeP();
}

void PersonTracker::Predict(double dt)
{
    if (!is_initialized_ || dt <= 0.0) return;

    Eigen::Matrix4d transition_matrix = Eigen::Matrix4d::Identity();
    transition_matrix(0, 2) = dt;
    transition_matrix(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    const double q11 = tracker_params::kQIntensity * (dt3 / 3.0);
    const double q12 = tracker_params::kQIntensity * (dt2 / 2.0);
    const double q22 = tracker_params::kQIntensity * dt;

    // Continuous white noise acceleration (CWNA) model decoupled in X and Y axes
    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    process_noise(0, 0) = q11; process_noise(0, 2) = q12;
    process_noise(2, 0) = q12; process_noise(2, 2) = q22;
    process_noise(1, 1) = q11; process_noise(1, 3) = q12;
    process_noise(3, 1) = q12; process_noise(3, 3) = q22;

    state_ = transition_matrix * state_;
    covariance_ = transition_matrix * covariance_ * transition_matrix.transpose() + process_noise;
    SymmetrizeP();
}

void PersonTracker::ProcessMeasurement(double angle, double distance, double time_now)
{
    const double dt_raw = time_now - last_meas_time_;

    if (!is_initialized_ || dt_raw > tracker_params::kMaxDropoutTime || dt_raw < 0.0)
    {
        InitFromMeasurement(angle, distance, time_now);
        return;
    }

    const double dt = std::clamp(dt_raw, 0.001, 0.5);
    Predict(dt);

    const Eigen::Vector2d measurement(distance * std::sin(angle), distance * std::cos(angle));
    const Eigen::Matrix2d meas_noise = ComputeR(angle, distance);

    Eigen::Matrix<double, 2, 4> obs_matrix = Eigen::Matrix<double, 2, 4>::Zero();
    obs_matrix(0, 0) = 1.0;
    obs_matrix(1, 1) = 1.0;

    const Eigen::Vector2d innovation = measurement - obs_matrix * state_;
    const Eigen::Matrix2d innovation_cov = obs_matrix * covariance_ * obs_matrix.transpose() + meas_noise;
    const Eigen::Matrix2d innovation_cov_inv = innovation_cov.inverse();

    const double mahalanobis_dist_sq = innovation.transpose() * innovation_cov_inv * innovation;
    if (mahalanobis_dist_sq > tracker_params::kMahalanobisGate)
    {
        consecutive_rejections_++;
        if (consecutive_rejections_ >= tracker_params::kMaxConsecutiveRejections)
        {
            InitFromMeasurement(angle, distance, time_now);
        }
        else
        {
            last_meas_time_ = time_now;
        }
        return;
    }

    consecutive_rejections_ = 0;
    last_meas_time_ = time_now;

    const Eigen::Matrix<double, 4, 2> kalman_gain = covariance_ * obs_matrix.transpose() * innovation_cov_inv;
    state_ = state_ + kalman_gain * innovation;

    // Joseph form covariance update for numerical symmetry and positive definiteness
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d ikh = identity - kalman_gain * obs_matrix;
    covariance_ = ikh * covariance_ * ikh.transpose() + kalman_gain * meas_noise * kalman_gain.transpose();
    SymmetrizeP();
}

bool PersonTracker::StepWithoutMeasurement(double dt_ego, double time_now)
{
    if (is_initialized_ && (time_now - last_meas_time_) <= tracker_params::kMaxDropoutTime)
    {
        const double dt = std::clamp(dt_ego, 0.001, 0.5);
        Predict(dt);
        return true;
    }
    Reset();
    return false;
}

std::tuple<double, double> PersonTracker::GetPolarEstimate() const
{
    const double px = state_(0);
    const double py = state_(1);
    return {std::atan2(px, py), std::hypot(px, py)};
}

double PersonTracker::GetRadialVelocity() const
{
    const auto [est_angle, est_dist] = GetPolarEstimate();
    if (est_dist < 1e-3) return 0.0;
    return state_(2) * std::sin(est_angle) + state_(3) * std::cos(est_angle);
}

bool PersonTracker::IsInitialized() const
{
    return is_initialized_;
}

Eigen::Matrix2d PersonTracker::ComputeR(double angle, double distance)
{
    Eigen::Matrix2d jacobian;
    jacobian << std::sin(angle),  distance * std::cos(angle),
                std::cos(angle), -distance * std::sin(angle);

    Eigen::Matrix2d polar_cov;
    polar_cov << tracker_params::kSigmaDistance * tracker_params::kSigmaDistance, 0.0,
                 0.0, tracker_params::kSigmaAngle * tracker_params::kSigmaAngle;

    return jacobian * polar_cov * jacobian.transpose();
}

void PersonTracker::InitFromMeasurement(double angle, double distance, double time_now)
{
    state_ << distance * std::sin(angle), distance * std::cos(angle), 0.0, 0.0;

    covariance_.setZero();
    covariance_.block<2, 2>(0, 0) = ComputeR(angle, distance);
    covariance_(2, 2) = 1000.0 * 1000.0;
    covariance_(3, 3) = 1000.0 * 1000.0;

    is_initialized_ = true;
    last_meas_time_ = time_now;
    consecutive_rejections_ = 0;
}

void PersonTracker::SymmetrizeP()
{
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}
