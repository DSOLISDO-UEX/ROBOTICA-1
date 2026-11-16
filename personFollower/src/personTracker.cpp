#include "personTracker.h"
#include <cmath>
#include <algorithm>

// Parámetros globales de sintonización del Filtro de Kalman
namespace TrackerParams
{
    double sigmaAngle = 0.05;               // Desviación estándar del ruido angular (rad)
    double sigmaDistance = 150.0;           // Desviación estándar del ruido en distancia (mm)
    double qIntensity = 500.0 * 500.0;      // Densidad espectral del ruido de aceleración (mm/s^2)^2
    double mahalanobisGate = 9.21;          // Umbral chi-cuadrado (2 gl, 99%)
    double maxDropoutTime = 2.0;            // Tiempo máximo sin medidas antes de reiniciar (s)
    int maxConsecutiveRejections = 5;       // Rechazos consecutivos antes de forzar reinicio
}

void PersonTracker::Reset()
{
    isInitialized_ = false;
    state_.setZero();
    covariance_.setZero();
    lastMeasTime_ = -1.0;
    consecutiveRejections_ = 0;
}

void PersonTracker::EgoCompensate(double advSpeed, double rotSpeed, double dtEgo)
{
    if (!isInitialized_ || dtEgo <= 0.0) return;

    // En RoboComp rotSpeed positivo gira el robot en sentido antihorario (+z); el mundo relativo rota en sentido horario (-phi)
    const double phi = rotSpeed * dtEgo;
    const double cosPhi = std::cos(phi);
    const double sinPhi = std::sin(phi);

    Eigen::Matrix2d rotMatrix;
    rotMatrix <<  cosPhi,  sinPhi,
                 -sinPhi,  cosPhi;

    const Eigen::Vector2d translation(0.0, advSpeed * dtEgo);

    state_.head<2>() = rotMatrix * (state_.head<2>() - translation);
    state_.tail<2>() = rotMatrix * state_.tail<2>();

    Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
    transform.block<2, 2>(0, 0) = rotMatrix;
    transform.block<2, 2>(2, 2) = rotMatrix;

    covariance_ = transform * covariance_ * transform.transpose();
    SymmetriseP();
}

void PersonTracker::Predict(double dt)
{
    if (!isInitialized_ || dt <= 0.0) return;

    Eigen::Matrix4d transitionMatrix = Eigen::Matrix4d::Identity();
    transitionMatrix(0, 2) = dt;
    transitionMatrix(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    const double q11 = TrackerParams::qIntensity * (dt3 / 3.0);
    const double q12 = TrackerParams::qIntensity * (dt2 / 2.0);
    const double q22 = TrackerParams::qIntensity * dt;

    // Modelo de ruido blanco de aceleración continua (CWNA) desacoplado en ejes X (0,2) e Y (1,3)
    Eigen::Matrix4d processNoise = Eigen::Matrix4d::Zero();
    processNoise(0, 0) = q11; processNoise(0, 2) = q12;
    processNoise(2, 0) = q12; processNoise(2, 2) = q22;
    processNoise(1, 1) = q11; processNoise(1, 3) = q12;
    processNoise(3, 1) = q12; processNoise(3, 3) = q22;

    state_ = transitionMatrix * state_;
    covariance_ = transitionMatrix * covariance_ * transitionMatrix.transpose() + processNoise;
    SymmetriseP();
}

void PersonTracker::ProcessMeasurement(double angle, double distance, double timeNow)
{
    const double dtRaw = timeNow - lastMeasTime_;

    if (!isInitialized_ || dtRaw > TrackerParams::maxDropoutTime || dtRaw < 0.0)
    {
        InitFromMeasurement(angle, distance, timeNow);
        return;
    }

    const double dt = std::clamp(dtRaw, 0.001, 0.5);
    Predict(dt);

    const Eigen::Vector2d measurement(distance * std::sin(angle), distance * std::cos(angle));
    const Eigen::Matrix2d measNoise = ComputeR(angle, distance);

    Eigen::Matrix<double, 2, 4> obsMatrix = Eigen::Matrix<double, 2, 4>::Zero();
    obsMatrix(0, 0) = 1.0;
    obsMatrix(1, 1) = 1.0;

    const Eigen::Vector2d innovation = measurement - obsMatrix * state_;
    const Eigen::Matrix2d innovationCov = obsMatrix * covariance_ * obsMatrix.transpose() + measNoise;
    const Eigen::Matrix2d innovationCovInv = innovationCov.inverse();

    const double mahalanobisDist2 = innovation.transpose() * innovationCovInv * innovation;
    if (mahalanobisDist2 > TrackerParams::mahalanobisGate)
    {
        consecutiveRejections_++;
        if (consecutiveRejections_ >= TrackerParams::maxConsecutiveRejections)
        {
            InitFromMeasurement(angle, distance, timeNow);
        }
        else
        {
            lastMeasTime_ = timeNow;
        }
        return;
    }

    consecutiveRejections_ = 0;
    lastMeasTime_ = timeNow;

    const Eigen::Matrix<double, 4, 2> kalmanGain = covariance_ * obsMatrix.transpose() * innovationCovInv;
    state_ = state_ + kalmanGain * innovation;

    // Forma de Joseph para garantizar estabilidad numérica y definición positiva de P
    const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d ikh = identity - kalmanGain * obsMatrix;
    covariance_ = ikh * covariance_ * ikh.transpose() + kalmanGain * measNoise * kalmanGain.transpose();
    SymmetriseP();
}

bool PersonTracker::StepWithoutMeasurement(double dtEgo, double timeNow)
{
    if (isInitialized_ && (timeNow - lastMeasTime_) <= TrackerParams::maxDropoutTime)
    {
        const double dt = std::clamp(dtEgo, 0.001, 0.5);
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
    const auto [estAngle, estDist] = GetPolarEstimate();
    if (estDist < 1e-3) return 0.0;
    return state_(2) * std::sin(estAngle) + state_(3) * std::cos(estAngle);
}

bool PersonTracker::IsInitialized() const
{
    return isInitialized_;
}

Eigen::Matrix2d PersonTracker::ComputeR(double angle, double distance)
{
    // Jacobiano de la transformación polar a cartesiana: x = d*sin(a), y = d*cos(a)
    Eigen::Matrix2d jacobian;
    jacobian << std::sin(angle),  distance * std::cos(angle),
                std::cos(angle), -distance * std::sin(angle);

    Eigen::Matrix2d polarCov;
    polarCov << TrackerParams::sigmaDistance * TrackerParams::sigmaDistance, 0.0,
                0.0, TrackerParams::sigmaAngle * TrackerParams::sigmaAngle;

    return jacobian * polarCov * jacobian.transpose();
}

void PersonTracker::InitFromMeasurement(double angle, double distance, double timeNow)
{
    state_ << distance * std::sin(angle), distance * std::cos(angle), 0.0, 0.0;

    covariance_.setZero();
    covariance_.block<2, 2>(0, 0) = ComputeR(angle, distance);
    covariance_(2, 2) = 1000.0 * 1000.0;
    covariance_(3, 3) = 1000.0 * 1000.0;

    isInitialized_ = true;
    lastMeasTime_ = timeNow;
    consecutiveRejections_ = 0;
}

void PersonTracker::SymmetriseP()
{
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}