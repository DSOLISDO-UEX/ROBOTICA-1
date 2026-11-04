#ifndef PERSON_TRACKER_H
#define PERSON_TRACKER_H

#include <Eigen/Dense>
#include <tuple>

class PersonTracker
{
public:
    PersonTracker() = default;

    void reset();

    // 4. Ego-motion compensation (done before the predict step)
    void ego_compensate(double adv, double rot, double dt_ego);

    // 1. Constant-velocity prediction step
    void predict(double dt);

    // 2 & 3. Measurement update, Mahalanobis gating and reinitialisation
    void process_measurement(double a, double d, double t_now);

    // Handle sensor dropout (predict without measurement up to 1 second)
    bool step_without_measurement(double dt_ego, double t_now);

    std::tuple<double, double> get_polar_estimate() const;
    double get_radial_velocity() const;
    bool is_initialized() const;

private:
    Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P_ = Eigen::Matrix4d::Zero();
    bool initialized_ = false;
    double last_meas_time_ = -1.0;
    int consecutive_rejections_ = 0;

    static constexpr double SIGMA_A = 0.05;               // rad
    static constexpr double SIGMA_D = 150.0;              // mm
    static constexpr double Q_INTENSITY = 500.0 * 500.0;  // (mm/s^2)^2
    static constexpr double MAHALANOBIS_GATE = 9.21;      // 2 dof, 99%

    static Eigen::Matrix2d compute_R(double a, double d);
    void init_from_measurement(double a, double d, double t_now);
    void symmetrise_P();
};

#endif // PERSON_TRACKER_H