#ifndef PERSON_TRACKER_H
#define PERSON_TRACKER_H

#include <tuple>
#include <Eigen/Dense>

/**
 * @brief Linear Kalman filter with egomotion compensation for 2D person tracking.
 */
class PersonTracker
{
public:
    PersonTracker() = default;

    /**
     * @brief Resets filter state vector, covariance matrix, and rejection counters.
     */
    void Reset();

    /**
     * @brief Compensates for robot ego-motion displacement and rotation in state space.
     * @param adv_speed Linear velocity command (mm/s).
     * @param rot_speed Angular velocity command (rad/s).
     * @param dt_ego Elapsed time step (s).
     */
    void EgoCompensate(double adv_speed, double rot_speed, double dt_ego);

    /**
     * @brief Executes constant-velocity Kalman prediction step.
     * @param dt Time delta (s).
     */
    void Predict(double dt);

    /**
     * @brief Updates filter with new polar observation after Mahalanobis gating.
     * @param angle Measured target angle in robot frame (rad).
     * @param distance Measured target distance in robot frame (mm).
     * @param time_now Current timestamp (s).
     */
    void ProcessMeasurement(double angle, double distance, double time_now);

    /**
     * @brief Propagates filter state without observations until dropout threshold.
     * @param dt_ego Cycle time delta (s).
     * @param time_now Current timestamp (s).
     * @return True if state remains valid; false if expired and filter was reset.
     */
    bool StepWithoutMeasurement(double dt_ego, double time_now);

    /**
     * @brief Returns current estimated position in polar coordinates.
     * @return Tuple containing target angle (rad) and distance (mm).
     */
    std::tuple<double, double> GetPolarEstimate() const;

    /**
     * @brief Returns current estimated Cartesian position in robot frame (mm).
     */
    Eigen::Vector2d GetCartesianPosition() const { return state_.head<2>(); }

    /**
     * @brief Returns current estimated Cartesian velocity in robot frame (mm/s).
     */
    Eigen::Vector2d GetCartesianVelocity() const { return state_.tail<2>(); }

    /**
     * @brief Computes radial velocity projected along the line of sight to target.
     * @return Radial velocity (mm/s).
     */
    double GetRadialVelocity() const;

    /**
     * @brief Checks whether the tracker has been initialized with active tracking.
     * @return True if initialized.
     */
    bool IsInitialized() const;

private:
    Eigen::Vector4d state_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Zero();
    bool is_initialized_ = false;
    double last_meas_time_ = -1.0;
    int consecutive_rejections_ = 0;

    /**
     * @brief Projects polar measurement noise covariance into Cartesian space.
     * @param angle Measured angle (rad).
     * @param distance Measured distance (mm).
     * @return Cartesian 2x2 measurement covariance matrix R.
     */
    static Eigen::Matrix2d ComputeR(double angle, double distance);

    /**
     * @brief Initializes state vector and covariance from initial polar measurement.
     * @param angle Initial angle (rad).
     * @param distance Initial distance (mm).
     * @param time_now Current timestamp (s).
     */
    void InitFromMeasurement(double angle, double distance, double time_now);

    /**
     * @brief Enforces numerical symmetry on covariance matrix P.
     */
    void SymmetrizeP();
};

#endif // PERSON_TRACKER_H
