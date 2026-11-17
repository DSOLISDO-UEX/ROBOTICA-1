#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <tuple>
#include <utility>
#include <vector>

#include <QPointF>
#include <Lidar3D.h>

/**
 * @brief Dynamic Window Approach (DWA) + Proportional Controller
 *        for obstacle avoidance while tracking a moving person.
 */
class Controller
{
public:
    struct Params
    {
        // Robot Kinematic and Physical Limits
        float target_dist_mm = 1000.0f;       // Nominal following distance (mm)
        float max_adv_speed = 1000.0f;        // Max advance speed (mm/s)
        float min_adv_speed = 0.0f;           // Min forward advance speed (mm/s)
        float max_rot_speed = 1.2f;           // Max rotational speed (rad/s)
        float robot_radius_mm = 285.0f;       // Robot chassis half-width with wheelpods (284mm)
        float safety_margin_mm = 45.0f;       // Extra clearance margin (mm) -> collision_threshold = 330mm

        // DWA Simulation Settings
        float sim_time_sec = 1.1f;            // Forward projection horizon (s)
        int sim_steps = 11;                   // Arc discretization steps along trajectory
        int adv_samples = 21;                 // Discretization samples for advance speed (0..1000, 50mm/s resolution)
        int rot_samples = 25;                 // Discretization samples for rotational speed (-1.2..1.2)

        // Proportional Controller Baseline
        float kp_dist = 0.9f;                 // Linear distance gain
        float kp_rot = 1.2f;                  // Angular steering gain
        float kd_vel = 0.85f;                 // Radial velocity feedforward gain

        // Cost Function Weights
        float weight_target_dist = 2.5f;      // Term: end of arc at 1m from person (primary goal)
        float weight_heading = 0.8f;          // Term: end of arc faces the person (allows dodging)
        float weight_obstacle = 1.8f;         // Term: distance to obstacles (avoidance priority)
        float weight_speed = 0.8f;            // Term: prefer fast movements when far
        float weight_dist_to_person = 0.6f;   // Term: close distance to person when far
        float weight_prop_alignment = 0.8f;   // Bias towards proportional controller (only in free space)

        // Clearance thresholds
        float obstacle_influence_dist_mm = 1100.0f; // Range where obstacle creates repulsive cost
        float person_body_radius_mm = 450.0f;       // Radius around person centroid excluded from obstacles
        float min_self_chassis_dist_mm = 340.0f;     // Self-chassis filter threshold (wheelpods reach ~331mm)
    };

    struct Output
    {
        float adv_speed = 0.0f;              // Commanded advance velocity (mm/s)
        float rot_speed = 0.0f;              // Commanded rotational velocity (rad/s)
        bool is_avoiding = false;            // True if DWA deviates from proportional tracking
        float min_obs_dist = 99999.0f;       // Minimum obstacle clearance along trajectory
        std::vector<QPointF> best_arc;       // Trajectory points of selected arc
        std::vector<std::vector<QPointF>> candidate_arcs; // Sample of candidate arcs for visualization
    };

    Controller();
    explicit Controller(const Params& params);

    void SetParams(const Params& params);
    const Params& GetParams() const;

    /**
     * @brief Computes advance and rotational velocities using DWA and proportional control.
     * @param person_angle Polar angle towards person in robot frame (rad).
     * @param person_dist Distance towards person in robot frame (mm).
     * @param radial_vel Estimated radial velocity of person (mm/s).
     * @param person_vel_x Estimated Cartesian X velocity of person in robot frame (mm/s).
     * @param person_vel_y Estimated Cartesian Y velocity of person in robot frame (mm/s).
     * @param lidar_points 3D LiDAR point cloud.
     * @return Output command and arc points for visualization.
     */
    Output Update(double person_angle,
                  double person_dist,
                  double radial_vel,
                  double person_vel_x,
                  double person_vel_y,
                  const RoboCompLidar3D::TPoints& lidar_points);

    /**
     * @brief Forward kinematic model of the robot: predicts displacement after time dt
     *        given advance and rotational velocities.
     * @param adv Advance velocity (mm/s).
     * @param rot Rotational velocity (rad/s).
     * @param dt Time delta (s).
     * @return Trajectory end pose (x, y in mm, theta in rad) referenced to robot frame.
     */
    static std::tuple<double, double, double> ForwardModel(double adv, double rot, double dt);

    /**
     * @brief Computes the trajectory arc points for a given velocity command.
     * @param adv Advance velocity (mm/s).
     * @param rot Rotational velocity (rad/s).
     * @param duration Time duration of arc (s).
     * @param steps Discretization steps.
     * @return Sampled trajectory points.
     */
    std::vector<QPointF> GenerateArc(double adv, double rot, double duration, int steps) const;

private:
    Params params_;
    float last_chosen_rot_ = 0.0f; // Turn continuity across consecutive DWA cycles

    // Extracts relevant 2D static obstacle points from LiDAR cloud
    std::vector<QPointF> ExtractObstaclePoints(const RoboCompLidar3D::TPoints& points,
                                              const QPointF& person_pos) const;

    // Checks collision and computes minimum clearance along an arc
    // Returns pair<has_collision, min_distance>
    std::pair<bool, float> EvaluateObstacleClearance(const std::vector<QPointF>& arc,
                                                    const std::vector<QPointF>& obstacles,
                                                    float collision_threshold) const;
};

#endif // CONTROLLER_H
