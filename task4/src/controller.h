#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <vector>
#include <tuple>
#include <utility>
#include <cmath>
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
        float targetDistMm = 1000.0f;       // Nominal following distance (mm)
        float maxAdvSpeed = 1000.0f;        // Max advance speed (mm/s)
        float minAdvSpeed = 0.0f;           // Min forward advance speed (mm/s)
        float maxRotSpeed = 1.0f;           // Max rotational speed (rad/s)
        float robotRadiusMm = 280.0f;       // Robot effective radius (mm)
        float safetyMarginMm = 65.0f;       // Extra clearance margin (mm)

        // DWA Simulation Settings
        float simTimeSec = 1.2f;            // Forward projection horizon (s)
        int simSteps = 6;                   // Arc discretization steps along trajectory
        int advSamples = 17;                // Discretization samples for advance speed (0..1000)
        int rotSamples = 25;                // Discretization samples for rotational speed (-1..1)

        // Proportional Controller Baseline
        float kpDist = 0.9f;                // Linear gain
        float kpRot = 2.2f;                 // Angular steering gain
        float kdVel = 0.85f;                // Radial velocity feedforward gain

        // Cost Function Weights
        float weightTargetDist = 1.2f;      // Term: end of arc at 1m from person
        float weightHeading = 0.8f;         // Term: end of arc faces the person
        float weightObstacle = 2.0f;        // Term: distance to obstacles (avoidance priority)
        float weightSpeed = 0.35f;          // Term: prefer fast movements
        float weightDistToPerson = 0.5f;    // Term: close distance to person when far
        float weightPropAlignment = 0.5f;   // Bias towards proportional controller

        // Clearance thresholds
        float obstacleInfluenceDistMm = 1200.0f; // Range where obstacle creates repulsive cost
        float personBodyRadiusMm = 450.0f;       // Radius around person centroid excluded from obstacles
    };

    struct Output
    {
        float advSpeed = 0.0f;              // Commanded advance velocity (mm/s)
        float rotSpeed = 0.0f;              // Commanded rotational velocity (rad/s)
        bool isAvoiding = false;            // True if DWA deviates from proportional tracking
        float minObsDist = 99999.0f;        // Minimum obstacle clearance along trajectory
        std::vector<QPointF> bestArc;       // Trajectory points of selected arc
        std::vector<std::vector<QPointF>> candidateArcs; // Sample of candidate arcs for visualization
    };

    Controller();
    explicit Controller(const Params& params);

    void setParams(const Params& params);
    const Params& getParams() const;

    /**
     * @brief Computes advance and rotational velocities using DWA and proportional control.
     * @param personAngle Polar angle towards person in robot frame (rad).
     * @param personDist Distance towards person in robot frame (mm).
     * @param radialVel Estimated radial velocity of person (mm/s).
     * @param personVelX Estimated Cartesian X velocity of person in robot frame (mm/s).
     * @param personVelY Estimated Cartesian Y velocity of person in robot frame (mm/s).
     * @param lidarPoints 3D LiDAR point cloud.
     * @return Output command and arc points for visualization.
     */
    Output update(double personAngle,
                  double personDist,
                  double radialVel,
                  double personVelX,
                  double personVelY,
                  const RoboCompLidar3D::TPoints& lidarPoints);

    /**
     * @brief Forward kinematic model of the robot: predicts displacement after time dt
     *        given advance and rotational velocities.
     * @param adv Advance velocity (mm/s).
     * @param rot Rotational velocity (rad/s).
     * @param dt Time delta (s).
     * @return Trajectory end pose (x, y in mm, theta in rad) referenced to robot frame.
     */
    static std::tuple<double, double, double> forwardModel(double adv, double rot, double dt);

    /**
     * @brief Computes the trajectory arc points for a given velocity command.
     */
    std::vector<QPointF> generateArc(double adv, double rot, double duration, int steps) const;

private:
    Params params_;

    // Extracts relevant 2D static obstacle points from LiDAR cloud
    std::vector<QPointF> extractObstaclePoints(const RoboCompLidar3D::TPoints& points,
                                              const QPointF& personPos) const;

    // Checks collision and computes minimum clearance along an arc
    // Returns pair<hasCollision, minDistance>
    std::pair<bool, float> evaluateObstacleClearance(const std::vector<QPointF>& arc,
                                                    const std::vector<QPointF>& obstacles,
                                                    float collisionThreshold) const;
};

#endif // CONTROLLER_H
