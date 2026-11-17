#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#ifdef emit
#undef emit
#endif

#include <optional>
#include <tuple>
#include <vector>

#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <genericworker.h>
#include <Eigen/Dense>

#include "controller.h"
#include "personTracker.h"

/**
 * @brief Main RoboComp worker component for person tracking and DWA obstacle avoidance.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT

public:
    /**
     * @brief Constructs SpecificWorker component.
     * @param config_loader Configuration loader for component.
     * @param tprx RoboComp communication proxies tuple.
     * @param startup_check Flag indicating initial diagnostic startup check mode.
     */
    SpecificWorker(const ConfigLoader& config_loader, TuplePrx tprx, bool startup_check);

    /**
     * @brief Destructs SpecificWorker.
     */
    ~SpecificWorker() override;

public slots:
    /**
     * @brief Initializes graphical viewer and robot scene geometry.
     */
    void initialize() override;

    /**
     * @brief Main compute loop: compensates egomotion, fuses sensor data, and executes DWA control.
     */
    void compute() override;

    /**
     * @brief Handles component emergency state.
     */
    void emergency() override;

    /**
     * @brief Restores component from emergency state.
     */
    void restore() override;

    /**
     * @brief Executes component startup diagnostic check.
     * @return Status code (0 for success).
     */
    int startup_check();

private:
    bool startup_check_flag_ = false;
    AbstractGraphicViewer* viewer_ = nullptr;
    PersonTracker tracker_;
    Controller controller_;
    double time_last_cycle_ = -1.0;
    float last_adv_cmd_ = 0.0f;
    float last_rot_cmd_ = 0.0f;

    /**
     * @brief Commands omnidirectional base to zero velocity and resets command records.
     */
    void StopRobot();

    /**
     * @brief Filters 3D LiDAR point cloud within an angular cone and minimum distance.
     * @param points Input LiDAR point cloud.
     * @param min_angle Minimum cone angle (rad).
     * @param max_angle Maximum cone angle (rad).
     * @param min_distance Minimum 2D radial distance (mm).
     * @return Filtered points inside cone.
     */
    RoboCompLidar3D::TPoints FilterCone(const RoboCompLidar3D::TPoints& points, float min_angle,
                                        float max_angle, float min_distance = 0.0f);

    /**
     * @brief Projects LiDAR points onto 360 equirectangular camera image and finds target ROI depth.
     */
    std::optional<std::tuple<std::vector<QPointF>, float>> ProjectLidarToCamera(
        const RoboCompLidar3D::TPoints& points,
        const Eigen::Isometry3d& cam_transform,
        double person_angle,
        const RoboCompCamera360RGB::TImage& image,
        const QRect& roi,
        double azimuth_sign,
        double azimuth_offset);

    /**
     * @brief Renders 360 camera image with person ROI, centering guides, and projected points.
     */
    void DrawImage(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                   const std::vector<QPointF>& projected_points, QLabel* target_label);

    /**
     * @brief Visualizes estimated person location and metrics in 2D graphic scene.
     */
    void DrawPersonInLidar(float person_angle, float distance, QGraphicsScene* scene);

    /**
     * @brief Visualizes LiDAR point cloud returns in 2D graphic scene.
     */
    void DrawLidar(const RoboCompLidar3D::TPoints& points, QGraphicsScene* scene);

    /**
     * @brief Visualizes candidate and optimal trajectory arcs in 2D graphic scene.
     */
    void DrawDwaArcs(const Controller::Output& output, QGraphicsScene* scene);
};

#endif // SPECIFICWORKER_H
