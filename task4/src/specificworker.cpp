#include "specificworker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ranges>

namespace control_params
{
    constexpr float kTargetDistMm = 1000.0f;        // Nominal target following distance (mm)
    constexpr float kMaxAdvSpeed = 1000.0f;         // Maximum linear advance velocity (mm/s)
    constexpr float kMaxRotSpeed = 1.2f;            // Maximum angular rotational velocity (rad/s)
    constexpr float kConeHalfApertureRad = 0.45f;   // LiDAR filtering cone half-aperture (rad)
    constexpr double kAzimuthSign = 1.0;            // Camera-LiDAR azimuth projection sign
    constexpr double kAzimuthOffset = -0.1;         // Camera-LiDAR azimuth offset (rad)
    constexpr int kRobotLengthMm = 400;             // Robot visual representation size (mm)
}

namespace
{
    double GetTimeSec()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }
}

SpecificWorker::SpecificWorker(const ConfigLoader& config_loader, TuplePrx tprx, bool startup_check)
    : GenericWorker(config_loader, tprx), startup_check_flag_(startup_check)
{
    if (startup_check_flag_)
    {
        this->startup_check();
    }
    else
    {
        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        const auto error = statemachine.errorString();
        if (!error.isEmpty())
        {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker() = default;

void SpecificWorker::initialize()
{
    GenericWorker::initialize();

    const QRectF dimensions(-6000, -3000, 12000, 6000);
    viewer_ = new AbstractGraphicViewer(this->frame, dimensions);
    this->resize(900, 450);
    viewer_->show();
    viewer_->add_robot(control_params::kRobotLengthMm, control_params::kRobotLengthMm, 0, 190, QColor("Blue"));

    Controller::Params ctrl_params;
    ctrl_params.target_dist_mm = control_params::kTargetDistMm;
    ctrl_params.max_adv_speed = control_params::kMaxAdvSpeed;
    ctrl_params.max_rot_speed = control_params::kMaxRotSpeed;
    ctrl_params.min_adv_speed = 0.0f;
    ctrl_params.robot_radius_mm = 285.0f;
    ctrl_params.safety_margin_mm = 45.0f;
    ctrl_params.sim_time_sec = 1.1f;
    ctrl_params.sim_steps = 11;
    ctrl_params.min_self_chassis_dist_mm = 340.0f;
    ctrl_params.obstacle_influence_dist_mm = 1100.0f;
    ctrl_params.weight_obstacle = 1.8f;
    ctrl_params.weight_prop_alignment = 0.8f;
    controller_.SetParams(ctrl_params);
}

void SpecificWorker::compute()
{
    fps.print("Compute worker", 3000);

    const double time_now = GetTimeSec();
    const double dt_ego = (time_last_cycle_ < 0.0) ? 0.0 : std::clamp(time_now - time_last_cycle_, 0.0, 0.5);
    time_last_cycle_ = time_now;

    tracker_.EgoCompensate(last_adv_cmd_, last_rot_cmd_, dt_ego);

    RoboCompLidar3D::TData lidar_data;
    try
    {
        lidar_data = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "LiDAR Error: " << e << std::endl;
        StopRobot();
        return;
    }

    RoboCompVisualElements::TObjects visual_objects;
    try
    {
        visual_objects = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "VisualElements Error: " << e << std::endl;
        StopRobot();
        return;
    }

    RoboCompCamera360RGB::TImage img;
    try
    {
        img = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "Camera Error: " << e << std::endl;
        StopRobot();
        return;
    }

    if (img.width <= 0 || img.height <= 0)
    {
        StopRobot();
        return;
    }

    bool has_valid_measurement = false;
    double raw_angle = 0.0;
    float raw_depth = 0.0f;
    QRect roi;
    std::vector<QPointF> projected_points;

    if (!visual_objects.objects.empty())
    {
        const auto& person = visual_objects.objects.front();
        roi = QRect(person.left, person.top, person.right - person.left, person.bot - person.top);

        const double box_center = (person.left + person.right) / 2.0;
        raw_angle = (2.0 * M_PI / img.width) * (box_center - img.width / 2.0);
        raw_angle = std::remainder(raw_angle, 2.0 * M_PI);

        // Extrinsic translation from LiDAR bpearl to 360 camera (mm)
        Eigen::Isometry3d cam_transform = Eigen::Isometry3d::Identity();
        cam_transform.pretranslate(-Eigen::Vector3d(0.0, -170.0, 1275.0));

        if (const auto proj_result = ProjectLidarToCamera(lidar_data.points, cam_transform, raw_angle, img, roi,
                                                         control_params::kAzimuthSign, control_params::kAzimuthOffset);
            proj_result.has_value())
        {
            std::tie(projected_points, raw_depth) = proj_result.value();
            has_valid_measurement = true;
        }
        else
        {
            if (tracker_.IsInitialized())
            {
                const auto [_, prev_depth] = tracker_.GetPolarEstimate();
                if (prev_depth > 400.0 && prev_depth < 15000.0)
                {
                    raw_depth = static_cast<float>(prev_depth);
                    has_valid_measurement = true;
                }
            }

            if (!has_valid_measurement)
            {
                const double h_norm = static_cast<double>(roi.height()) / img.height;
                if (h_norm > 0.04)
                {
                    raw_depth = std::clamp(static_cast<float>(1700.0 / std::tan(h_norm * M_PI)), 500.0f, 10000.0f);
                    has_valid_measurement = true;
                }
            }
        }
    }

    if (has_valid_measurement)
    {
        tracker_.ProcessMeasurement(raw_angle, raw_depth, time_now);
    }
    else
    {
        if (!tracker_.StepWithoutMeasurement(dt_ego, time_now))
        {
            StopRobot();
            DrawImage(img, roi, projected_points, label);
            return;
        }
    }

    const auto [est_angle, est_dist] = tracker_.GetPolarEstimate();
    const double radial_vel = tracker_.GetRadialVelocity();
    const auto cart_vel = tracker_.GetCartesianVelocity();

    const auto ctrl_output = controller_.Update(est_angle, est_dist, radial_vel,
                                               cart_vel.x(), cart_vel.y(),
                                               lidar_data.points);

    static auto last_log_time = std::chrono::steady_clock::now();
    const auto now_log = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now_log - last_log_time).count() >= 500)
    {
        last_log_time = now_log;
        std::cout << "[task4-DWA] det=" << visual_objects.objects.size()
                  << " meas=" << has_valid_measurement
                  << " estA=" << est_angle << " estD=" << est_dist
                  << " adv=" << ctrl_output.adv_speed << " rot=" << ctrl_output.rot_speed
                  << " obsDist=" << ctrl_output.min_obs_dist
                  << " avoiding=" << ctrl_output.is_avoiding << std::endl;
    }

    try
    {
        omnirobot_proxy->setSpeedBase(0.0f, ctrl_output.adv_speed, ctrl_output.rot_speed);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "OmniRobot error: " << e << std::endl;
    }
    last_adv_cmd_ = ctrl_output.adv_speed;
    last_rot_cmd_ = ctrl_output.rot_speed;

    DrawImage(img, roi, projected_points, label);
    if (viewer_ != nullptr)
    {
        auto* scene = &viewer_->scene;
        DrawLidar(lidar_data.points, scene);
        DrawPersonInLidar(static_cast<float>(est_angle), static_cast<float>(est_dist), scene);
        DrawDwaArcs(ctrl_output, scene);
    }
}

void SpecificWorker::StopRobot()
{
    try
    {
        omnirobot_proxy->setSpeedBase(0.0f, 0.0f, 0.0f);
    }
    catch (const Ice::Exception&)
    {
    }
    last_adv_cmd_ = 0.0f;
    last_rot_cmd_ = 0.0f;
}

RoboCompLidar3D::TPoints SpecificWorker::FilterCone(const RoboCompLidar3D::TPoints& points, float min_angle,
                                                    float max_angle, float min_distance)
{
    const float center = (min_angle + max_angle) * 0.5f;
    const float half_span = std::abs(max_angle - min_angle) * 0.5f;

    auto filtered = points | std::views::filter([=](const auto& point)
    {
        if (point.distance2d <= min_distance) return false;
        const float diff = std::remainder(point.phi - center, static_cast<float>(2.0 * M_PI));
        return std::abs(diff) <= half_span;
    });

    return std::ranges::to<RoboCompLidar3D::TPoints>(filtered);
}

std::optional<std::tuple<std::vector<QPointF>, float>>
SpecificWorker::ProjectLidarToCamera(
    const RoboCompLidar3D::TPoints& points,
    const Eigen::Isometry3d& cam_transform,
    double person_angle,
    const RoboCompCamera360RGB::TImage& image,
    const QRect& roi,
    double azimuth_sign,
    double azimuth_offset)
{
    const auto filtered_cone = FilterCone(
        points,
        static_cast<float>(person_angle - control_params::kConeHalfApertureRad),
        static_cast<float>(person_angle + control_params::kConeHalfApertureRad),
        250.0f);

    std::vector<QPointF> projected_points;
    projected_points.reserve(filtered_cone.size());
    std::vector<float> distances;

    const QRect expanded_roi = roi.adjusted(-12, -12, 12, 12);

    for (const auto& lp : filtered_cone)
    {
        const Eigen::Vector3d point_lidar(lp.x, lp.y, lp.z);
        const Eigen::Vector3d point_cam = cam_transform * point_lidar;

        const double radius = point_cam.norm();
        if (radius < 100.0) continue;

        double phi_cam = std::atan2(point_cam.x(), point_cam.y());
        phi_cam = azimuth_sign * phi_cam + azimuth_offset;
        phi_cam = std::remainder(phi_cam, 2.0 * M_PI);

        const double u = (phi_cam / (2.0 * M_PI) + 0.5) * image.width;
        if (u < 0.0 || u >= image.width) continue;

        const double v = (std::asin(-point_cam.z() / radius) / M_PI + 0.5) * image.height;

        if (expanded_roi.contains(QPoint(static_cast<int>(u), static_cast<int>(v))))
        {
            projected_points.emplace_back(u, v);
            distances.push_back(static_cast<float>(radius));
        }
    }

    if (distances.empty())
    {
        for (const auto& lp : filtered_cone)
        {
            const float diff = std::abs(std::remainder(lp.phi - static_cast<float>(person_angle), static_cast<float>(2.0 * M_PI)));
            if (diff < 0.20f && lp.distance2d > 400.0f && lp.distance2d < 15000.0f)
            {
                distances.push_back(lp.distance2d);
            }
        }
    }

    if (distances.empty()) return {};
    const float measured_depth = *std::ranges::min_element(distances);

    return std::make_tuple(projected_points, measured_depth);
}

void SpecificWorker::DrawImage(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                               const std::vector<QPointF>& projected_points, QLabel* target_label)
{
    const QImage img(raw.image.data(), raw.width, raw.height, raw.width * 3, QImage::Format_RGB888);
    QImage resized = img.scaled(target_label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

    const double scale_x = static_cast<double>(resized.width()) / raw.width;
    const double scale_y = static_cast<double>(resized.height()) / raw.height;

    QPainter painter(&resized);
    QPen pen(Qt::red, 2);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.scale(scale_x, scale_y);

    if (!roi.isNull())
        painter.drawRect(roi);

    pen.setColor(Qt::blue);
    painter.setPen(pen);
    for (const auto& pt : projected_points)
        painter.drawPoint(pt);

    const double image_mid_x = raw.width / 2.0;
    const int vertical_guide_thickness = std::clamp(static_cast<int>(raw.height / 60.0), 2, 12);
    pen.setColor(Qt::green);
    pen.setWidth(vertical_guide_thickness);
    pen.setCosmetic(false);
    painter.setPen(pen);
    painter.drawLine(QPointF(image_mid_x, 0.0), QPointF(image_mid_x, raw.height));

    if (!roi.isNull())
    {
        const double roi_mid_y = roi.top() + roi.height() / 2.0;
        const double roi_left_dist = std::abs(roi.left() - image_mid_x);
        const double roi_right_dist = std::abs(roi.right() - image_mid_x);
        const double roi_closest_x = (roi_left_dist <= roi_right_dist) ? roi.left() : roi.right();
        const double roi_guide_length = std::abs(image_mid_x - roi_closest_x);
        const int horizontal_guide_thickness = std::clamp(static_cast<int>(roi_guide_length / 8.0), 2, 25);

        pen.setColor(QColor(255, 140, 0));
        pen.setWidth(horizontal_guide_thickness);
        pen.setCosmetic(false);
        painter.setPen(pen);
        painter.drawLine(QPointF(roi_closest_x, roi_mid_y), QPointF(image_mid_x, roi_mid_y));
    }

    target_label->setPixmap(QPixmap::fromImage(resized));
}

void SpecificWorker::DrawPersonInLidar(float person_angle, float distance, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> drawn_items;
    for (const auto& item : drawn_items)
    {
        scene->removeItem(item);
        delete item;
    }
    drawn_items.clear();

    const QPen pen(QColor(Qt::red), 50);
    auto* circle = scene->addEllipse(-100, -100, 200, 200, pen);
    const QPointF center(distance * std::sin(person_angle), distance * std::cos(person_angle));
    circle->setPos(center);
    drawn_items.push_back(circle);

    auto* line = scene->addLine(0, 0, center.x(), center.y(), pen);
    line->setZValue(-1);
    drawn_items.push_back(line);

    const double angle_deg = person_angle * (180.0 / M_PI);
    const QString label_text = QString("Target Person\nDist: %1mm\nAng: %2°")
                              .arg(distance, 0, 'f', 2)
                              .arg(angle_deg, 0, 'f', 1);

    static QGraphicsTextItem* person_label = nullptr;
    if (person_label == nullptr)
    {
        person_label = scene->addText(label_text);
        person_label->setTransform(QTransform().scale(1, -1));

        QFont font = person_label->font();
        font.setPixelSize(100);
        person_label->setFont(font);
        person_label->setDefaultTextColor(Qt::blue);
    }
    else
    {
        person_label->setPlainText(label_text);
    }

    person_label->setPos(center.x() + 10, center.y() - 10);
    person_label->setZValue(100);
}

void SpecificWorker::DrawLidar(const RoboCompLidar3D::TPoints& points, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> drawn_points;
    for (const auto& item : drawn_points)
    {
        scene->removeItem(item);
        delete item;
    }
    drawn_points.clear();

    const QPen pen(QColor("LightGreen"), 10);
    for (const auto& pt : points)
    {
        auto* rect = scene->addRect(-25, -25, 50, 50, pen);
        rect->setPos(pt.x, pt.y);
        drawn_points.push_back(rect);
    }
}

void SpecificWorker::DrawDwaArcs(const Controller::Output& output, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> arc_items;
    for (auto* item : arc_items)
    {
        scene->removeItem(item);
        delete item;
    }
    arc_items.clear();

    const QPen cand_pen(QColor(135, 206, 250, 50), 12);
    for (const auto& arc : output.candidate_arcs)
    {
        for (size_t i = 1; i < arc.size(); ++i)
        {
            auto* line = scene->addLine(arc[i - 1].x(), arc[i - 1].y(),
                                        arc[i].x(), arc[i].y(), cand_pen);
            arc_items.push_back(line);
        }
    }

    const QColor best_color = output.is_avoiding ? QColor(255, 140, 0) : QColor(0, 220, 0);
    QPen best_pen(best_color, 35);
    best_pen.setCapStyle(Qt::RoundCap);
    for (size_t i = 1; i < output.best_arc.size(); ++i)
    {
        auto* line = scene->addLine(output.best_arc[i - 1].x(), output.best_arc[i - 1].y(),
                                    output.best_arc[i].x(), output.best_arc[i].y(), best_pen);
        line->setZValue(10);
        arc_items.push_back(line);
    }
}

void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}
