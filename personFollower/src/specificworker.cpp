/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>

namespace
{
    double get_time_sec()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }
}

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(
    configLoader, tprx)
{
    this->startup_check_flag = startup_check;
    if (this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0)
        {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
    GenericWorker::initialize();

    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
    this->resize(900, 450);
    viewer->show();
    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}

void SpecificWorker::compute()
{
    fps.print("Compute worker", 3000);

    const double t_now = get_time_sec();
    const double dt_ego = (t_last_cycle < 0.0) ? 0.0 : std::clamp(t_now - t_last_cycle, 0.0, 0.5);
    t_last_cycle = t_now;

    // 1. Ego-motion compensation before prediction/update
    tracker.ego_compensate(last_adv_cmd, last_rot_cmd, dt_ego);

    RoboCompLidar3D::TData ldata;
    try
    {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "LiDAR Error: " << e << std::endl;
        stopRobot();
        return;
    }

    RoboCompVisualElements::TObjects objects;
    try
    {
        objects = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "VisualElements Error: " << e << std::endl;
        stopRobot();
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
        stopRobot();
        return;
    }

    if (img.width <= 0 || img.height <= 0)
    {
        stopRobot();
        return;
    }

    bool has_valid_measurement = false;
    double raw_angle = 0.0;
    float raw_depth = 0.0f;
    QRect roi;
    std::vector<QPointF> prjPoints;

    if (!objects.objects.empty())
    {
        const auto& person = objects.objects.front();
        roi = QRect(person.left, person.top, person.right - person.left, person.bot - person.top);

        const double boxCenter = (person.left + person.right) / 2.0;
        raw_angle = (2.0 * M_PI / img.width) * (boxCenter - img.width / 2.0);

        Eigen::Isometry3d camTransform = Eigen::Isometry3d::Identity();
        camTransform.pretranslate(-Eigen::Vector3d(0.f, -170.f, 1275.f));

        if (const auto res_ = project_lidar_to_camera(ldata.points, camTransform, raw_angle, img, roi); res_.has_value())
        {
            std::tie(prjPoints, raw_depth) = res_.value();
            has_valid_measurement = true;
        }
        else
        {
            qWarning() << "No lidar projection";
        }
    }

    // 2. Kalman Filter Predict & Update
    if (has_valid_measurement)
    {
        tracker.process_measurement(raw_angle, static_cast<double>(raw_depth), t_now);
    }
    else if (!tracker.step_without_measurement(dt_ego, t_now))
    {
        stopRobot();
        return;
    }

    const auto [est_angle, est_depth] = tracker.get_polar_estimate();

    draw_image(img, roi, prjPoints, label);
    draw_person_in_lidar(static_cast<float>(est_angle), static_cast<float>(est_depth), &viewer->scene);
    draw_lidar(ldata.points, &viewer->scene);

    // 3. Controller using Kalman state (position + velocity feed-forward for 1000 mm tracking)
    constexpr float TARGET_DIST_MM = 1000.0f; // Target following distance: 1.0 m
    constexpr float MAX_ADV_SPEED = 1200.0f;  // Max linear speed (mm/s)
    constexpr float MAX_ROT_SPEED = 2.0f;     // Max angular speed (rad/s)
    constexpr float KP_DIST = 1.0f;           // Linear P gain
    constexpr float KP_ROT = 1.5f;            // Angular P gain
    constexpr float KD_VEL = 0.85f;           // Velocity feed-forward gain

    float dist_error = static_cast<float>(est_depth) - TARGET_DIST_MM;
    double control_angle = est_angle;
    const double radial_vel = tracker.get_radial_velocity();

    // Small deadbands to avoid jitter at steady state
    if (std::abs(dist_error) < 10.0f && std::abs(radial_vel) < 30.0)
        dist_error = 0.0f;
    if (std::abs(control_angle) < 0.02)
        control_angle = 0.0;

    // Slow down linear advance if the target is far off-center
    const float alignment_factor = std::max(0.0f, static_cast<float>(std::cos(control_angle)));

    float adv_speed = (dist_error * KP_DIST + static_cast<float>(radial_vel) * KD_VEL) * alignment_factor;
    adv_speed = std::clamp(adv_speed, -MAX_ADV_SPEED, MAX_ADV_SPEED);

    float rot_speed = std::clamp(static_cast<float>(-control_angle * KP_ROT), -MAX_ROT_SPEED, MAX_ROT_SPEED);

    omnirobot_proxy->setSpeedBase(0.0f, adv_speed, rot_speed);
    last_adv_cmd = adv_speed;
    last_rot_cmd = rot_speed;
}

void SpecificWorker::stopRobot()
{
    omnirobot_proxy->setSpeedBase(0.0f, 0.f, 0.f);
    last_adv_cmd = 0.0f;
    last_rot_cmd = 0.0f;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << p;
}

RoboCompLidar3D::TPoints SpecificWorker::cono(const RoboCompLidar3D::TPoints& points, float minAngle, float maxAngle,
                                              float distance)
{
    // Filter points based on the specified angle range and distance
    auto filtered = points | std::views::filter([=](const auto& point)
    {
        return point.phi >= minAngle && point.phi <= maxAngle && point.distance2d > distance;
    });

    // Materialize view to vector to match return type RoboCompLidar3D::TPoints
    return std::ranges::to<RoboCompLidar3D::TPoints>(filtered);
}

std::optional<std::tuple<std::vector<QPointF>, float>>
SpecificWorker::project_lidar_to_camera(
    const RoboCompLidar3D::TPoints& points,
    const Eigen::Isometry3d& cam_transform,
    double personAngle,
    const RoboCompCamera360RGB::TImage& image,
    const QRect& roi,
    double azimuth_sign,
    double azimuth_offset)
{
    // filter points based on the specified angle range and distance
    auto filtered_cone = cono(points, personAngle - 0.3, personAngle + 0.3);
    if (filtered_cone.empty())
    {
        qWarning() << "Empty cone";
        return {};
    }

    std::vector<QPointF> projected_points;
    projected_points.reserve(points.size());
    std::vector<float> distances;

    for (const auto& lp : filtered_cone)
    {
        // Convert Lidar point to Eigen vector
        Eigen::Vector3d p_lidar(lp.x, lp.y, lp.z);

        // Transform point to Camera Local Frame
        Eigen::Vector3d p_cam = cam_transform * p_lidar;

        // Equirectangular Projection
        const double r = p_cam.norm();
        if (r < 1e-9)
        {
            projected_points.emplace_back(image.width / 2.0, image.height / 2.0);
            continue;
        }

        // Azimuth (u)
        double u = ((azimuth_sign * std::atan2(p_cam.x(), p_cam.y()) + azimuth_offset) / (2.0 * M_PI) + 0.5) * image.width;
        u = std::fmod(u, static_cast<double>(image.width));
        if (u < 0.0) u += image.width;

        // Elevation (v)
        const double v = (std::asin(-p_cam.z() / r) / M_PI + 0.5) * image.height;

        // Check if the projected point is within the person's bounding box
        if (roi.contains(QPoint(u, v)))
        {
            projected_points.emplace_back(u, v);
            distances.push_back(r);
        }
    }

    // compute the minimal depth of the projected points within the bounding box
    if (distances.empty()) return {};
    float measured_depth = *std::ranges::min_element(distances);

    return std::make_tuple(projected_points, measured_depth);
}

void SpecificWorker::draw_image(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                                const std::vector<QPointF>& projected_points, QLabel* label)
{
    QImage img(raw.image.data(),
               raw.width,
               raw.height,
               raw.width * 3,
               QImage::Format_RGB888);

    QSize targetSize = label->size();

    QImage resized = img.scaled(
        targetSize,
        Qt::IgnoreAspectRatio,
        Qt::SmoothTransformation);

    double sx = double(resized.width()) / raw.width;
    double sy = double(resized.height()) / raw.height;

    QPainter painter(&resized);

    QPen pen(Qt::red, 2);
    pen.setCosmetic(true);
    painter.setPen(pen);

    painter.scale(sx, sy);

    if (!roi.isNull())
        painter.drawRect(roi);

    // Draw projected points
    pen.setColor(Qt::blue);
    painter.setPen(pen);
    for (const auto& pt : projected_points)
        painter.drawPoint(pt);

    // Draw a vertical line at mid-width with thickness proportional to image height
    const double image_mid_x = raw.width / 2.0;
    const int vertical_guide_thickness = std::clamp<int>(static_cast<int>(raw.height / 60.0), 2, 12);
    pen.setColor(Qt::green);
    pen.setWidth(vertical_guide_thickness);
    pen.setCosmetic(false);
    painter.setPen(pen);
    painter.drawLine(QPointF(image_mid_x, 0.0), QPointF(image_mid_x, raw.height));

    if (!roi.isNull())
    {
        // Draw a horizontal line from the closest ROI side to the vertical guide
        const double roi_mid_y = roi.top() + roi.height() / 2.0;
        const double roi_left_dist = std::abs(roi.left() - image_mid_x);
        const double roi_right_dist = std::abs(roi.right() - image_mid_x);
        const double roi_closest_x = (roi_left_dist <= roi_right_dist) ? roi.left() : roi.right();
        const double roi_guide_length = std::abs(image_mid_x - roi_closest_x);
        const int horizontal_guide_thickness = std::clamp<int>(static_cast<int>(roi_guide_length / 8.0), 2, 25);
        pen.setColor(QColor(255, 140, 0));
        pen.setWidth(horizontal_guide_thickness);
        pen.setCosmetic(false);
        painter.setPen(pen);
        painter.drawLine(QPointF(roi_closest_x, roi_mid_y), QPointF(image_mid_x, roi_mid_y));
    }

    label->setPixmap(QPixmap::fromImage(resized));
}

void SpecificWorker::draw_person_in_lidar(float person_angle, float distance, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> draw_points;
    for (const auto& p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color(Qt::red);
    const QPen pen(color, 50);

    auto circ = scene->addEllipse(-100, -100, 200, 200, pen);
    auto cx = distance * sin(person_angle);
    auto cy = distance * cos(person_angle);
    QPointF center{cx, cy};
    circ->setPos(center);
    draw_points.push_back(circ);

    auto line = scene->addLine(0, 0, center.x(), center.y(), pen);
    line->setZValue(-1);
    draw_points.push_back(line);

    double angle_deg = person_angle * (180.0 / M_PI);
    QString labelText = QString("Target Person\nDist: %1mm\nAng: %2°")
                        .arg(distance, 0, 'f', 2)
                        .arg(angle_deg, 0, 'f', 1);

    static QGraphicsTextItem* m_personLabel = nullptr;
    if (m_personLabel == nullptr)
    {
        m_personLabel = scene->addText(labelText);
        m_personLabel->setTransform(QTransform().scale(1, -1));

        QFont font = m_personLabel->font();
        font.setPixelSize(100);
        m_personLabel->setFont(font);
        m_personLabel->setDefaultTextColor(Qt::blue);
    }
    else
    {
        m_personLabel->setPlainText(labelText);
    }

    m_personLabel->setPos(center.x() + 10, center.y() - 10);
    m_personLabel->setZValue(100);
}

void SpecificWorker::draw_lidar(const auto& points, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> draw_points;
    for (const auto& p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen pen(color, 10);
    for (const auto& p : points)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp);
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
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}