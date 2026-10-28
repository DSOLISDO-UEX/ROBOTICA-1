/*
-    Copyright (C) 2026 by YOUR NAME HERE
 *
-    This file is part of RoboComp
 *
-    RoboComp is free software: you can redistribute it and/or modify
-    it under the terms of the GNU General Public License as published by
-    the Free Software Foundation, either version 3 of the License, or
-    (at your option) any later version.
 *
-    RoboComp is distributed in the hope that it will be useful,
-    but WITHOUT ANY WARRANTY; without even the implied warranty of
-    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-    GNU General Public License for more details.
 *
-    You should have received a copy of the GNU General Public License
-    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <cmath>
#include <iostream>

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

    RoboCompLidar3D::TData ldata;
    try { ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1); }
    catch (const Ice::Exception& e) { std::cout << e << std::endl; }

    RoboCompVisualElements::TObjects objects;
    RoboCompVisualElements::TObject person;
    try
    {
        objects = visualelements_proxy->getVisualObjects(
            RoboCompVisualElements::TObjects{});
    }
    catch (Ice::Exception& e) { std::cout << e << std::endl; }
    if (objects.objects.empty()) return;
    person = objects.objects.front();
    QRect roi = {person.left, person.top, person.right - person.left, person.bot - person.top};

    RoboCompCamera360RGB::TImage img;
    try
    {
        img = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
    }
    catch (Ice::Exception& e) { std::cout << e << std::endl; }

    const double boxCenter = (person.left + person.right) / 2.0;
    double personAngle = (2.0 * M_PI / img.width) * (boxCenter - img.width / 2.f);

    Eigen::Isometry3d camTransform = Eigen::Isometry3d::Identity();
    camTransform.pretranslate(-Eigen::Vector3d(0.f, -170.f, 1275.f));
    std::tuple<std::vector<QPointF>, float> projection;
    if (auto res = project_lidar_to_camera(ldata.points, camTransform, personAngle, img, roi); !res.has_value())
    {
        //std::cout << "No lidar projection" << std::endl;
        //return;
    }
    else
    {
        projection = res.value();
    }


    const auto& [prjPoints, depth] = projection;
    draw_image(img, roi, {}, label);


    draw_person_in_lidar(float(personAngle), depth, &viewer->scene);
    draw_lidar(ldata.points, &viewer->scene);

    float target_rot = 0.f;
    float adv = 0.f;
    bool person_detected = false;

    const double imgWidth = 1920.0;

    if (!objects.objects.empty())
    {
        const auto& obj = objects.objects.front();

        // Centro X en la imagen 360°
        double centroPersonaX = (obj.left + obj.right) / 2.0;
        double deltaX = centroPersonaX - (imgWidth / 2.0);

        // 1. Proyección lineal equirrectangular (360° = 2 * PI rad)
        // deltaX va de -480 a +480, lo que mapea linealmente de -PI a +PI rad
        double yawRad = deltaX * (2.0 * M_PI / imgWidth);
        double yawDeg = yawRad * (180.0 / M_PI);

        // 2. Parámetros del Controlador Proporcional Suavizado
        const float Kp = 1.0f; // Ganancia proporcional (1/s)
        const float MAX_ROT_SPEED = 1.2f; // Límite de velocidad en rad/s (~68 deg/s)
        const double DEADBAND_DEG = 2.0; // Margen de histeresis para evitar micro-ajustes

        if (std::abs(yawDeg) > DEADBAND_DEG)
        {
            // Invertimos o mantenemos el signo según el sentido de rotación de tu base
            target_rot = -Kp * static_cast<float>(yawRad);
            target_rot = std::clamp(target_rot, -MAX_ROT_SPEED, MAX_ROT_SPEED);
        }
        else
        {
            target_rot = 0.f;
        }

        person_detected = true;
    }

    if (person_detected)
    {
        omnirobot_proxy->setSpeedBase(0.f, 0.f, target_rot);
    }
}

void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << p;
}

RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints& points, float d)
// set to 200mm
{
    if (points.empty()) return {};

    const float d_squared = d * d; // Avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index vector for parallel iteration
    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), size_t{0});

    // Parallelize outer loop - each thread checks one point
    std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
    {
        const auto& p1 = points[i];
        // Sequential inner loop (avoid nested parallelism)
        for (auto&& [j,p2] : iter::enumerate(points))
        {
            if (i == j) continue;
            const float dx = p1.x - p2.x;
            const float dy = p1.y - p2.y;
            if (dx * dx + dy * dy <= d_squared)
            {
                hasNeighbor[i] = true;
                break;
            }
        }
    });

    // Collect results
    std::vector<RoboCompLidar3D::TPoint> result;
    result.reserve(points.size());
    for (auto&& [i, p] : iter::enumerate(points))
        if (hasNeighbor[i])
            result.push_back(points[i]);
    return result;
}

auto SpecificWorker::cono(const RoboCompLidar3D::TPoints& points, float minAngle,
                          float maxAngle, float distance)
{
    return points | std::views::filter([=, this](const auto& point)
    {
        bool inAngle = (point.phi > minAngle) and (point.phi < maxAngle);
        if (not inAngle) return false;

        if (distance > 0)
            return (point.distance2d < distance) && (point.distance2d > this->ROBOT_LENGTH / 2.1);
        return true;
    });
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
    };


    std::vector<QPointF> projected_points;
    projected_points.reserve(points.size());
    std::vector<float> distances;


    for (const auto& lp : filtered_cone)
    {
        // Convert Lidar point to Eigen vector
        Eigen::Vector3d p_lidar(lp.x, lp.y, lp.z);


        // Transform point to Camera Local Frame
        // P_cam = R_inv * (P_lidar - t)
        Eigen::Vector3d p_cam = cam_transform * p_lidar;


        // Equirectangular Projection
        const double r = p_cam.norm();
        if (r < 1e-9)
        {
            projected_points.emplace_back(image.width / 2.0, image.height / 2.0);
            continue;
        }


        // Azimuth (u)
        double u = ((azimuth_sign * std::atan2(p_cam.x(), p_cam.y()) + azimuth_offset) / (2.0 * M_PI) + 0.5) * image.
            width;
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
        Qt::SmoothTransformation
    );


    double sx = double(resized.width()) / raw.width;
    double sy = double(resized.height()) / raw.height;


    QPainter painter(&resized);


    QPen pen(Qt::red, 2);
    pen.setCosmetic(true);
    painter.setPen(pen);


    painter.scale(sx, sy);


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
    // compute center from angle and dist
    auto cx = distance * sin(person_angle + 0.1);
    auto cy = distance * cos(person_angle);
    QPointF center{cx, cy};
    circ->setPos(center);
    draw_points.push_back(circ);


    auto line = scene->addLine(0, 0, center.x(), center.y(), pen);
    line->setZValue(-1); // Ensure the line is behind the person polygon
    draw_points.push_back(line);


    // Add text label with distance and angle
    double angle_deg = person_angle * (180.0 / M_PI);
    QString labelText = QString("Target Person\nDist: %1mm\nAng: %2°")
                        .arg(distance, 0, 'f', 2)
                        .arg(angle_deg, 0, 'f', 1);


    static QGraphicsTextItem* m_personLabel = nullptr; // Persistent label across calls
    // Create or Update the Label
    if (m_personLabel == nullptr)
    {
        // Create the item once and add it to the scene
        m_personLabel = scene->addText(labelText);

        // --- FIX FOR FLIPPED Y-AXIS ---
        // This cancels out the scene's scale(1, -1) transform
        m_personLabel->setTransform(QTransform().scale(1, -1));
        // -----------------------------


        // Set a visible style
        QFont font = m_personLabel->font();
        font.setPixelSize(100);
        //font.setBold(true);
        m_personLabel->setFont(font);
        m_personLabel->setDefaultTextColor(Qt::blue);

        // Add a background box for readability (optional but recommended)
        // Since QGraphicsTextItem doesn't have a background, we wrap it in a GraphicsItem
        // or just set a semi-transparent background via a separate rectangle if needed.
    }
    else // Update existing item's text
        m_personLabel->setPlainText(labelText);


    // Update Position (slightly offset from the point)
    m_personLabel->setPos(center.x() + 10, center.y() - 10);

    // Ensure the label is always on top of other Lidar points
    m_personLabel->setZValue(100);
}

std::tuple<float, float> SpecificWorker::navigation(const RoboCompLidar3D::TPoints& points)
{
    float minDistance = 400.f;
    static float rot = 0.f;
    float adv = 500;

    switch (state)
    {
    case RobotState::Rotate:
        {
            auto myPoints = cono(points, -(M_PI / 4), (M_PI / 4), 700);
            if (myPoints.empty())
            {
                state = RobotState::Advance;
                return std::make_tuple(adv, 0);
            }

            std::cout << "Rotate worker" << std::endl;
            return std::make_tuple(0, rot);
        }
    case RobotState::Advance:
        {
            auto myPoints = cono(points, -(M_PI / 4), (M_PI / 4), 700);
            if (!myPoints.empty())
            {
                auto min = myPoints.front();
                if (min.phi < 0.f)
                {
                    rot = -1.f;
                }
                else
                {
                    rot = 1.f;
                }

                state = RobotState::Rotate;
                return std::make_tuple(0, rot);
            }

            std::cout << "Advance worker" << std::endl;
            return std::make_tuple(adv, 0);
        }
    }

    return std::make_tuple(0, 0);
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
    //const QBrush brush(color, Qt::SolidPattern);
    for (const auto& p : points)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp); // add to the list of points to be deleted next time
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
