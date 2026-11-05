#include "specificworker.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <ranges>

// Parámetros globales de control y percepción para ajuste del agente
namespace ControlParams
{
    float targetDistMm = 890.0f;         // Distancia objetivo ajustada (mm) [explora el mínimo en 885 mm]
    float maxAdvSpeed = 1200.0f;         // Velocidad lineal máxima (mm/s)
    float maxRotSpeed = 2.0f;            // Velocidad angular máxima (rad/s)
    float kpDist = 1.0f;                 // Ganancia proporcional lineal estable de línea base
    float kpRot = 5.0f;                  // Ganancia proporcional angular
    float kdVel = 0.85f;                 // Ganancia feed-forward de velocidad radial
    float distDeadbandMm = 5.0f;         // Zona muerta de error en distancia (mm)
    double velDeadbandMmS = 20.0;        // Zona muerta de velocidad radial (mm/s)
    double angleDeadbandRad = 0.01;      // Zona muerta de error angular (rad)
    float coneHalfApertureRad = 0.3f;    // Semiapertura del cono de filtrado LiDAR (rad)
    double azimuthSign = 1.0;            // Signo de proyección acimutal cámara-LiDAR
    double azimuthOffset = -0.1;         // Offset acimutal cámara-LiDAR (rad)
    int robotLengthMm = 400;             // Tamaño visual del robot en el visor (mm)
}

namespace
{
    double GetTimeSec()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }
}

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startupCheck)
    : GenericWorker(configLoader, tprx), startupCheckFlag_(startupCheck)
{
    if (this->startupCheckFlag_)
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

    dimensions_ = QRectF(-6000, -3000, 12000, 6000);
    viewer_ = new AbstractGraphicViewer(this->frame, dimensions_);
    this->resize(900, 450);
    viewer_->show();
    const auto rob = viewer_->add_robot(ControlParams::robotLengthMm, ControlParams::robotLengthMm, 0, 190, QColor("Blue"));
    robotPolygon_ = std::get<0>(rob);
}

void SpecificWorker::compute()
{
    fps.print("Compute worker", 3000);

    const double timeNow = GetTimeSec();
    const double dtEgo = (timeLastCycle_ < 0.0) ? 0.0 : std::clamp(timeNow - timeLastCycle_, 0.0, 0.5);
    timeLastCycle_ = timeNow;

    tracker_.EgoCompensate(lastAdvCmd_, lastRotCmd_, dtEgo);

    RoboCompLidar3D::TData lidarData;
    try
    {
        lidarData = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "LiDAR Error: " << e << std::endl;
        StopRobot();
        return;
    }

    RoboCompVisualElements::TObjects visualObjects;
    try
    {
        visualObjects = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});
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

    bool hasValidMeasurement = false;
    double rawAngle = 0.0;
    float rawDepth = 0.0f;
    QRect roi;
    std::vector<QPointF> projectedPoints;

    if (!visualObjects.objects.empty())
    {
        const auto& person = visualObjects.objects.front();
        roi = QRect(person.left, person.top, person.right - person.left, person.bot - person.top);

        const double boxCenter = (person.left + person.right) / 2.0;
        rawAngle = (2.0 * M_PI / img.width) * (boxCenter - img.width / 2.0);

        // Traslación extrínseca desde el LiDAR bpearl hasta la cámara 360 (mm)
        Eigen::Isometry3d camTransform = Eigen::Isometry3d::Identity();
        camTransform.pretranslate(-Eigen::Vector3d(0.0, -170.0, 1275.0));

        if (const auto projResult = ProjectLidarToCamera(lidarData.points, camTransform, rawAngle, img, roi,
                                                         ControlParams::azimuthSign, ControlParams::azimuthOffset);
            projResult.has_value())
        {
            std::tie(projectedPoints, rawDepth) = projResult.value();
            hasValidMeasurement = true;
        }
    }

    if (hasValidMeasurement)
    {
        tracker_.ProcessMeasurement(rawAngle, static_cast<double>(rawDepth), timeNow);
    }
    else if (!tracker_.StepWithoutMeasurement(dtEgo, timeNow))
    {
        StopRobot();
        return;
    }

    const auto [estAngle, estDepth] = tracker_.GetPolarEstimate();

    DrawImage(img, roi, projectedPoints, label);
    DrawPersonInLidar(static_cast<float>(estAngle), static_cast<float>(estDepth), &viewer_->scene);
    DrawLidar(lidarData.points, &viewer_->scene);

    float distError = static_cast<float>(estDepth) - ControlParams::targetDistMm;
    double controlAngle = estAngle;
    const double radialVel = tracker_.GetRadialVelocity();

    if (std::abs(distError) < ControlParams::distDeadbandMm && std::abs(radialVel) < ControlParams::velDeadbandMmS)
        distError = 0.0f;
    if (std::abs(controlAngle) < ControlParams::angleDeadbandRad)
        controlAngle = 0.0;

    // Atenúa el avance lineal proporcionalmente al coseno del desfase angular para priorizar el centrado
    const float alignmentFactor = std::max(0.0f, static_cast<float>(std::cos(controlAngle)));

    float advSpeed = (distError * ControlParams::kpDist + static_cast<float>(radialVel) * ControlParams::kdVel) * alignmentFactor;
    advSpeed = std::clamp(advSpeed, -ControlParams::maxAdvSpeed, ControlParams::maxAdvSpeed);

    float rotSpeed = std::clamp(static_cast<float>(-controlAngle * ControlParams::kpRot),
                                -ControlParams::maxRotSpeed, ControlParams::maxRotSpeed);

    omnirobot_proxy->setSpeedBase(0.0f, advSpeed, rotSpeed);
    lastAdvCmd_ = advSpeed;
    lastRotCmd_ = rotSpeed;
}

void SpecificWorker::StopRobot()
{
    omnirobot_proxy->setSpeedBase(0.0f, 0.0f, 0.0f);
    lastAdvCmd_ = 0.0f;
    lastRotCmd_ = 0.0f;
}

RoboCompLidar3D::TPoints SpecificWorker::FilterCone(const RoboCompLidar3D::TPoints& points, float minAngle,
                                                    float maxAngle, float minDistance)
{
    auto filtered = points | std::views::filter([=](const auto& point)
    {
        return point.phi >= minAngle && point.phi <= maxAngle && point.distance2d > minDistance;
    });

    return std::ranges::to<RoboCompLidar3D::TPoints>(filtered);
}

std::optional<std::tuple<std::vector<QPointF>, float>>
SpecificWorker::ProjectLidarToCamera(
    const RoboCompLidar3D::TPoints& points,
    const Eigen::Isometry3d& camTransform,
    double personAngle,
    const RoboCompCamera360RGB::TImage& image,
    const QRect& roi,
    double azimuthSign,
    double azimuthOffset)
{
    const auto filteredCone = FilterCone(points,
                                         static_cast<float>(personAngle) - ControlParams::coneHalfApertureRad,
                                         static_cast<float>(personAngle) + ControlParams::coneHalfApertureRad);
    if (filteredCone.empty()) return {};

    std::vector<QPointF> projectedPoints;
    projectedPoints.reserve(filteredCone.size());
    std::vector<float> distances;

    for (const auto& lp : filteredCone)
    {
        const Eigen::Vector3d pointLidar(lp.x, lp.y, lp.z);
        const Eigen::Vector3d pointCam = camTransform * pointLidar;

        const double radius = pointCam.norm();
        if (radius < 1e-9)
        {
            projectedPoints.emplace_back(image.width / 2.0, image.height / 2.0);
            continue;
        }

        // Proyección esférica equirectangular (u: acimut, v: elevación)
        double u = ((azimuthSign * std::atan2(pointCam.x(), pointCam.y()) + azimuthOffset) / (2.0 * M_PI) + 0.5) * image.width;
        u = std::fmod(u, static_cast<double>(image.width));
        if (u < 0.0) u += image.width;

        const double v = (std::asin(-pointCam.z() / radius) / M_PI + 0.5) * image.height;

        if (roi.contains(QPoint(static_cast<int>(u), static_cast<int>(v))))
        {
            projectedPoints.emplace_back(u, v);
            distances.push_back(static_cast<float>(radius));
        }
    }

    if (distances.empty()) return {};
    const float measuredDepth = *std::ranges::min_element(distances);

    return std::make_tuple(projectedPoints, measuredDepth);
}

void SpecificWorker::DrawImage(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                               const std::vector<QPointF>& projectedPoints, QLabel* targetLabel)
{
    const QImage img(raw.image.data(), raw.width, raw.height, raw.width * 3, QImage::Format_RGB888);
    QImage resized = img.scaled(targetLabel->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

    const double scaleX = static_cast<double>(resized.width()) / raw.width;
    const double scaleY = static_cast<double>(resized.height()) / raw.height;

    QPainter painter(&resized);
    QPen pen(Qt::red, 2);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.scale(scaleX, scaleY);

    if (!roi.isNull())
        painter.drawRect(roi);

    pen.setColor(Qt::blue);
    painter.setPen(pen);
    for (const auto& pt : projectedPoints)
        painter.drawPoint(pt);

    const double imageMidX = raw.width / 2.0;
    const int verticalGuideThickness = std::clamp(static_cast<int>(raw.height / 60.0), 2, 12);
    pen.setColor(Qt::green);
    pen.setWidth(verticalGuideThickness);
    pen.setCosmetic(false);
    painter.setPen(pen);
    painter.drawLine(QPointF(imageMidX, 0.0), QPointF(imageMidX, raw.height));

    if (!roi.isNull())
    {
        const double roiMidY = roi.top() + roi.height() / 2.0;
        const double roiLeftDist = std::abs(roi.left() - imageMidX);
        const double roiRightDist = std::abs(roi.right() - imageMidX);
        const double roiClosestX = (roiLeftDist <= roiRightDist) ? roi.left() : roi.right();
        const double roiGuideLength = std::abs(imageMidX - roiClosestX);
        const int horizontalGuideThickness = std::clamp(static_cast<int>(roiGuideLength / 8.0), 2, 25);

        pen.setColor(QColor(255, 140, 0));
        pen.setWidth(horizontalGuideThickness);
        pen.setCosmetic(false);
        painter.setPen(pen);
        painter.drawLine(QPointF(roiClosestX, roiMidY), QPointF(imageMidX, roiMidY));
    }

    targetLabel->setPixmap(QPixmap::fromImage(resized));
}

void SpecificWorker::DrawPersonInLidar(float personAngle, float distance, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> drawnItems;
    for (const auto& item : drawnItems)
    {
        scene->removeItem(item);
        delete item;
    }
    drawnItems.clear();

    const QPen pen(QColor(Qt::red), 50);
    auto* circle = scene->addEllipse(-100, -100, 200, 200, pen);
    const QPointF center(distance * std::sin(personAngle), distance * std::cos(personAngle));
    circle->setPos(center);
    drawnItems.push_back(circle);

    auto* line = scene->addLine(0, 0, center.x(), center.y(), pen);
    line->setZValue(-1);
    drawnItems.push_back(line);

    const double angleDeg = personAngle * (180.0 / M_PI);
    const QString labelText = QString("Target Person\nDist: %1mm\nAng: %2°")
                              .arg(distance, 0, 'f', 2)
                              .arg(angleDeg, 0, 'f', 1);

    static QGraphicsTextItem* personLabel = nullptr;
    if (personLabel == nullptr)
    {
        personLabel = scene->addText(labelText);
        // Invertir eje Y de la fuente porque la escena gráfica del visor tiene el eje Y apuntando hacia arriba
        personLabel->setTransform(QTransform().scale(1, -1));

        QFont font = personLabel->font();
        font.setPixelSize(100);
        personLabel->setFont(font);
        personLabel->setDefaultTextColor(Qt::blue);
    }
    else
    {
        personLabel->setPlainText(labelText);
    }

    personLabel->setPos(center.x() + 10, center.y() - 10);
    personLabel->setZValue(100);
}

void SpecificWorker::DrawLidar(const RoboCompLidar3D::TPoints& points, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> drawnPoints;
    for (const auto& item : drawnPoints)
    {
        scene->removeItem(item);
        delete item;
    }
    drawnPoints.clear();

    const QPen pen(QColor("LightGreen"), 10);
    for (const auto& pt : points)
    {
        auto* rect = scene->addRect(-25, -25, 50, 50, pen);
        rect->setPos(pt.x, pt.y);
        drawnPoints.push_back(rect);
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