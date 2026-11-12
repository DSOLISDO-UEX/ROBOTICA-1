#include "specificworker.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <ranges>

namespace ControlParams
{
    float targetDistMm = 1000.0f;        // Distancia objetivo nominal (mm) [1 metro]
    float maxAdvSpeed = 1000.0f;         // Velocidad lineal máxima (mm/s)
    float maxRotSpeed = 1.0f;            // Velocidad angular máxima (rad/s)
    float coneHalfApertureRad = 0.45f;   // Semiapertura del cono de filtrado LiDAR (rad)
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

    // Configurar parámetros del controlador DWA
    Controller::Params ctrlParams;
    ctrlParams.targetDistMm = ControlParams::targetDistMm;
    ctrlParams.maxAdvSpeed = ControlParams::maxAdvSpeed;
    ctrlParams.maxRotSpeed = ControlParams::maxRotSpeed;
    ctrlParams.minAdvSpeed = 0.0f;
    ctrlParams.robotRadiusMm = 285.0f;
    ctrlParams.safetyMarginMm = 45.0f;
    ctrlParams.simTimeSec = 0.8f;
    ctrlParams.simSteps = 8;
    ctrlParams.minSelfChassisDistMm = 340.0f;
    controller_.setParams(ctrlParams);
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
        rawAngle = std::remainder(rawAngle, 2.0 * M_PI);

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
        else
        {
            // Fallback 1: Si el tracker ya tiene una estimación previa de profundidad, mantenerla
            if (tracker_.IsInitialized())
            {
                const auto [_, prevDepth] = tracker_.GetPolarEstimate();
                if (prevDepth > 400.0 && prevDepth < 15000.0)
                {
                    rawDepth = static_cast<float>(prevDepth);
                    hasValidMeasurement = true;
                }
            }

            // Fallback 2: Estimar profundidad a partir de la altura aparente de la caja 2D en la imagen equirectangular
            if (!hasValidMeasurement)
            {
                const double hNorm = static_cast<double>(roi.height()) / img.height;
                if (hNorm > 0.04)
                {
                    // Altura típica del torso/humano ~1700mm
                    rawDepth = std::clamp(static_cast<float>(1700.0 / std::tan(hNorm * M_PI)), 500.0f, 10000.0f);
                    hasValidMeasurement = true;
                }
            }
        }
    }

    if (hasValidMeasurement)
    {
        tracker_.ProcessMeasurement(rawAngle, rawDepth, timeNow);
    }
    else
    {
        if (!tracker_.StepWithoutMeasurement(dtEgo, timeNow))
        {
            StopRobot();
            DrawImage(img, roi, projectedPoints, label);
            return;
        }
    }

    const auto [estAngle, estDist] = tracker_.GetPolarEstimate();
    const double estDepth = estDist;
    const double radialVel = tracker_.GetRadialVelocity();
    const auto cartVel = tracker_.GetCartesianVelocity();

    // Actualizar controlador jerárquico DWA + Proporcional
    const auto ctrlOutput = controller_.update(estAngle, estDepth, radialVel,
                                               cartVel.x(), cartVel.y(),
                                               lidarData.points);

    static auto lastLogTime = std::chrono::steady_clock::now();
    const auto nowLog = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(nowLog - lastLogTime).count() >= 500)
    {
        lastLogTime = nowLog;
        std::cout << "[task4-DWA] det=" << visualObjects.objects.size()
                  << " meas=" << hasValidMeasurement
                  << " estA=" << estAngle << " estD=" << estDepth
                  << " adv=" << ctrlOutput.advSpeed << " rot=" << ctrlOutput.rotSpeed
                  << " obsDist=" << ctrlOutput.minObsDist
                  << " avoiding=" << ctrlOutput.isAvoiding << std::endl;
    }

    try
    {
        omnirobot_proxy->setSpeedBase(0.0f, ctrlOutput.advSpeed, ctrlOutput.rotSpeed);
    }
    catch (const Ice::Exception& e)
    {
        std::cerr << "OmniRobot error: " << e << std::endl;
    }
    lastAdvCmd_ = ctrlOutput.advSpeed;
    lastRotCmd_ = ctrlOutput.rotSpeed;

    DrawImage(img, roi, projectedPoints, label);
    if (viewer_ != nullptr)
    {
        auto* scene = &viewer_->scene;
        DrawLidar(lidarData.points, scene);
        DrawPersonInLidar(static_cast<float>(estAngle), static_cast<float>(estDepth), scene);
        DrawDwaArcs(ctrlOutput, scene);
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
    lastAdvCmd_ = 0.0f;
    lastRotCmd_ = 0.0f;
}

RoboCompLidar3D::TPoints SpecificWorker::FilterCone(const RoboCompLidar3D::TPoints& points, float minAngle,
                                                    float maxAngle, float minDistance)
{
    const float center = (minAngle + maxAngle) * 0.5f;
    const float halfSpan = std::abs(maxAngle - minAngle) * 0.5f;

    auto filtered = points | std::views::filter([=](const auto& point)
    {
        if (point.distance2d <= minDistance) return false;
        const float diff = std::remainder(point.phi - center, static_cast<float>(2.0 * M_PI));
        return std::abs(diff) <= halfSpan;
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
    const auto filteredCone = FilterCone(
        points,
        static_cast<float>(personAngle - ControlParams::coneHalfApertureRad),
        static_cast<float>(personAngle + ControlParams::coneHalfApertureRad),
        250.0f);

    std::vector<QPointF> projectedPoints;
    projectedPoints.reserve(filteredCone.size());
    std::vector<float> distances;

    const QRect expandedRoi = roi.adjusted(-12, -12, 12, 12);

    for (const auto& lp : filteredCone)
    {
        const Eigen::Vector3d pointLidar(lp.x, lp.y, lp.z);
        const Eigen::Vector3d pointCam = camTransform * pointLidar;

        const double radius = pointCam.norm();
        if (radius < 100.0) continue;

        double phiCam = std::atan2(pointCam.x(), pointCam.y());
        phiCam = azimuthSign * phiCam + azimuthOffset;
        phiCam = std::remainder(phiCam, 2.0 * M_PI);

        const double u = (phiCam / (2.0 * M_PI) + 0.5) * image.width;
        if (u < 0.0 || u >= image.width) continue;

        const double v = (std::asin(-pointCam.z() / radius) / M_PI + 0.5) * image.height;

        if (expandedRoi.contains(QPoint(static_cast<int>(u), static_cast<int>(v))))
        {
            projectedPoints.emplace_back(u, v);
            distances.push_back(static_cast<float>(radius));
        }
    }

    if (distances.empty())
    {
        for (const auto& lp : filteredCone)
        {
            const float diff = std::abs(std::remainder(lp.phi - static_cast<float>(personAngle), static_cast<float>(2.0 * M_PI)));
            if (diff < 0.20f && lp.distance2d > 400.0f && lp.distance2d < 15000.0f)
            {
                distances.push_back(lp.distance2d);
            }
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

void SpecificWorker::DrawDwaArcs(const Controller::Output& output, QGraphicsScene* scene)
{
    static std::vector<QGraphicsItem*> arcItems;
    for (auto* item : arcItems)
    {
        scene->removeItem(item);
        delete item;
    }
    arcItems.clear();

    // Dibujar arcos candidatos evaluados en azul traslúcido
    const QPen candPen(QColor(135, 206, 250, 50), 12);
    for (const auto& arc : output.candidateArcs)
    {
        for (size_t i = 1; i < arc.size(); ++i)
        {
            auto* line = scene->addLine(arc[i - 1].x(), arc[i - 1].y(),
                                        arc[i].x(), arc[i].y(), candPen);
            arcItems.push_back(line);
        }
    }

    // Dibujar el arco óptimo seleccionado: verde si sigue persona, naranja si esquiva obstáculo
    const QColor bestColor = output.isAvoiding ? QColor(255, 140, 0) : QColor(0, 220, 0);
    QPen bestPen(bestColor, 35);
    bestPen.setCapStyle(Qt::RoundCap);
    for (size_t i = 1; i < output.bestArc.size(); ++i)
    {
        auto* line = scene->addLine(output.bestArc[i - 1].x(), output.bestArc[i - 1].y(),
                                    output.bestArc[i].x(), output.bestArc[i].y(), bestPen);
        line->setZValue(10);
        arcItems.push_back(line);
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
