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

// Faster speeds & adjusted braking distances
#define MAX_ADV_VELOCITY 1000.f                 // mm/s (Faster linear speed)
#define MAX_ROT_VELOCITY 1.0f                   // rad/s (Faster rotation when dodging obstacles)
#define CONE_ANGLE (M_PI / 3.2f)                // ~56 degrees
#define SAFE_DISTANCE 600.f                     // mm (Increased slightly to compensate for higher speed)
#define CLEAR_DISTANCE 1000.f                   // mm (Hysteresis clearance distance)

// Fordward to spiral and spiral to fordward
#define MIN_RANDOM_FORDWARD_TO_SPIRAL_DURATION 2 // seconds
#define MAX_RANDOM_FORDWARD_TO_SPIRAL_DURATION 8 // seconds
#define MIN_RANDOM_SPIRAL_TO_FORDWARD_DURATION 5 // seconds
#define MAX_RANDOM_SPIRAL_TO_FORDWARD_DURATION 10 // seconds

// Spiral parameters (Tuned for a wider radius)
#define SPIRAL_ADV_SPEED_DECREMENT_FACTOR 1.2f  // Keeps spiral linear speed high (1600 / 1.2 = 1333 mm/s)
#define SPIRAL_INITIAL_ROT 0.8f                 // rad/s (Lower starting rot = much wider initial radius)
#define SPIRAL_ROT_SPEED_DECREMENT 0.002f       // Opens the spiral smoothly
#define SPIRAL_MIN_ROT_THRESHOLD 0.05f          // Safety floor so spiralRot never goes negative

SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(
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

        // Example statemachine:
        /***
        //Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
        states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
                                                            std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
                                                            std::bind(&SpecificWorker::customEnter, this), // On-enter function
                                                            std::bind(&SpecificWorker::customExit, this)); // On-exit function

        //Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
        states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
        states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

        //Add your custom state
        statemachine.addState(states["CustomState"].get());
        ***/

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
    std::srand(std::time(nullptr));
    fps.print("Compute worker", 3000);

    RoboCompLidar3D::TData data;
    try
    {
        data = lidar3d_proxy->getLidarData("lidar", 0, 2 * M_PI, 2);
        if (data.points.empty())
        {
            qFatal("No Lidar data");
            return;
        }

        draw_lidar(data.points, &viewer->scene);
    }
    catch (Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    };

    // logic to avoid hitting the wall.
    // Takes lidar data and returns new values for adv and rot
    auto [adv, rot] = navigation(data.points);
    try
    {
        omnirobot_proxy->setSpeedBase(0.f, adv, rot);
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    };
}

void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << p;
}

auto SpecificWorker::cono(const RoboCompLidar3D::TPoints &points, float minAngle,
                          float maxAngle, float distance)
{
    return points | std::views::filter([=, this](const auto &point)
                                       {
        bool inAngle = (point.phi > minAngle) and (point.phi < maxAngle);
        if (not inAngle) return false;

        if (distance > 0)
            return (point.distance2d < distance) && (point.distance2d > this->ROBOT_LENGTH / 1.2);
        return true; });
}
std::tuple<float, float> SpecificWorker::navigation(const RoboCompLidar3D::TPoints &points)
{
    static auto stateStartTime = std::chrono::steady_clock::now();
    static int targetAdvanceDuration = (std::rand() % (MAX_RANDOM_FORDWARD_TO_SPIRAL_DURATION - MIN_RANDOM_FORDWARD_TO_SPIRAL_DURATION + 1)) + MIN_RANDOM_FORDWARD_TO_SPIRAL_DURATION; // 3 to 6 seconds
    static int targetSpiralDuration = (std::rand() % (MAX_RANDOM_SPIRAL_TO_FORDWARD_DURATION - MIN_RANDOM_SPIRAL_TO_FORDWARD_DURATION + 1)) + MIN_RANDOM_SPIRAL_TO_FORDWARD_DURATION;  // 3 to 6 seconds
    static float spiralRot = SPIRAL_INITIAL_ROT;
    static float spiralSign = 1.f; // 1.f o -1.f para el sentido de la espiral
    static float lockedRotVel = MAX_ROT_VELOCITY;

    auto frontPoints = cono(points, -CONE_ANGLE, CONE_ANGLE, SAFE_DISTANCE);
    const bool obstacleAhead = !frontPoints.empty();

    auto clearPoints = cono(points, -CONE_ANGLE, CONE_ANGLE, CLEAR_DISTANCE);
    const bool pathIsClear = clearPoints.empty();

    auto startRotation = [&]() -> std::tuple<float, float>
    {
        auto closest = std::ranges::min_element(frontPoints, {}, &RoboCompLidar3D::TPoint::distance2d);
        float rotDir = (closest->phi < 0.f) ? -1.f : 1.f;
        
        lockedRotVel = rotDir * MAX_ROT_VELOCITY;
        state = RobotState::Rotate;
        return {0.f, lockedRotVel};
    };

    auto startAdvance = [&]() -> std::tuple<float, float>
    {
        stateStartTime = std::chrono::steady_clock::now();
        targetAdvanceDuration = (std::rand() % (MAX_RANDOM_FORDWARD_TO_SPIRAL_DURATION - MIN_RANDOM_FORDWARD_TO_SPIRAL_DURATION + 1)) + MIN_RANDOM_FORDWARD_TO_SPIRAL_DURATION; // Random 3-6s for Advance
        state = RobotState::Advance;
        return {MAX_ADV_VELOCITY, 0.f};
    };

    switch (state)
    {
    case RobotState::Advance:
    {
        if (obstacleAhead)
            return startRotation();

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - stateStartTime)
                           .count();

        if (elapsed >= targetAdvanceDuration)
        {
            stateStartTime = std::chrono::steady_clock::now();
            targetSpiralDuration = (std::rand() % (MAX_RANDOM_SPIRAL_TO_FORDWARD_DURATION - MIN_RANDOM_SPIRAL_TO_FORDWARD_DURATION + 1)) + MIN_RANDOM_SPIRAL_TO_FORDWARD_DURATION; // Random 3-6s for Spiral
            spiralRot = SPIRAL_INITIAL_ROT;               // Starts with a wider radius
            spiralSign = (std::rand() % 2 == 0) ? 1.f : -1.f; // 50% probabilidad izq / der
            state = RobotState::Spiral;
        }

        return {MAX_ADV_VELOCITY, 0.f};
    }

    case RobotState::Rotate:
    {
        if (!obstacleAhead)
            return startAdvance();

        return {0.f, lockedRotVel};
    }

    case RobotState::Spiral:
    {
        if (!pathIsClear)
            return startRotation();

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                           std::chrono::steady_clock::now() - stateStartTime)
                           .count();

        float adv = MAX_ADV_VELOCITY / SPIRAL_ADV_SPEED_DECREMENT_FACTOR;
        spiralRot -= SPIRAL_ROT_SPEED_DECREMENT;

        // Transition back to Advance after the random duration OR if the spiral becomes a straight line
        if (elapsed >= targetSpiralDuration || spiralRot <= SPIRAL_MIN_ROT_THRESHOLD)
            return startAdvance();

        return {adv, spiralSign * spiralRot};
    }
    }

    return {0.f, 0.f};
}

void SpecificWorker::draw_lidar(const auto &points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> draw_points;
    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("LightGreen");
    const QPen pen(color, 10);
    // const QBrush brush(color, Qt::SolidPattern);

    auto myPoints = cono(points, -CONE_ANGLE, CONE_ANGLE, SAFE_DISTANCE);

    for (const auto &p : myPoints)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp); // add to the list of points to be deleted next time
    }
}

void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    // emergencyCODE
    //
    // if (SUCCESSFUL) //The componet is safe for continue
    //   emmit goToRestore()
}

// Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    // restoreCODE
    // Restore emergency component
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCamera360RGB you can call this methods:
// RoboCompCamera360RGB::TImage this->camera360rgb_proxy->getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TColorCloudData this->lidar3d_proxy->getColorCloudData()
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can call this methods:
// RoboCompVisualElements::TObjects this->visualelements_proxy->getVisualObjects(TObjects objects)
// RoboCompVisualElements::void this->visualelements_proxy->setVisualObjects(TObjects objects)

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects
