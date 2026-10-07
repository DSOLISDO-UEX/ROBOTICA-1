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
    catch (Ice::Exception& e){ std::cout << e.what() << std::endl; };

    // logic to avoid hitting the wall.
    // Takes lidar data and returns new values for adv and rot
     auto [adv, rot] = navigation(data.points);
    //
     try { omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch (std::exception& e){ std::cout << e.what() << std::endl; };
}

void SpecificWorker::new_target_slot(QPointF p)
{
    qInfo() << p;
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
                    rot = 1.f;
                }
                else
                {
                    rot = -1.f;
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

    auto myPoints = cono(points, -0.01, 0.01, 0);

    for (const auto& p : myPoints)
    {
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);
        dp->setPos(p.x, p.y);
        draw_points.push_back(dp); // add to the list of points to be deleted next time
    }
}


void SpecificWorker::emergency()
{
    fps.print("Emergency worker", 3000);
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component
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
