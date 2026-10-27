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
		data = this->lidar3d_proxy->getLidarData("lidar", 0, 2 * M_PI, 2);
		if (data.points.empty())
		{
			qWarning("No Lidar data");
			return;
		}

		draw_lidar(data.points, &viewer->scene);

		RoboCompVisualElements::TObjects objects = visualelements_proxy->getVisualObjects(RoboCompVisualElements::TObjects{});

		float target_rot = 0.f;
		float adv = 0.f;
		bool person_detected = false;

		const double imgWidth = 1920.0;

		if (!objects.objects.empty())
		{
			const auto &obj = objects.objects.front();

			// Centro X en la imagen 360°
			double centroPersonaX = (obj.left + obj.right) / 2.0;
			double deltaX = centroPersonaX - (imgWidth / 2.0);

			// 1. Proyección lineal equirrectangular (360° = 2 * PI rad)
			// deltaX va de -480 a +480, lo que mapea linealmente de -PI a +PI rad
			double yawRad = deltaX * (2.0 * M_PI / imgWidth);
			double yawDeg = yawRad * (180.0 / M_PI);

			// 2. Parámetros del Controlador Proporcional Suavizado
			const float Kp = 1.0f;			  // Ganancia proporcional (1/s)
			const float MAX_ROT_SPEED = 1.2f; // Límite de velocidad en rad/s (~68 deg/s)
			const double DEADBAND_DEG = 2.0;  // Margen de histeresis para evitar micro-ajustes

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
	catch (Ice::Exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
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
            return (point.distance2d < distance) && (point.distance2d > this->ROBOT_LENGTH / 2.1);
        return true; });
}

std::tuple<float, float> SpecificWorker::navigation(const RoboCompLidar3D::TPoints &points)
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

	auto myPoints = cono(points, -0.01, 0.01, 0);

	for (const auto &p : myPoints)
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