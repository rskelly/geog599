/*
 * platform.cpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#include <iostream>

#include <Eigen/Geometry>

#include "uav.hpp"
#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"
#include "sim/gimbal.hpp"
#include "sim/terrain.hpp"
#include "surface.hpp"
#include "util.hpp"

using namespace uav::sim;
using namespace uav::surface;
using namespace uav::util;

Platform::Platform() :
	m_forwardVelocity(10),
	m_lastTime(0),
	m_gimbal(nullptr),
	m_rangefinder(nullptr),
	m_surface(nullptr) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);

	m_position << 489103, 6502712, 320; // 30m high
	m_orientation << 0, 0, 0; // straight level
}


void Platform::update(double time) {
	double t = time - m_lastTime;
	m_lastTime = time;

	m_position[0] += t * m_posPoisson.next(m_forwardVelocity);
	m_position[1] += m_posPoisson.nextCentered();
	m_position[2] += m_posPoisson.nextCentered();

	m_orientation[0] += 0; //m_rotPoisson.nextCentered();
	m_orientation[1] += 0; //m_rotPoisson.nextCentered();
	m_orientation[2] += 0; //m_rotPoisson.nextCentered();

	// 1) Translate the laser into the gimbal's frame and rotate it using the gimbal's dynamic orientation. (The laser has no position info of its own).

	Eigen::Vector3d Gpd(m_gimbal->position());
	Eigen::Matrix3d God = eulerToMatrix(m_gimbal->orientation());
	Eigen::Vector3d A = God * Gpd;

	//std::cerr << "Gpd " << Gpd << "\n";
	//std::cerr << "God " << God << "\n";
	//std::cerr << "A " << A << "\n";

	// 2) Rotate the laser by the gimbal's static orientation and translate it into the platform's frame.

	Eigen::Vector3d Gps(m_gimbal->staticPosition());
	Eigen::Matrix3d Gos = eulerToMatrix(m_gimbal->staticOrientation());

	Eigen::Vector3d B = Gos * A + Gps;

	//std::cerr << "Gps " << Gps << "\n";
	//std::cerr << "Gos " << Gos << "\n";
	//std::cerr << "B " << B << "\n";

	// 3) Rotate the platform then translate into the inertial frame.

	Eigen::Vector3d Pp(position());
	Eigen::Matrix3d Po = eulerToMatrix(orientation());

	//std::cerr << "Pp " << Pp << "\n";
	//std::cerr << "Po " << Po << "\n";

	m_laserPosition = Po * B + Pp;

	// 4) Compute the laser orientation by adding the orientations. It passes through the laserPosition.

	//std::cerr << "laser orientation " << God * Gos * Po << "\n";

	m_laserDirection = eulerToVector(matrixToEuler(God * Gos * Po));

	//std::cerr << "Laser Position " << m_laserPosition << "\n";
	//std::cerr << "Laser Direction " << m_laserDirection << "\n";

	RangeBridge::setLaser(m_laserPosition, m_laserDirection);

	Range* range = m_rangefinder->range();
	if(!std::isnan(range->range())) {
		Eigen::Vector3d point = (m_laserDirection.normalized() * range->range()) + m_laserPosition;
		m_surface->addPoint(point, range->time());
	}

	/*
	if(range) {
		std::cerr << "Range " << range->range() << ", " << range->time() << "\n";
		std::cerr << "Position " << m_position[0] << ", " << m_position[1] << "\n";
		delete range;
	}
	*/
}

uav::Range* Platform::range() const {
	return m_rangefinder->range();
}

const Eigen::Vector3d Platform::laserPosition() const {
	return m_laserPosition;
}

const Eigen::Vector3d Platform::laserDirection() const {
	return m_laserDirection;
}

void Platform::setGimbal(uav::Gimbal* gimbal) {
	m_gimbal = gimbal;
}

uav::Gimbal* Platform::gimbal() const {
	return m_gimbal;
}

void Platform::setRangefinder(uav::Rangefinder* rangefinder) {
	m_rangefinder = rangefinder;
}

uav::Rangefinder* Platform::rangefinder() const {
	return m_rangefinder;
}

void Platform::setSurface(Surface* surface) {
	m_surface = surface;
}

Surface* Platform::surface() const {
	return m_surface;
}

const Eigen::Vector3d& Platform::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& Platform::position() const {
	return m_position;
}

Platform::~Platform() {
	delete m_gimbal;
	delete m_rangefinder;
}
