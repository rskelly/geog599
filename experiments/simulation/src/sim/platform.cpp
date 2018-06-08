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

void _update(Platform* p, bool* running) {
	while(*running) {
		p->update(uavtime());
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

Platform::Platform() :
	m_forwardVelocity(10),
	m_lastTime(-1),
	m_gimbal(nullptr),
	m_rangefinder(nullptr),
	m_nadirRangefinder(nullptr),
	m_surface(nullptr),
	m_elevation(0), m_elevationTime(0),
	m_running(false),
	m_startTime(0) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);
}

void Platform::start() {
	if(!m_running) {
		m_startTime = uavtime();
		m_running = true;
		m_thread = std::thread(_update, this, &m_running);
	}
}

void Platform::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
}

void Platform::update(double time) {
	if(m_lastTime < 0) {
		m_lastTime = time;
		return;
	}

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

	// 2) Rotate the laser by the gimbal's static orientation and translate it into the platform's frame.

	Eigen::Vector3d Gps(m_gimbal->staticPosition());
	Eigen::Matrix3d Gos = eulerToMatrix(m_gimbal->staticOrientation());

	Eigen::Vector3d B = Gos * A + Gps;

	// 3) Rotate the platform then translate into the inertial frame.

	Eigen::Vector3d Pp(position());
	Eigen::Matrix3d Po = eulerToMatrix(orientation());

	m_laserPosition = Po * B + Pp;

	// 4) Compute the laser orientation by adding the orientations. It passes through the laserPosition.

	m_laserDirection = eulerToVector(matrixToEuler(God * Gos * Po));

	dynamic_cast<uav::sim::Rangefinder*>(m_rangefinder)->rangeBridge()->setLaser(m_laserPosition, m_laserDirection);

	Range* range = m_rangefinder->range();
	if(range && !std::isnan(range->range())) {
		Eigen::Vector3d point = (m_laserDirection.normalized() * range->range()) + m_laserPosition;
		m_surface->addPoint(point, range->time());
	}

}

void Platform::setInitialOrientation(const Eigen::Vector3d orientation) {
	m_orientation = orientation;
}

void Platform::setInitialPosition(const Eigen::Vector3d position) {
	m_position = position;
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

void Platform::setNadirRangefinder(uav::Rangefinder* rangefinder) {
	m_nadirRangefinder = rangefinder;
}

uav::Rangefinder* Platform::nadirRangefinder() const {
	return m_nadirRangefinder;
}

double Platform::elevation() {
	dynamic_cast<uav::sim::Rangefinder*>(m_nadirRangefinder)->rangeBridge()->setLaser(m_position, Eigen::Vector3d(0, 0, -1));
	Range* r = m_nadirRangefinder->range();
	if(r) {
		m_elevation = r->range();
		m_elevationTime = r->time();
	}
	return m_elevation;
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
	stop();
	if(m_gimbal)
		delete m_gimbal;
	if(m_rangefinder)
		delete m_rangefinder;
	if(m_nadirRangefinder)
		delete m_nadirRangefinder;
}
