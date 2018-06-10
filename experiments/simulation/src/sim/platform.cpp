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


PlatformState::PlatformState() :
		m_batteryLevel(0),
		m_mass(0),
		m_surfaceElevation(0),
		m_surfaceElevationTime(0) {
}

const Eigen::Vector3d& PlatformState::position() const {
	return m_position;
}

const Eigen::Vector3d& PlatformState::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& PlatformState::linearVelocity() const {
	return m_linearVelocity;
}

const Eigen::Vector3d& PlatformState::angularVelocity() const {
	return m_angularVelocity;
}

double PlatformState::batteryLevel() const {
	return m_batteryLevel;
}

double PlatformState::mass() const {
	return m_mass;
}

double PlatformState::surfaceElevation() const {
	return m_surfaceElevation;
}

double PlatformState::surfaceElevationTime() const {
	return m_surfaceElevationTime;
}

void PlatformState::setPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

void PlatformState::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

void PlatformState::setBatteryLevel(double level) {
	m_batteryLevel = level;
}

void PlatformState::setMass(double mass) {
	m_mass = mass;
}

void PlatformState::setSurfaceElevation(double elevation) {
	m_surfaceElevation = elevation;
}

void PlatformState::setSurfaceElevationTime(double time) {
	m_surfaceElevationTime = time;
}

void RangefinderState::setLaserPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

void RangefinderState::setLaserDirection(const Eigen::Vector3d& direction) {
	m_direction = direction;
}

const Eigen::Vector3d& RangefinderState::laserPosition() const {
	return m_position;
}

const Eigen::Vector3d& RangefinderState::laserDirection() const {
	return m_direction;
}

Platform::Platform() :
	m_gimbal(nullptr),
	m_rangefinder(nullptr),
	m_nadirRangefinder(nullptr),
	m_surface(nullptr) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);
}

void Platform::update(double time) {

	Eigen::Vector3d Pp(m_platformState.position());
	Eigen::Vector3d Po(m_platformState.orientation());

	const Eigen::Vector3d& lVel = m_platformState.linearVelocity();

	Pp[0] += time * m_posPoisson.next(lVel[0]);
	Pp[1] += m_posPoisson.nextCentered();
	Pp[2] += m_posPoisson.nextCentered();

	Po[0] += 0; //m_rotPoisson.nextCentered();
	Po[1] += 0; //m_rotPoisson.nextCentered();
	Po[2] += 0; //m_rotPoisson.nextCentered();

	m_platformState.setPosition(Pp);
	m_platformState.setOrientation(Po);

	// 1) Translate the laser into the gimbal's frame and rotate it using the gimbal's dynamic orientation. (The laser has no position info of its own).

	const Eigen::Vector3d& Gpd = m_gimbal->position();
	const Eigen::Vector3d& God = m_gimbal->orientation();

	Eigen::Vector3d A = eulerToMatrix(God) * Gpd;

	// 2) Rotate the laser by the gimbal's static orientation and translate it into the platform's frame.

	const Eigen::Vector3d& Gps = m_gimbal->staticPosition();
	const Eigen::Vector3d& Gos = m_gimbal->staticOrientation();

	Eigen::Vector3d B = eulerToMatrix(Gos) * A + Gps;

	// 3) Rotate the platform then translate into the inertial frame.

	Eigen::Vector3d laserPos = eulerToMatrix(Po) * B + Pp;

	// 4) Compute the laser orientation by adding the orientations. It passes through the laserPosition.

	Eigen::Vector3d laserDir = eulerToVector(God + Gos + Po);

	m_rangefinderState.setLaserPosition(laserPos);
	m_rangefinderState.setLaserDirection(laserDir);

	// Set the laser position for the range bridge, get the point and
	// TODO: This will be wrong because all the ranges are using the same rotation/position.
	dynamic_cast<uav::sim::Rangefinder*>(m_rangefinder)->rangeBridge()->setLaser(laserPos, laserDir);
	std::vector<uav::Range*> ranges;
	if(m_rangefinder->getRanges(ranges)) {
		for(Range* range : ranges) {
			Eigen::Vector3d point = (laserDir.normalized() * range->range()) + laserPos;
			m_surface->addPoint(point, range->time());
			delete range;
		}
		ranges.clear();
	}

	// Get the surface elevation; use the last returned range.
	// TODO: This will be wrong because all the ranges are using the same rotation/position.
	dynamic_cast<uav::sim::Rangefinder*>(m_nadirRangefinder)->rangeBridge()->setLaser(Pp, Eigen::Vector3d(0, 0, -1));
	if(m_nadirRangefinder->getRanges(ranges)) {
		const Range* last = ranges[ranges.size() - 1];
		m_platformState.setSurfaceElevation(last->range());
		m_platformState.setSurfaceElevationTime(last->time());
		for(Range* range : ranges)
			delete range;
		ranges.clear();
	}

}

void Platform::setInitialPlatformState(const uav::sim::PlatformState& state) {
	m_platformState = state;
}

const uav::RangefinderState& Platform::rangefinderState() const {
	return m_rangefinderState;
}

const uav::RangefinderState& Platform::nadirRangefinderState() const {
	return m_nadirRangefinderState;
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

void Platform::setSurface(Surface* surface) {
	m_surface = surface;
}

Surface* Platform::surface() const {
	return m_surface;
}

const uav::PlatformState& Platform::platformState() const {
	return m_platformState;
}

void Platform::setControlInput(const uav::PlatformControlInput& input) {

}

Platform::~Platform() {
	if(m_gimbal)
		delete m_gimbal;
	if(m_rangefinder)
		delete m_rangefinder;
	if(m_nadirRangefinder)
		delete m_nadirRangefinder;
}
