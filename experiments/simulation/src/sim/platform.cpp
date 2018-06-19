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
		m_altitude(0),
		m_altitudeTime(0),
		m_pulseRate(0) {
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

double PlatformState::altitude() const {
	return m_altitude;
}

double PlatformState::altitudeTime() const {
	return m_altitudeTime;
}

void PlatformState::setPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

void PlatformState::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

void PlatformState::setLinearVelocity(const Eigen::Vector3d& linearVelocity) {
	m_linearVelocity = linearVelocity;
}

void PlatformState::setAngularVelocity(const Eigen::Vector3d& angularVelocity) {
	m_angularVelocity = angularVelocity;
}

void PlatformState::setBatteryLevel(double level) {
	m_batteryLevel = level;
}

void PlatformState::setMass(double mass) {
	m_mass = mass;
}

void PlatformState::setAltitude(double altitude) {
	m_altitude = altitude;
}

void PlatformState::setAltitudeTime(double time) {
	m_altitudeTime = time;
}

double PlatformState::pulseRate() const {
	return m_pulseRate;
}

void PlatformState::setPulseRate(double rate) {
	m_pulseRate = rate;
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

constexpr double PF_CLOCK_DELAY = 1.0 / 500.0;

Platform::Platform() :
	m_gimbal(nullptr),
	m_rangefinder(nullptr),
	m_nadirRangefinder(nullptr),
	m_surface(nullptr) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);

}

void Platform::start() {
	Clock::addObserver(this, PF_CLOCK_DELAY);
	m_gimbal->start();
	m_rangefinder->start();
	m_nadirRangefinder->start();
}

void Platform::stop() {
	Clock::removeObserver(this);
	m_gimbal->stop();
	m_rangefinder->stop();
	m_nadirRangefinder->stop();
}

void Platform::rangeUpdate(uav::Rangefinder* rangefinder, uav::Range* range) {
	if(rangefinder == m_rangefinder) {
		if(range->valid()) {
			Eigen::Vector3d point = (m_rangefinderState.laserDirection() * range->range()) + m_rangefinderState.laserPosition();
			m_surface->addPoint(point, range->time());
		}
	} else if(rangefinder == m_nadirRangefinder) {
		if(range->valid()) {
			m_platformState.setAltitude(range->range());
			m_platformState.setAltitudeTime(range->time());
		}
	}
	delete range;
}

void Platform::tick(double time) {

	Eigen::Vector3d Pp(m_platformState.position());
	Eigen::Vector3d Po(m_platformState.orientation());

	if(!std::isnan(m_controlInput.altitude())) {
		double adj = (m_controlInput.altitude() - m_platformState.altitude()) / 2.0;
		Pp[2] += adj;
	}
	m_controlInput.reset();


	const Eigen::Vector3d& lVel = m_platformState.linearVelocity();

	Pp[0] += m_posPoisson.next(lVel[0] * PF_CLOCK_DELAY); // m/s multiplied by the delay
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
	laserDir.normalize();

	m_rangefinderState.setLaserPosition(laserPos);
	m_rangefinderState.setLaserDirection(laserDir);

	// Set the laser position for the range bridge, get the point and
	// TODO: This will be wrong because all the ranges are using the same rotation/position.
	dynamic_cast<uav::sim::Rangefinder*>(m_rangefinder)->rangeBridge()->setLaser(laserPos, laserDir);

	// Get the surface elevation; use the last returned range.
	// TODO: This will be wrong because all the ranges are using the same rotation/position.
	dynamic_cast<uav::sim::Rangefinder*>(m_nadirRangefinder)->rangeBridge()->setLaser(Pp, Eigen::Vector3d(0, 0, -1));
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
	m_rangefinder->setObserver(this);
}

uav::Rangefinder* Platform::rangefinder() const {
	return m_rangefinder;
}

void Platform::setNadirRangefinder(uav::Rangefinder* rangefinder) {
	m_nadirRangefinder = rangefinder;
	m_nadirRangefinder->setObserver(this);
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
	m_controlInput << input;
}

Platform::~Platform() {
	if(m_gimbal)
		delete m_gimbal;
	if(m_rangefinder)
		delete m_rangefinder;
	if(m_nadirRangefinder)
		delete m_nadirRangefinder;
}
