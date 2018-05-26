/*
 * platform.cpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#include <iostream>

#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"

using namespace uav::sim;

Platform::Platform() :
	m_forwardVelocity(10),
	m_lastTime(0),
	m_gimbal(nullptr),
	m_rangefinder(nullptr) {

	m_posPoisson.setMean(1000);
	m_rotPoisson.setMean(1000);

	m_position << 0, 0, 0;
	m_orientation << 0, 0, 0;
}

void Platform::update(double time) {
	double t = time - m_lastTime;
	m_lastTime = time;

	m_position[0] += t * m_posPoisson.next(m_forwardVelocity);
	m_position[1] += m_posPoisson.nextCentered();
	m_position[2] += m_posPoisson.nextCentered();

	m_orientation[0] += m_rotPoisson.nextCentered();
	m_orientation[1] += m_rotPoisson.nextCentered();
	m_orientation[2] += m_rotPoisson.nextCentered();

	// TODO: Add some noisy movement to the orientation and position.
	std::cerr << "position: " << m_position[0] << ", " << m_position[1] << ", " << m_position[2] << "\n";
	std::cerr << "orientation: " << m_orientation[0] << ", " << m_orientation[1] << ", " << m_orientation[2] << "\n";

	// Get position of rangefinder nodal point and axis.
	//Eigen::Vector3d gpos = position() + gimbal()->position();
	//Eigen::Vector3d qrot = orientation() + gimbal()->orientation();

}

const uav::Gimbal* Platform::gimbal() const {
	return m_gimbal;
}

const uav::Rangefinder* Platform::rangefinder() const {
	return m_rangefinder;
}

const Eigen::Vector3d& Platform::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& Platform::position() const {
	return m_position;
}

Platform::~Platform() {}
