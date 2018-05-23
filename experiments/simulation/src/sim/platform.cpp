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
	m_lastTime(0) {
	m_posPoisson.setMean(1000);
}

void Platform::update(double time) {
	double t = time - m_lastTime;
	m_lastTime = time;

	m_position[0] += t * m_posPoisson.next(m_forwardVelocity);
	m_position[1] += m_posPoisson.nextCentred();
	m_position[2] += m_posPoisson.nextCentred();
	// TODO: Add some noisy movement to the rotation and position.
	std::cerr << "position: " << m_position[0] << ", " << m_position[1] << ", " << m_position[2] << "\n";
}

const Eigen::Matrix3d& Platform::rotation() const {
	return m_rotation;
}

const Eigen::Vector3d& Platform::position() const {
	return m_position;
}

Platform::~Platform() {}
