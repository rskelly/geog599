/*
 * platform.cpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */


#include "sim/rangefinder.hpp"
#include "sim/platform.hpp"

SimPlatform::SimPlatform() :
	m_forwardVelocity(10) {
}

void SimPlatform::update(double time) {
	m_position[0] = time * m_forwardVelocity;
	// TODO: Add some noisy movement to the orientation and position.
}

const Eigen::Matrix3d& SimPlatform::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& SimPlatform::position() const {
	return m_position;
}

SimPlatform::~SimPlatform() {}
