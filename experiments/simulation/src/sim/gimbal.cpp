/*
 * gimbal.cpp
 *
 *  Created on: May 25, 2018
 *      Author: rob
 */

#include <chrono>
#include <iostream>

#include "sim/gimbal.hpp"

#define PI 3.141592653589793238

using namespace uav::sim;

void updateGimbal(Eigen::Vector3d* orientation) {
	double t = 0;
	while(true) {
		double a = -(PI / 4) + std::sin(t += 0.1) + (PI / 2);
		//std::cerr << "gimbal angle: " << a << "\n";
		(*orientation)[2] = a; // around z-axis (side to side)
		std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
	}
}

Gimbal::Gimbal() :
	m_position(Eigen::Vector3d(0, 0, -0.02)), // 2cm down
	m_thread(nullptr) {

	m_thread = new std::thread(updateGimbal, &m_orientation);
}

const Eigen::Vector3d& Gimbal::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& Gimbal::position() const {
	return m_position;
}

void Gimbal::setStaticOrientation(const Eigen::Vector3d& mtx) {
	m_staticOrientation = mtx;
}

const Eigen::Vector3d& Gimbal::staticOrientation() const {
	return m_staticOrientation;
}

void Gimbal::setStaticPosition(const Eigen::Vector3d& mtx) {
	m_staticPosition = mtx;
}

const Eigen::Vector3d& Gimbal::staticPosition() const {
	return m_staticPosition;
}

Gimbal::~Gimbal() {
	delete m_thread;
}



