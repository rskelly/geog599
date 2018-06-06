/*
 * gimbal.cpp
 *
 *  Created on: May 25, 2018
 *      Author: rob
 */

#include <chrono>
#include <iostream>

#include "uav.hpp"

#include "sim/gimbal.hpp"

using namespace uav::sim;

/**
 * Updates the gimbal's dynamic orientation in a separate thread.
 * TODO: This should be configurable to test different sweep rates, etc.
 *
 * @param orientation The gimbal's dynamic orientation vector (Euler angles).
 */
void updateGimbal(Eigen::Vector3d* orientation, bool* running) {
	double t = 0;
	while(*running) {
		double a = std::sin(t += 0.05) * (PI / 4);
		//std::cerr << "gimbal angle: " << a << "\n";
		(*orientation)[2] = a; // around z-axis (side to side)
		std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
	}
}

SinGimbal::SinGimbal(double sweepAngle, double sweepFrequency) :
	m_position(Eigen::Vector3d(0, 0, 0)), // 2cm down
	m_running(true),
	m_sweepAngle(sweepAngle), m_sweepFrequency(sweepFrequency) {

	m_thread = std::thread(updateGimbal, &m_orientation, &m_running);
}

void SinGimbal::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

const Eigen::Vector3d& SinGimbal::orientation() const {
	return m_orientation;
}

void SinGimbal::setPosition(const Eigen::Vector3d& position) {
	m_position = position;
}

const Eigen::Vector3d& SinGimbal::position() const {
	return m_position;
}

void SinGimbal::setStaticOrientation(const Eigen::Vector3d& mtx) {
	m_staticOrientation = mtx;
}

const Eigen::Vector3d& SinGimbal::staticOrientation() const {
	return m_staticOrientation;
}

void SinGimbal::setStaticPosition(const Eigen::Vector3d& mtx) {
	m_staticPosition = mtx;
}

const Eigen::Vector3d& SinGimbal::staticPosition() const {
	return m_staticPosition;
}

SinGimbal::~SinGimbal() {
	m_running = false;
	m_thread.join();
}
