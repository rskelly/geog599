/*
 * gimbal.cpp
 *
 *  Created on: May 25, 2018
 *      Author: rob
 */

#include <chrono>
#include <iostream>

#include "uav.hpp"
#include "util.hpp"
#include "sim/gimbal.hpp"

using namespace uav::sim;
using namespace uav::util;

/**
 * Updates the gimbal's dynamic orientation in a separate thread.
 * TODO: This should be configurable to test different sweep rates, etc.
 *
 * @param orientation The gimbal's dynamic orientation vector (Euler angles).
 */
void updateGimbal(Eigen::Vector3d* orientation, bool* running) {
	double t = uavtime();
	while(*running) {
		double t0 = uavtime();
		// Every second, performs a full rotation of 2PI, with a +-20 degree sweep.
		double a = std::sin((t0 - t) * PI * 2) * (PI / 9);
		(*orientation)[2] = a; // around z-axis (side to side)
		std::this_thread::sleep_for(std::chrono::microseconds(10));
	}
}

SinGimbal::SinGimbal(double sweepAngle, double sweepFrequency) :
	m_position(Eigen::Vector3d(0, 0, -2)), // 2cm down
	m_running(false),
	m_sweepAngle(sweepAngle), m_sweepFrequency(sweepFrequency) {
}

void SinGimbal::start() {
	if(!m_running) {
		m_running = true;
		m_thread = std::thread(updateGimbal, &m_orientation, &m_running);
		uav::thread::setPriority(m_thread, SCHED_FIFO, 98);
	}
}

void SinGimbal::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
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
	stop();
}
