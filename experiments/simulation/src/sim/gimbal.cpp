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

constexpr double G_CLOCK_DELAY = 1.0 / 500.0 / 2.0;

SinGimbal::SinGimbal(double sweepAngle, double sweepFrequency) :
	m_position(Eigen::Vector3d(0, 0, -2)), // 2cm down
	m_running(false),
	m_sweepAngle(sweepAngle), m_sweepFrequency(sweepFrequency) {

}

void SinGimbal::tick(double time) {
	double a = std::sin(time * PI * 2) * (PI / 9);
	m_orientation[2] = a; // around z-axis (side to side)
}

void SinGimbal::start() {
	Clock::addObserver(this, G_CLOCK_DELAY);
}

void SinGimbal::stop() {
	Clock::removeObserver(this);
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
