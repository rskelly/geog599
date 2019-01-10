/*
 * gimbalminimu9v5.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#include "impl/gimbalminimu9v5.hpp"
#include "impl/serialbridge.hpp"
#include "sensor/teensy.hpp"

using namespace uav::impl;
using namespace sensor;

void GimbalMinIMU9v5::setOrientation(const Eigen::Vector3d& orientation) {
	m_orientation = orientation;
}

const Eigen::Vector3d& GimbalMinIMU9v5::orientation() const {
	return m_orientation;
}

void GimbalMinIMU9v5::setPosition(const Eigen::Vector3d& position)  {
	m_position = position;
}

const Eigen::Vector3d& GimbalMinIMU9v5::position() const {
	return m_position;
}

void GimbalMinIMU9v5::setStaticOrientation(const Eigen::Vector3d& mtx) {
	m_staticOrientation = mtx;
}

const Eigen::Vector3d& GimbalMinIMU9v5::staticOrientation() const {
	return m_staticOrientation;
}

void GimbalMinIMU9v5::setStaticPosition(const Eigen::Vector3d& mtx) {
	m_staticPosition = mtx;
}

const Eigen::Vector3d& GimbalMinIMU9v5::staticPosition() const {
	return m_staticPosition;
}

void GimbalMinIMU9v5::start() {
	SerialBridge::getInstance().addListener(this);
	SerialBridge::getInstance().start();
}

void GimbalMinIMU9v5::stop() {
	SerialBridge::getInstance().removeListener(this);
}

void GimbalMinIMU9v5::tick(double time) {

}

void GimbalMinIMU9v5::serialBridgeUpdate(SerialBridge* bridge) {
	const Orientation& so = bridge->orientation();
	uint64_t t1, t0;
	if(m_timestamp == 0) {
		t0 = t1 = m_timestamp;
	} else {
		t1 = m_timestamp;
	}
	Eigen::Vector3d o = orientation();
	o += so.gyro() * (double) (t1 - t0) / 1000000.0;
	setOrientation(o);
}

GimbalMinIMU9v5::~GimbalMinIMU9v5() {

}


