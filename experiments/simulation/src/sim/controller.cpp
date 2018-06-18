/*
 * controller.cpp
 *
 *  Created on: Jun 9, 2018
 *      Author: rob
 */

#include <chrono>

#include "sim/controller.hpp"
#include "sim/platform.hpp"
#include "util.hpp"

using namespace uav::sim;
using namespace uav::util;

Controller::Controller() :
		m_platform(nullptr),
		m_lastTickTime(0),
		m_lastAltitudeTime(0) {

}

void Controller::start() {
	Clock::addObserver(this, 0.01);
	Clock::start();
	m_platform->start();
}

void Controller::stop() {
	m_platform->stop();
	Clock::stop();
	Clock::removeObserver(this);
}

void Controller::setPlatform(uav::Platform* platform) {
	m_platform = platform;
}

void Controller::tick(double time) {
	const uav::PlatformState& state = m_platform->platformState();
	uav::sim::PlatformControlInput input;
	if(state.altitude() != 10 && state.altitudeTime() > m_lastAltitudeTime) {
		// Check for a recent altitude measurement. If there is one, adjust.
		input.setAltitude(10);
		m_lastAltitudeTime = state.altitudeTime();
	}
	m_lastTickTime = time;
	m_platform->setControlInput(input);
}

Controller::~Controller() {
	stop();
}
