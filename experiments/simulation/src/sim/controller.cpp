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

void _run(Controller* controller, bool* running) {
	double time = -1;
	while(*running) {
		double time0 = uavtime();
		if(time == -1) {
			time = time0;
		} else {
			controller->tick(time0 - time);
			time = time0;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

Controller::Controller() :
		m_platform(nullptr),
		m_running(false) {
}

void Controller::start() {
	if(!m_running) {
		m_running = true;
		m_thread = std::thread(_run, this, &m_running);
	}
}

void Controller::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
}

void Controller::setPlatform(uav::Platform* platform) {
	m_platform = platform;
}

void Controller::tick(double time) {
	m_platform->update(time);
	const uav::PlatformState& state = m_platform->platformState();
	uav::sim::PlatformControlInput input;
	if(state.altitude() != 10) {
		input.setAltitude(10);
		m_platform->setControlInput(input);
	}
}

Controller::~Controller() {
	stop();
}
