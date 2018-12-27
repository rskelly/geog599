/*
 * serialbridge.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#include "impl/serialbridge.hpp"
#include "sensor/teensy.hpp"

using namespace uav::impl;

SerialBridge* __inst;

void _run(SerialBridge* bridge) {

	sensor::Teensy teensy;
	if(!teensy.open("/dev/ttyACM0", B115200))
		throw std::runtime_error("Failed to connect to scanner.");

	while(bridge->running())
		bridge->update(teensy);
}

SerialBridge& SerialBridge::getInstance() {
	if(!__inst) __inst = new SerialBridge();
	return *__inst;
}

SerialBridge::SerialBridge() :
	m_running(false) {
}

bool SerialBridge::running() const {
	return m_running;
}

void SerialBridge::update(sensor::Teensy& teensy) {
	teensy.readData(m_ranges, m_orientation);
	for(SerialBridgeListener* l : m_listeners)
		l->serialBridgeUpdate(this);
}

void SerialBridge::start() {
	if(!m_running) {
		m_running = true;
		m_thread = std::thread(_run, this);
	}
}

void SerialBridge::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
}

const sensor::Orientation& SerialBridge::orientation() const {
	return m_orientation;
}

const std::vector<sensor::Range>& SerialBridge::ranges() const {
	return m_ranges;
}

void SerialBridge::addListener(SerialBridgeListener* listener) {
	m_listeners.insert(listener);
}

void SerialBridge::removeListener(SerialBridgeListener* listener) {
	m_listeners.erase(listener);
}

