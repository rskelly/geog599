/*
 * RangefinderSF30C.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#include "impl/rangefindersf30c.hpp"
#include "impl/serialbridge.hpp"

using namespace uav::impl;

void RangeSF30C::update(double range, double time) {
	m_range = range;
	m_time = time;
}

double RangeSF30C::range() const {
	return m_range;
}

double RangeSF30C::time() const {
	return m_time;
}

bool RangeSF30C::valid() const {
	return true;
}

void RangefinderSF30C::setObserver(RangefinderObserver* obs) {
	m_observer = obs;
}

void RangefinderSF30C::start() {
	SerialBridge::getInstance().addListener(this);
	SerialBridge::getInstance().start();
}

void RangefinderSF30C::stop() {
	SerialBridge::getInstance().removeListener(this);
}

void RangefinderSF30C::serialBridgeUpdate(SerialBridge* bridge) {
	for(const sensor::Range& range : bridge->ranges()) {
		m_range.update(range.range(), range.timestamp());
		m_observer->rangeUpdate(this, &m_range);
	}
}

RangefinderSF30C::~RangefinderSF30C() {}
