/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#include <chrono>
#include <thread>

#include "sim/rangefinder.hpp"
#include "util.hpp"

using namespace uav::util;

namespace uav {
namespace sim {

class Range : public uav::Range {
friend Rangefinder;
private:
	double m_range;
	double m_time;
protected:
	Range(double range, double time) :
		m_range(range), m_time(time) {
	}
public:
	double range() const {
		return m_range;
	}
	double time() const {
		return m_time;
	}
	bool valid() const {
		return !std::isnan(m_range) && m_time > 0 && !std::isnan(m_time);
	}
};

} // sim
} // uav


using namespace uav::sim;

RangeBridge::RangeBridge() :
	m_terrain(nullptr),
	m_lastTime(0) {
}

double RangeBridge::getRange() {
	m_lastTime = Clock::currentTime();
	return m_terrain->range(m_position, m_direction);
}

void RangeBridge::setLaser(const Eigen::Vector3d& position, const Eigen::Vector3d& direction) {
	m_prevPosition = m_position;
	m_prevDirection = m_direction;
	m_position = position;
	m_direction = direction;
}

void RangeBridge::setTerrain(Terrain* terrain) {
	m_terrain = terrain;
}

Terrain* RangeBridge::terrain() {
	return m_terrain;
}


Rangefinder::Rangefinder() :
	m_obs(nullptr),
	m_bridge(nullptr),
	m_pulseFreq(5.0),
	m_nextTime(0),
	m_maxRange(100),
	m_rangeError(1),
	m_running(false) {

	m_gauss.setMean(0);
	m_gauss.setStdDev(m_rangeError);
}

void Rangefinder::setObserver(uav::RangefinderObserver* obs) {
	m_obs = obs;
}

void Rangefinder::setMaxRange(double range, double error) {
	m_maxRange = range;
	m_rangeError = error;
	m_gauss.setStdDev(m_rangeError);
}

void Rangefinder::tick(double time) {
	generatePulse();
}

void Rangefinder::start() {
	Clock::addObserver(this, 1.0 / m_pulseFreq);
}

void Rangefinder::stop() {
	Clock::removeObserver(this);
}


void Rangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

void Rangefinder::generatePulse() {
	double range = m_bridge->getRange();
	if(range <= m_maxRange) {
		// If the range is within the max range, generate a pulse.
		double time = Clock::currentTime();
		// The error is scaled from zero at zero range, to 1*SD at max range.
		double error = (range / m_maxRange) * m_gauss.next();
		// Send the range update.
		m_obs->rangeUpdate(this, new Range(range + error, time));
	}
}

void Rangefinder::setRangeBridge(RangeBridge* bridge) {
	m_bridge = bridge;
}

RangeBridge* Rangefinder::rangeBridge() const {
	return m_bridge;
}

Rangefinder::~Rangefinder() {
	if(m_bridge)
		delete m_bridge;
}
