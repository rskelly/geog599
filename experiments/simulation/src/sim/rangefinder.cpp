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
	m_lastTime = uavtime();
	return m_terrain->range(m_position, m_direction);
}

void RangeBridge::getInterpRanges(int pulseCount, std::vector<double>& ranges, std::vector<double>& times) {
	double time = uavtime();
	if(m_lastTime == 0) {
		m_lastTime = time;
		return;
	} else {
		int count = (int) std::max(1.0, (time - m_lastTime) * pulseCount);
		Eigen::Vector3d posStep = (m_position - m_prevPosition) / count;
		Eigen::Vector3d dirStep = (m_direction - m_prevDirection) / count;
		double timeStep = (time - m_lastTime) / count;
		for(int i = 0; i < count; ++i) {
			double r = m_terrain->range(m_prevPosition + posStep * i, m_prevDirection + dirStep * i);
			if(r > 100.0) // TODO: Configurable max range.
				continue;
			double t = time + timeStep + i;
			ranges.push_back(r);
			times.push_back(t);
		}
		m_lastTime = time;
	}
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


void __runRangefinder(Rangefinder* rangefinder, double freq, bool* running) {
	int sleep = 1.0 / freq * 1000000.0;
	while(*running) {
		rangefinder->generatePulse();
		std::this_thread::sleep_for(std::chrono::microseconds(sleep));
	}
}

Rangefinder::Rangefinder() :
	m_obs(nullptr),
	m_bridge(nullptr),
	m_pulseFreq(5.0),
	m_nextTime(0),
	m_running(false) {
}

void Rangefinder::setObserver(uav::RangefinderObserver* obs) {
	m_obs = obs;
}

void Rangefinder::start() {
	if(!m_running) {
		m_running = true;
		m_thread = std::thread(__runRangefinder, this, m_pulseFreq, &m_running);
	}
}

void Rangefinder::stop() {
	if(m_running) {
		m_running = false;
		if(m_thread.joinable())
			m_thread.join();
	}
}


void Rangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

void Rangefinder::generatePulse() {
	m_obs->rangeUpdate(this, new Range(m_bridge->getRange(), uavtime()));
}

int Rangefinder::getRanges(std::vector<uav::Range*>& ranges) {
	std::vector<double> r;
	std::vector<double> t;
	m_bridge->getInterpRanges((int) m_pulseFreq, r, t);
	for(size_t i = 0; i < r.size(); ++i)
		ranges.push_back(new Range(r[i], t[i]));
	return ranges.size();
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
