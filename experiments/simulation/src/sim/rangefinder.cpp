/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#include <time.h>
#include <sys/time.h>

#include "sim/rangefinder.hpp"

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
	m_terrain(nullptr) {
}

double RangeBridge::getRange() {
	return m_terrain->range(m_position, m_direction);
}

void RangeBridge::setLaser(const Eigen::Vector3d& position, const Eigen::Vector3d& direction) {
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
	m_bridge(nullptr),
	m_pulseFreq(1. / 866.),
	m_nextTime(0) {
}

void Rangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

int Rangefinder::getRanges(std::vector<uav::Range*>& ranges) {
	double r = m_bridge->getRange();
	if(!std::isnan(r) && r < 100.0) { // TODO: Configurable max range.
		ranges.push_back(new Range(m_bridge->getRange(), uav::util::uavtime()));
		return 1;
	}
	return 0;
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
