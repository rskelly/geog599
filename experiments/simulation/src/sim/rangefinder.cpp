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

uav::Range* Rangefinder::range() {
	Range* result = nullptr;

	timeval time;
	gettimeofday(&time, NULL);
	double t = (double) time.tv_sec + ((double) time.tv_usec / 1000000);

	if(t >= m_nextTime) {
		result = new Range(m_bridge->getRange(), t);
		m_nextTime = t + m_poisson.next(m_pulseFreq);
	}

	return result;
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
