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

double RangeBridge::getRange() {
	return 0;
}

Rangefinder::Rangefinder() :
	m_scanFreq(1), m_pulseFreq(1),
	m_nextTime(0) {
}

void Rangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

void Rangefinder::setScanFrequency(double freq) {
	m_scanFreq = freq;
}

uav::Range* Rangefinder::range() {
	Range* result = nullptr;

	timeval time;
	gettimeofday(&time, NULL);
	double t = (double) time.tv_sec + ((double) time.tv_usec / 1000000);

	if(t >= m_nextTime)
		result = new Range(RangeBridge::getRange(), t);

	m_nextTime = t + m_poisson.next(m_pulseFreq);
	return result;
}

Rangefinder::~Rangefinder() {
}
