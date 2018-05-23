/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#include <time.h>
#include <sys/time.h>

#include "sim/rangefinder.hpp"

using namespace uav;
using namespace uav::sim;

double SimRangeBridge::getRange() {
	return 0;
}

class SimRange : public Range {
friend SimRangefinder;
private:
	double m_range;
	double m_time;
protected:
	SimRange(double range, double time) :
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

SimRangefinder::SimRangefinder() :
	m_scanFreq(1), m_pulseFreq(1),
	m_nextTime(0) {
}

void SimRangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

void SimRangefinder::setScanFrequency(double freq) {
	m_scanFreq = freq;
}

Range* SimRangefinder::range() {
	Range* result = nullptr;

	timeval time;
	gettimeofday(&time, NULL);
	double t = (double) time.tv_sec + ((double) time.tv_usec / 1000000);

	if(t >= m_nextTime)
		result = new SimRange(SimRangeBridge::getRange(), t);

	m_nextTime = t + m_poisson.next(m_pulseFreq);
	return result;
}

SimRangefinder::~SimRangefinder() {
}
