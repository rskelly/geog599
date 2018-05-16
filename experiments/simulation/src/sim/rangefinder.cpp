/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#include <time.h>

#include "sim/rangefinder.hpp"

class SimRange : public Range {
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
	m_scanType(ScanType::None),
	m_scanParam1(0), m_scanParam2(0),
	m_scanFreq(1), m_pulseFreq(1),
	m_demds(nullptr), m_demband(1),
	m_nextTime(0) {

	std::poisson_distribution<double>::param_type m(m_pulseFreq);
	m_generator.seed(1.0);
	m_distribution.param(m);
}

void SimRangefinder::setDEM(const std::string& filename, int band = 1) {
	m_demds = (GDALDataset*) GDALOpen(filename.c_str(), GA_ReadOnly);
	if(!m_demds)
		throw std::invalid_argument("Failed to open " + filename);
	m_demband = band;
}

void SimRangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
	std::poisson_distribution<double>::param_type m(m_pulseFreq);
	m_distribution.param(m);
}

void SimRangefinder::setScanFrequency(double freq) {
	m_scanFreq = freq;
}

void SimRangefinder::setOrientation(const Eigen::Matrix3d& mtx) {

}

void SimRangefinder::setPosition(const Eigen::Matrix3d& mtx) {

}

void SimRangefinder::setScanType(ScanType type, double param1 = 0, double param2 = 0) {
	m_scanType = type;
	m_scanParam1 = param1;
	m_scanParam2 = param2;
}

#include <sys/time.h>

double SimRangefinder::computeScanAngle(double time) const {
	return std::sin(time / m_scanFreq);
}

double SimRangefinder::computeRange(double angle) const {

}

Range* SimRangefinder::range() const {
	Range* result = nullptr;

	timeval time;
	gettimeofday(&time, NULL);
	double t = (double) time.tv_sec + ((double) time.tv_usec / 1000000);

	if(t >= m_nextTime) {
		double angle = computeScanAngle(t);
		double range = computeRange(angle);
		result = new SimRange(range, t);
	}
	m_nextTime = t + m_distribution(m_generator);
	return result;
}

SimRangefinder::~SimRangefinder() {
	if(m_demds)
		GDALClose((void*) m_demds);
}
