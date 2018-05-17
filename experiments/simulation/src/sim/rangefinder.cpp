/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#include <time.h>
#include <gdal_priv.h>

#include "sim/rangefinder.hpp"

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
	m_scanType(ScanType::None),
	m_scanParam1(0), m_scanParam2(0),
	m_scanFreq(1), m_pulseFreq(1),
	m_demds(nullptr), m_demband(1),
	m_nextTime(0) {
}

void SimRangefinder::setDEM(const std::string& filename, int band) {
	m_demds = (GDALDataset*) GDALOpen(filename.c_str(), GA_ReadOnly);
	if(!m_demds)
		throw std::invalid_argument("Failed to open " + filename);
	m_demband = band;
}

void SimRangefinder::setPulseFrequency(double freq) {
	m_pulseFreq = freq;
}

void SimRangefinder::setScanFrequency(double freq) {
	m_scanFreq = freq;
}

void SimRangefinder::setPlatformOrientation(const Eigen::Vector3d& mtx) {
	m_platformOrientation = mtx;
}

void SimRangefinder::setPlatformPosition(const Eigen::Vector3d& mtx) {
	m_platformPosition = mtx;
}

void SimRangefinder::setOrientation(const Eigen::Vector3d& mtx) {
	m_orientation = mtx;
}

void SimRangefinder::setPosition(const Eigen::Vector3d& mtx) {
	m_position = mtx;
}

const Eigen::Vector3d& SimRangefinder::orientation() const {
	return m_orientation;
}

const Eigen::Vector3d& SimRangefinder::position() const {
	return m_position;
}

void SimRangefinder::setScanType(ScanType type, double param1, double param2) {
	m_scanType = type;
	m_scanParam1 = param1;
	m_scanParam2 = param2;
}

#include <sys/time.h>

double SimRangefinder::computeScanAngle(double time) const {
	return std::sin(time / m_scanFreq);
}

double SimRangefinder::computeRange(double angle) const {
	return 0;
}

Range* SimRangefinder::range() {
	Range* result = nullptr;

	timeval time;
	gettimeofday(&time, NULL);
	double t = (double) time.tv_sec + ((double) time.tv_usec / 1000000);

	if(t >= m_nextTime) {
		double angle = computeScanAngle(t);
		double range = computeRange(angle);
		result = new SimRange(range, t);
	}
	m_nextTime = t + m_poisson.next(m_pulseFreq);
	return result;
}

SimRangefinder::~SimRangefinder() {
	if(m_demds)
		GDALClose((void*) m_demds);
}
