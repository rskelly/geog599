/*
 * rangefinder.cpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

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
	m_scanParam1(0), m_scanParam2(0) {
}

void SimRangefinder::setDEM(const std::string& filename) {

}

void SimRangefinder::setOrientation(/* matrix here */) {

}

void SimRangefinder::setScanType(ScanType type, double param1 = 0, double param2 = 0) {

}

Range* range() const {

}

~SimRangefinder() {}
