/*
 * lwsf30c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_LWSF30C_HPP_
#define INCLUDE_LWSF30C_HPP_

#include <string>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

#include "comm/serial.hpp"

using namespace comm;

namespace sensor {

class Range {
private:
	int m_maxAngle;
	int m_average;
	std::vector<int> m_angles;
	size_t m_angleIdx;
	bool m_angleUpdate;
	float m_angle;
	int m_status;
	int m_range;
	unsigned long m_timestamp;

public:

	Range(int maxAngle = 1023, int average = 5) :
		m_maxAngle(maxAngle),
		m_average(average),
		m_angleIdx(0),
		m_angleUpdate(false),
		m_angle(0),
		m_status(0),
		m_range(0),
		m_timestamp(0) {
		
		m_angles.resize(m_average);
	}

	void setStatus(int status) {
		m_status = status;
	}

	int status() const {
		return m_status;
	}

	void addRange(int range) {
		m_range = range;
	}

	int range() const {
		return m_range;
	}

	void setTimestamp(unsigned long timestamp) {
		m_timestamp = timestamp;
	}

	unsigned long timestamp() const {
		return m_timestamp;
	}

	void addAngle(int angle) {
		m_angleUpdate = true;
		m_angles[m_angleIdx % m_angles.size()] = angle;
		++m_angleIdx;
	}

	float angle() {
		if(m_angleUpdate) {
			int a = 0;
			for(size_t i = 0; i < m_angles.size(); ++i)
				a += m_angles[i];
			m_angle = ((float) a / m_angles.size()) / m_maxAngle;
			m_angleUpdate = false;
		} else {
			return m_angle;
		}
	}

	// Relative to scan axis; cm.
	float x() {
		return std::sin((angle() / m_maxAngle) * M_PI * 2.0) * m_range;
	}

	// Relative to scan axis; cm.
	float y() {
		return std::cos((angle() / m_maxAngle) * M_PI * 2.0) * m_range;
	}

	float z() {
		return 0.0;
	}

};

class Orientation {

};

/**
 * Provides interactivity with the LightWare Optoelectronics SF30/C
 * laser rangefinder over USB or I2C.
 */
class Teensy : public Serial {
private:

public:

	/**
	 * Connect to the device at the given path, with the given address.
	 *
	 * @param props Connection properties.
	 */
	Teensy(const std::string& dev, int speed = B115200);

	/**
	 * Create an unconfigured device.
	 */
	Teensy();

	/**
	 * Connect to the device.
	 *
	 * @return True on success.
	 */
	bool open();

	/**
	 * Connect to the device.
	 *
	 * @param dev The device path.
	 * @param speed The baud rate.
	 * @return True on success.
	 */
	bool open(const std::string& dev, int speed = B115200);

	bool readData(Range& range, Orientation& orientation);

};

} // sensor

#endif /* INCLUDE_LWSF30C_HPP_ */
