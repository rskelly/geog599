/*
 * lwsf30c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_LWSF30C_HPP_
#define INCLUDE_LWSF30C_HPP_

#define PI 			3.1415926535
#define IMU_DIVISOR 32767
#define IMU_ASCALE 	2.0
#define IMU_GSCALE 	245.0
#define TO_RAD 		(PI / 180.0)
#define GRAVITY		9.80665


#define G_CONV ((1.0 / IMU_DIVISOR) * IMU_GSCALE * TO_RAD)
#define A_CONV ((1.0 / IMU_DIVISOR) * IMU_ASCALE)


#include <string>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

#include <Eigen/Dense>

#include "comm/serial.hpp"

using namespace comm;

namespace sensor {

class Range {
private:
	uint16_t m_range;
	uint64_t m_timestamp;

public:

	Range(uint16_t range = 0, uint64_t timestamp = 0) :
		m_range(range), m_timestamp(timestamp) {
	}

	void update(uint16_t range, uint64_t time) {
		m_range = range;
		m_timestamp = time;
	}

	uint16_t range() const {
		return m_range;
	}

	uint64_t timestamp() const {
		return m_timestamp;
	}

};

template <class T>
class MovingAverage {
private:
	int m_count;				///<! The number of elements in the list.
	size_t m_idx;				///<! The current index for adding.
	double m_offset;			///<! The zero-offset for zeroing the output.
	std::vector<T> m_elements;	///<! The list of elements (a circular buffer.)

public:

	/**
	 * Construct a moving average filter with the given number of elements.
	 * @param count The number of elements (default 5).
	 */
	MovingAverage(int count = 5, T offset = 0) :
		m_idx(0) {
		setCount(count);
		setOffset(offset);
	}

	/**
	 * Set the number of elements.
	 * @param count The number of elements.
	 */
	void setCount(int count) {
		m_count = count;
		m_elements.resize(count);
	}

	/**
	 * Set the zero offset.
	 * @param offset The zero offset
	 */
	void setOffset(double offset) {
		m_offset = offset;
	}

	/**
	 * Add an element to the list.
	 * @param v A value to add.
	 */
	void add(T v) {
		m_elements[m_idx++ % m_count] = v;
	}

	/**
	 * Return the average of the elements in the list.
	 * @return The average of the elements in the list.
	 */
	double value() const {
		double s = 0;
		for(const T& v : m_elements)
			s += (double) v;
		return s / m_count + m_offset;
	}
};

class Orientation {
private:
	MovingAverage<int16_t> m_g0;
	MovingAverage<int16_t> m_g1;
	MovingAverage<int16_t> m_g2;
	MovingAverage<int16_t> m_a0;
	MovingAverage<int16_t> m_a1;
	MovingAverage<int16_t> m_a2;
	uint64_t m_timestamp;
	size_t m_count;
	bool m_update;
	Eigen::Vector3d m_displacement;

public:

	Orientation() :
		m_timestamp(0),
		m_count(0),
		m_update(false) {
	}

	bool hasUpdate() const {
		return m_update;
	}

	void update(int16_t g0, int16_t g1, int16_t g2, 
		int16_t a0, int16_t a1, int16_t a2, 
		uint64_t timestamp) {
		if(m_timestamp == 0) {
			m_timestamp = timestamp;
			return;
		}
		if(m_timestamp > timestamp) {

		}
		m_g0.add(g0);
		m_g1.add(g1);
		m_g2.add(g2);
		m_a0.add(a0);
		m_a1.add(a1);
		m_a2.add(a2);
		double dt = timestamp - m_timestamp;
		m_timestamp = timestamp;
		// TODO: Apply the offsets once they're calculated. This is a primitive calibration.
		if(++m_count == 100) {
			m_g0.setOffset(-m_g0.value());
			m_g1.setOffset(-m_g1.value());
			m_g2.setOffset(-m_g2.value());
			m_a0.setOffset(-m_a0.value());
			m_a1.setOffset(-m_a1.value());
			m_a2.setOffset(-m_a2.value());
			m_displacement << 0, 0, 0;
		}
		Eigen::Vector3d acc = accel();
		m_displacement += acc / dt;
		m_update = true;
	}

	Eigen::Vector3d gyro() const {
		return Eigen::Vector3d(
			m_g0.value() * G_CONV,
			m_g1.value() * G_CONV,
			m_g2.value() * G_CONV
		);
	}

	Eigen::Vector3d accel() const {
		return Eigen::Vector3d(
			m_a0.value() * A_CONV, 
			m_a1.value() * A_CONV, 
			m_a2.value() * A_CONV
		);
	}
	
	Eigen::Vector3d displacement() const {
		return m_displacement;
	}

	Eigen::Vector3d orientation() const {
		return Eigen::Vector3d(
			m_a0.value() * A_CONV,
			m_a1.value() * A_CONV,
			m_a2.value() * A_CONV
		);
	}

	uint64_t timestamp() const {
		return m_timestamp;
	}

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

	bool readData(std::vector<Range>& ranges, Orientation& orientation);

};

} // sensor

#endif /* INCLUDE_LWSF30C_HPP_ */
