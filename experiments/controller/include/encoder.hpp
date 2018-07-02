/*
 * Encoder.hpp
 *
 *  Created on: Jun 30, 2018
 *      Author: rob
 */

#ifndef INCLUDE_ENCODER_HPP_
#define INCLUDE_ENCODER_HPP_

#include <thread>

namespace uav {
namespace sensor {

constexpr char asd_addr = 0b01001000; ///<! Address byte for ASD1115 used when addr -> ground.

/**
 * This class represents the angle encoder on which the laser
 * rides. It reports its current angle at intervals.
 */
class Encoder {
private:
	std::thread m_thread;
	bool m_running;
	int m_fd; 				///<! File descriptor for encoder's device.
	double m_value;			///<! The current value of the sensor.

	/**
	 * Configure the device to read either single-shot or
	 * continuous at the given address.
	 *
	 * @param addr The address, AN#.
	 * @param mode Single-shot or continuous.
	 */
	void configure(int addr, int mode);

public:

	Encoder();

	/**
	 * Start reading from the device.
	 */
	void start();

	/**
	 * Stop reading from the device.
	 */
	void stop();

	/**
	 * Return the current value or NaN if there isn't one.
	 *
	 * @return The current value or NaN if there isn't one.
	 */
	double value() const;

	/**
	 * Get and the current value from the device.
	 *
	 * @return The current value from the device.
	 */
	int readValue();
};

} // sensor
} // uav


#endif /* INCLUDE_ENCODER_HPP_ */
