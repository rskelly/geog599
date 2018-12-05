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

	bool readData(int& range, int& rangeTime, float& angle, int& angleTime, int* gyro, int* acc);
};

} // sensor

#endif /* INCLUDE_LWSF30C_HPP_ */
