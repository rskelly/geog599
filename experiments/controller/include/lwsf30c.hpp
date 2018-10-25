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

#include "serial.hpp"

using namespace comm;

namespace sensor {

/**
 * Provides interactivity with the LightWare Optoelectronics SF30/C
 * laser rangefinder over USB or I2C.
 */
class LWSF30C : public Serial {
private:

public:

	/**
	 * Connect to the device at the given path, with the given address.
	 *
	 * @param props Connection properties.
	 */
	LWSF30C(const std::string& dev, int speed = B115200);

	/**
	 * Send a command to the laser.
	 *
	 * @param mnemonic The letter-code used to identify the command.
	 * @param value A numeric value for the argument, or -1 if no argument.
	 * @return True if the command is sent successfully.
	 */
	bool sendCommand(char mnemonic, int value = -1);

	/**
	 * Return the latest measurement. This may be
	 * distance or speed. Reads will be delayed according
	 * to the serial update rate.
	 *
	 * @return The measurement.
	 */
	double getMeasurement();

	/**
	 * Set the resolution; one of:
	 *
	 * 0 = 0.25 m
	 * 1 = 0.12 m
	 * 2 = 0.06 m
	 * 3 = 0.03 m
	 * 4 = Smoothed
	 *
	 * @param The resolution code.
	 * @return True if successful.
	 */
	bool setResolution(int value);

	/**
	 * Set the serial port update rate. Whole number factors of 18317 only.
	 *
	 * @param rate The update rate.
	 * @return True if successful.
	 */
	bool setSerialRate(int rate);

	/**
	 * Start the laser.
	 *
	 * @return True if successful.
	 */
	bool startLaser();

	/**
	 * Stop the laser.
	 *
	 * @return True if successful.
	 */
	bool stopLaser();

	/**
	 * Set the laser to distance mode.
	 *
	 * @return True if successful.
	 */
	bool setDistance();

	/**
	 * Set the laser to speed mode.
	 *
	 * @return True if successful.
	 */
	bool setSpeed();

};

} // sensor

#endif /* INCLUDE_LWSF30C_HPP_ */
