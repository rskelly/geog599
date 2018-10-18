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

//#define SF30C_API_IMPLEMENTATION

//#include "sf30capi.hpp"


/**
 * Provides interactivity with the LightWare Optoelectronics SF30/C
 * laser rangefinder over USB or I2C.
 */
class LWSF30C : public Serial {
private:
	//lwSF30C m_device;

public:

	/**
	 * Connect to the device at the given path, with the given address.
	 *
	 * @param props Connection properties.
	 */
	LWSF30C(const Properties& props) : Serial(props) {}

	bool sendCommand(char mnemonic, int value = -1) {
		char buf[32];
		if(value > -1) {
			sprintf(buf, "#%c%d:", mnemonic, value);
		} else {
			sprintf(buf, "#%c:", mnemonic);
		}
		int len = strlen(buf);
		int wrt = util::_write(m_fd, buf, len);
		usleep(len * 100);
		return wrt;
	}

	/**
	 * Set the resolution; one of:
	 *
	 * 0 = 0.25 m
	 * 1 = 0.12 m
	 * 2 = 0.06 m
	 * 3 = 0.03 m
	 * 4 = Smoothed
	 */
	bool setResolution(int value) {
		return sendCommand('R', value);
	}

	bool setSerialRate(int rate) {
		return sendCommand('U', rate);
	}

	bool startLaser() {
		return sendCommand('Y');
	}

	bool stopLaser() {
		return sendCommand('N');
	}

	bool setDistance() {
		return sendCommand('p', 0);
	}

	bool setSpeed() {
		return sendCommand('p', 1);
	}

};


#endif /* INCLUDE_LWSF30C_HPP_ */
