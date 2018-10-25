/*
 * ads1115.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <iostream>
#include <string>

#include <unistd.h>

#include "minimu9v5.hpp"


MinIMU9v5::MinIMU9v5(const std::string& dev, uint8_t gyroAddr, uint8_t accelAddr) {
	m_gyro = I2C(dev, gyroAddr);
	m_accel = I2C(dev, accelAddr);
}

bool MinIMU9v5::configGyro(uint8_t reg, uint8_t value) {
	if(!m_gyro.writeByteData(reg, value)) {
		std::cerr << "Failed to configure gyro: " << reg << ", " << value << ".\n";
		return false;
	}
	return true;
}

bool MinIMU9v5::configAccel(uint8_t reg, uint8_t value) {
	if(!m_accel.writeByteData(reg, value)) {
		std::cerr << "Failed to configure accel: " << reg << ", " << value << ".\n";
		return false;
	}
	return true;
}

bool MinIMU9v5::open() {
	if(m_gyro.open()) {// && m_accel.open()) {
		if(!configGyro(CTRL2_G, 0b10001100))
			return false;
		if(!configGyro(CTRL7_G, 0b00000000))
			return false;
		if(!configGyro(CTRL1_XL, 0b10001100))
			return false;
		if(!configGyro(CTRL3_C, 0b00000100))
			return false;
		return true;
	}
	return false;

}

void MinIMU9v5::close() {
	m_gyro.close();
//	m_accel.close();
}

MinIMU9v5::~MinIMU9v5() {
	close();
}

bool MinIMU9v5::hasGyro() {
	uint8_t val = 0;
	if(m_gyro.readByteData(WHO_AM_I, val))
		return val == 0b01101001;
	return false;
}

bool MinIMU9v5::hasAccel() {
	uint8_t val = 0;
	if(m_gyro.readByteData(WHO_AM_I, val))
		return val == 0b01101001;
	return false;
}

bool MinIMU9v5::getState(MinIMU9v5State& state) {

}


