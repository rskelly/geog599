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


MinIMU9v5::MinIMU9v5(const Properties& gyroProps, const Properties& accelProps) {
	m_gyro.configure(gyroProps);
	m_accel.configure(accelProps);
}

bool MinIMU9v5::config_gyro(uint8_t reg, uint8_t value) {
	char config[2] = {reg, value};
	return m_gyro.write(config, 2);
}

bool MinIMU9v5::config_accel(uint8_t reg, uint8_t value) {
	char config[2] = {reg, value};
	return m_accel.write(config, 2);
}

bool MinIMU9v5::open() {
	if(m_qyro.open()) {// && m_accel.open()) {
		if(!m_gyro.config_gyro(CTRL2_G, 0b10001100)) {
			std::cerr << "Failed to configure CTRL2_G.\n";
			return false;
		}
		if(!m_gyro.config_gyro(CTRL7_G, 0b00000000)) {
			std::cerr << "Failed to configure CTRL7_G.\n";
			return false;
		}
		if(!m_gyro.config_gyro(CTRL1_XL, 0b10001100)) {
			std::cerr << "Failed to configure CTRL1_XL.\n";
			return false;
		}
		if(!m_gyro.config_gyro(CTRL3_C, 0b00000100)) {
			std::cerr << "Failed to configure CTRL3_C.\n";
			return false;
		}
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

	m_gyro.write();
}

bool MinIMU9v5::hasAccel() {

}

bool MinIMU9v5::getState(MinIMU9v5State& state) {

}


