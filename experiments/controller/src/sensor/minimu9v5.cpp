/*
 * ads1115.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <iostream>
#include <string>
#include <cstring>

#include <unistd.h>

#include "sensor/minimu9v5.hpp"

using namespace sensor;
using namespace comm;

MinIMU9v5::MinIMU9v5(const std::string& dev, uint8_t gyroAddr, uint8_t magAddr) {
	m_gyro = I2C(dev, gyroAddr);
	m_mag = I2C(dev, magAddr);
}

bool MinIMU9v5::configGyro(uint8_t reg, uint8_t value) {
	if(!m_gyro.writeByteData(reg, value)) {
		std::cerr << "Failed to configure gyro: " << reg << ", " << value << ".\n";
		return false;
	}
	return true;
}

bool MinIMU9v5::configMag(uint8_t reg, uint8_t value) {
	if(!m_mag.writeByteData(reg, value)) {
		std::cerr << "Failed to configure mag: " << reg << ", " << value << ".\n";
		return false;
	}
	return true;
}

bool MinIMU9v5::open() {
	if(m_gyro.open()) {
		// Gyroscope -- Data rate: 1.66kHz; Full scale: 245dps; Full-scale @ 125: disabled
		if(!configGyro(GyroReg::CTRL2_G, 0b10000000))
			return false;
		// Gyroscope -- High performance: enabled; High-pass filter: disabled; HP filter reset: off; Rounding status: disabled; High-pass cutoff: 0.0081Hz
		if(!configGyro(GyroReg::CTRL7_G, 0b00000000))
			return false;
		// Accelerometer -- Data rate: 1.66kHz; Full scale: +-2g; Anti-aliasing filter bandwidth: 400Hz
		if(!configGyro(GyroReg::CTRL1_XL, 0b10001100))
			return false;
		// Control -- automatically increment register byte on multiple-byte access.
		if(!configGyro(GyroReg::CTRL3_C, 0b00000100))
			return false;
		return true;
	}
	if(m_mag.open()) {
		// TODO: Not using the magnetometer at the moment.
	}
	return false;

}

void MinIMU9v5::close() {
	m_gyro.close();
	m_mag.close();
}

MinIMU9v5::~MinIMU9v5() {
	close();
}

bool MinIMU9v5::hasGyro() {
	uint8_t val = 0;
	if(m_gyro.readByteData(GyroReg::G_WHO_AM_I, val))
		return val == 0b01101001;
	return false;
}

bool MinIMU9v5::hasMag() {
	uint8_t val = 0;
	if(m_mag.readByteData(MagReg::M_WHO_AM_I, val))
		return val == 0b01101001;
	return false;
}

bool _readWord(I2C& device, uint8_t cmda, uint8_t cmdb, uint16_t& value) {
	uint8_t a = 0, b = 0;
	if(device.readByteData(cmda, a) && device.readByteData(cmdb, b)) {
		value = (a << 8) | b;
		return true;
	} else {
		std::cerr << "Failed to read from device: " << cmda << ", " << cmdb << "\n";
		return false;
	}
}

bool _readInt(I2C& device, uint8_t cmda, uint8_t cmdb, uint8_t cmdc, uint32_t& value) {
	uint8_t a = 0, b = 0, c = 0;
	if(device.readByteData(cmda, a) && device.readByteData(cmdb, b), device.readByteData(cmdc, c)) {
		value = (a << 16) | (b << 8) | c;
		return true;
	} else {
		std::cerr << "Failed to read from device: " << cmda << ", " << cmdb << ", " << cmdc << "\n";
		return false;
	}
}

bool MinIMU9v5::getState(MinIMU9v5State& state) {
	// 1) get angular values
	// 2) get linear values
	// 3) get timestamp

	uint8_t xyz[6];
	uint8_t len;

	state.reset();

	// Read the angular component. See GyroReg::CTRL3_C setting in configGyro
	len = 6;
	if(m_gyro.readBlockData(GyroReg::OUTX_H_G, xyz, len) && len == 6)
		state.setAngular(xyz);

	/*
	if(_readWord(m_gyro, GyroReg::OUTX_H_G, GyroReg::OUTX_L_G, xyz[0])
			&& _readWord(m_gyro, GyroReg::OUTY_H_G, GyroReg::OUTY_L_G, xyz[2])
			&& _readWord(m_gyro, GyroReg::OUTZ_H_G, GyroReg::OUTZ_L_G, xyz[4]))
		state.setAngular(xyz);
	*/

	// Read the linear component. See GyroReg::CTRL3_C setting in configGyro
	len = 6;
	if(m_gyro.readBlockData(GyroReg::OUTX_H_XL, xyz, len) && len == 6)
		state.setLinear(xyz);

	/*
	if(_readWord(m_gyro, GyroReg::OUTX_H_XL, GyroReg::OUTX_L_XL, xyz[0])
			&& _readWord(m_gyro, GyroReg::OUTY_H_XL, GyroReg::OUTY_L_XL, xyz[2])
			&& _readWord(m_gyro, GyroReg::OUTZ_H_XL, GyroReg::OUTZ_L_XL, xyz[4]))
		state.setLinear(xyz);
	*/

	// Read the timestamp. See GyroReg::CTRL3_C setting in configGyro
	len = 3;
	if(m_gyro.readBlockData(GyroReg::TIMESTAMP0_REG, xyz, len) && len == 3)
		state.setTimestamp(xyz);

	/*
	if(_readInt(m_gyro, GyroReg::TIMESTAMP0_REG, GyroReg::TIMESTAMP1_REG, GyroReg::TIMESTAMP2_REG, xyz))
		state.setTimestamp(xyz);
	*/

	return state.updated();
}


int main(int argc, char** argv) {
	return 0;
}

