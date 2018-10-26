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

MinIMU9v5::MinIMU9v5(const std::string& dev, uint8_t gyroAddr, uint8_t magAddr) :
	m_dev(dev),
	m_gyroAddr(gyroAddr),
	m_magAddr(magAddr),

	m_gyroDataRate(GDR_1660Hz),
	m_gyroFullScale(GFS_245dps),
	m_accelDataRate(ADR_1660Hz),
	m_accelFullScale(AFS_2g),
	m_accelFilterBW(AAFB_400Hz),
	m_autoInc(AUTO_INC_ON) {
}

MinIMU9v5::MinIMU9v5() :
	MinIMU9v5("", 0, 0) {
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

void MinIMU9v5::setGyroDataRate(GyroConfig config) {
	m_gyroDataRate = config;
}

void MinIMU9v5::setGyroFullScale(GyroConfig config) {
	m_gyroFullScale = config;
}

void MinIMU9v5::setAccelDataRate(GyroConfig config) {
	m_accelDataRate = config;
}

void MinIMU9v5::setAccelFullScale(GyroConfig config) {
	m_accelFullScale = config;
}

void MinIMU9v5::setAccelFilterBandwidth(GyroConfig config) {
	m_accelFilterBW = config;
}

void MinIMU9v5::setAddrAutoIncrement(GyroConfig config) {
	m_autoInc = config;
}

bool MinIMU9v5::open() {
	if(m_gyro.open(m_dev, m_gyroAddr)) {
		// Gyroscope: data rate, full-scale, full-scale @ 125 disabled.
		uint8_t config = 0b00000000 | (m_gyroDataRate << 4) | (m_gyroFullScale << 2);
		if(!configGyro(GyroReg::CTRL2_G, config))
			return false;
		// Gyroscope -- High performance: enabled; High-pass filter: disabled; HP filter reset: off; Rounding status: disabled; High-pass cutoff: 0.0081Hz
		config = 0b00000000;
		if(!configGyro(GyroReg::CTRL7_G, config))
			return false;
		// Accelerometer -- Data rate: 1.66kHz; Full scale: +-2g; Anti-aliasing filter bandwidth: 400Hz
		config = 0b00000000 | (m_accelDataRate << 4) | (m_accelFullScale << 2) | (m_accelFilterBW << 1);
		if(!configGyro(GyroReg::CTRL1_XL, config))
			return false;
		// Control -- automatically increment register byte on multiple-byte access.
		config = 0b00000000 | (m_autoInc << 3);
		if(!configGyro(GyroReg::CTRL3_C, config))
			return false;
		return true;
	}
	if(m_mag.open(m_dev, m_magAddr)) {
		// TODO: Not using the magnetometer at the moment.
	}
	return false;

}

bool MinIMU9v5::open(const std::string& dev, uint8_t gyroAddr, uint8_t magAddr) {
	m_dev = dev;
	m_gyroAddr = gyroAddr;
	m_magAddr = magAddr;
	return open();
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

/*
int main(int argc, char** argv) {
	return 0;
}
*/
