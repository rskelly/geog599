/*
 * i2c.cpp
 *
 *  Created on: Oct 24, 2018
 *      Author: rob
 */

extern "C" { // https://stackoverflow.com/questions/50154296/undefined-reference-to-i2c-smbus-read-word-dataint-unsigned-char
	#include <linux/i2c.h>
	#include <linux/i2c-dev.h> // I2C bus definitions
	#include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep

#include <cstring>
#include <iostream>

#include "i2c.hpp"

using namespace comm;

namespace util {
	int _open(const char* path, int flags) {
		return open(path, flags);
	}
	int _close(int file) {
		return close(file);
	}
} // util

void _err(std::ostream& str, const std::string& msg, const std::string& dev, int addr, int err) {
	str << msg << " at " << dev << " (" << addr << "): " << strerror(err) << "\n";
}

bool I2C::readByte(uint8_t& value) {
	int v = 0;
	if((v = i2c_smbus_read_byte(m_fd)) > -1) {
		value = (uint8_t) v;
		return true;
	}
	return false;
}

bool I2C::writeByte(uint8_t value) {
	return i2c_smbus_write_byte(m_fd, value) > -1;
}

bool I2C::readByteData(uint8_t cmd, uint8_t& value) {
	int ret;
	if((ret = i2c_smbus_read_byte_data(m_fd, cmd)) == -1) {
		return false;
	} else {
		value = (uint8_t) ret;
		return true;
	}
}

bool I2C::writeByteData(uint8_t cmd, uint8_t value) {
	return i2c_smbus_write_byte_data(m_fd, cmd, value) != -1;
}

bool I2C::readWordData(uint8_t cmd, uint16_t& value) {
	int ret;
	if((ret = i2c_smbus_read_word_data(m_fd, cmd)) == -1) {
		return false;
	} else {
		value = (uint16_t) ret;
		return true;
	}
}

bool I2C::writeWordData(uint8_t cmd, uint16_t value) {
	return i2c_smbus_write_word_data(m_fd, cmd, value) != -1;
}

bool I2C::readBlockData(uint8_t cmd, std::vector<uint8_t>& data, uint8_t& len) {
	int ret;
	uint8_t buf[32];
	// Kernel 2.6.23 and later.
	if((ret = i2c_smbus_read_i2c_block_data(m_fd, cmd, len, buf)) == -1) {
		return false;
	} else {
		data.resize(ret);
		for(int i = 0; i < ret; ++i)
			data[i] = buf[i];
		len = ret;
		return true;
	}
}

bool I2C::writeBlockData(uint8_t cmd, const std::vector<uint8_t>& data, uint8_t len) {
	uint8_t l = (uint8_t) (len > 32 ? 32 : len);
	return i2c_smbus_write_block_data(m_fd, cmd, l, data.data()) != -1;
}

bool I2C::readBlockData(uint8_t cmd, uint8_t* data, uint8_t& len) {
	int ret;
	uint8_t buf[128];
	// Kernel 2.6.23 and later.
	if((ret = i2c_smbus_read_i2c_block_data(m_fd, cmd, len, buf)) == -1) {
		return false;
	} else {
		for(int i = 0; i < ret; ++i)
			data[i] = buf[i];
		len = ret;
		return true;
	}
}

bool I2C::writeBlockData(uint8_t cmd, const uint8_t* data, uint8_t len) {
	uint8_t l = (uint8_t) (len > 32 ? 32 : len);
	return i2c_smbus_write_block_data(m_fd, cmd, l, data) != -1;
}

I2C::I2C(const std::string& dev, uint8_t addr) :
		m_fd(0),
		m_addr(addr),
		m_dev(dev) {}

I2C::I2C() : I2C("", 0) {}

bool I2C::open(const std::string& dev, uint8_t addr) {
	m_addr = addr;
	m_dev = dev;
	return open();
}

bool I2C::open() {
	std::cout << "Opening I2C device at " << m_dev << ", " << m_addr << ".\n";
	if((m_fd = util::_open(m_dev.c_str(), O_RDWR)) < 0) {
		_err(std::cerr, "Failed to open device", m_dev, m_addr, errno);
		return false;
	}
	std::cerr << "Connecting to I2C device at " << m_addr << " (" << m_fd << ").\n";
	if(ioctl(m_fd, I2C_SLAVE, m_addr) < 0) {
		_err(std::cerr, "Failed to configure device", m_dev, m_addr, errno);
		util::_close(m_fd);
		return false;
	}
	return true;
}

void I2C::close() {
	util::_close(m_fd);
}

I2C::~I2C() {
	close();
}




