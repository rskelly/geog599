#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <linux/i2c-dev.h> // I2C bus definitions
#include <signal.h>
#include <math.h>
#include <stdio.h>

#include <vector>
#include <string>
#include <iostream>

#include "serial.hpp"


Serial::Serial(const std::string& dev, long addr) :
	m_dev(dev),
	m_addr(addr),
	m_fd(0) {
}

int Serial::write(char* buf, int len) {
	return util::_write(m_fd, buf, len);
}

int Serial::read(char* buf, int len) {
	return util::_read(m_fd, buf, len);
}

int Serial::read(std::vector<char>& buf) {
	int len = buf.size();
	return util::_read(m_fd, buf.data(), len);
}

int Serial::write(std::vector<char>& buf, int len = -1) {
	if(len == -1)
		len = (int) buf.size();
	return util::_write(m_fd, buf.data(), len);
}


bool I2C::open() {
	std::cout << "Opening I2C device at " << m_dev << ".\n";
	if((m_fd = util::_open(m_dev.c_str(), O_RDWR)) < 0) {
		std::cerr << "Failed to open device at " << m_dev << ".\n";
		return false;
	}
	std::cerr << "Connecting to I2C device at " << m_addr << " (" << m_fd << ").\n";
	if(ioctl(m_fd, I2C_SLAVE, m_addr) < 0) {
		perror("Failed to configure device");
		util::_close(m_fd);
		return false;
	}
	return true;
}

void I2C::close() {
	util::_close(m_fd);
}
	
