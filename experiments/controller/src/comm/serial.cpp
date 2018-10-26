#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#include <cstring>
#include <vector>
#include <string>
#include <iostream>

#include "comm/serial.hpp"

using namespace comm;

namespace _s {

	inline int _open(const char* path, int flags) {
		return open(path, flags);
	}

	inline void _close(int fd) {
		close(fd);
	}

	inline int _read(int fd, char* buf, int len) {
		return read(fd, buf, len);
	}

	inline int _write(int fd, char* buf, int len) {
		return write(fd, buf, len);
	}

} // _s


Serial::Serial(const std::string& dev, int speed) :
	m_dev(dev),
	m_fd(0),
	m_speed(speed) {
}

Serial::Serial() :
	Serial("", 0) {
}

bool Serial::open(const std::string& dev, int speed) {
	m_dev = dev;
	m_speed = speed;
	return open();
}

int Serial::write(char* buf, int len) {
	return _s::_write(m_fd, buf, len);
}

int Serial::read(char* buf, int len) {
	return _s::_read(m_fd, buf, len);
}

int Serial::read(std::vector<char>& buf) {
	int len = buf.size();
	std::cout << "reading " << len << "\n";
	int red = _s::_read(m_fd, buf.data(), len);
	std::cout << "read " << red << " of " << len << "\n";
	return red;
}

int Serial::write(std::vector<char>& buf, int len) {
	if(len == -1)
		len = (int) buf.size();
	return _s::_write(m_fd, buf.data(), len);
}

bool Serial::open() {
	if(m_dev.empty())
		return false;

	if((m_fd = _s::_open(m_dev.c_str(), O_RDWR | O_NOCTTY /*| O_NDELAY*/)) < 0) {
		std::cerr << "Error opening serial device at: " << m_dev << " (" << strerror(errno) << ")\n";
		return false;
	}
	
	struct termios tty;
	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(m_fd, &tty) < 0) {
		std::cerr << "Error from tcgetattr: " <<  strerror(errno) << "\n";
		return -1;
	}

	cfsetospeed(&tty, (speed_t) m_speed);
	cfsetispeed(&tty, (speed_t) m_speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         		/* 8-bit characters */
	tty.c_cflag &= ~PARENB;     		/* no parity bit */
	tty.c_cflag &= ~CSTOPB;     		/* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    		/* no hardware flowcontrol */

	/* setup for non-canonical mode */
	cfmakeraw(&tty);

	//tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 0; 
	tty.c_cc[VTIME] = 5;

	if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
		std::cerr << "Error from tcsetattr: " << strerror(errno) << ".\n";
		return false;
	}

	return true;
}

bool Serial::setBlocking(bool blocking) {
	return fcntl(m_fd, F_SETFL, blocking ? 0 : FNDELAY) != -1;
}

void Serial::close() {
	_s::_close(m_fd);
}

Serial::~Serial() {
	close();
}
	
