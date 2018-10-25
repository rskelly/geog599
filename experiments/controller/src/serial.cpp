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

#include "serial.hpp"

using namespace comm;

namespace util {

	int _open(const char* path, int flags) {
		return open(path, flags);
	}

	void _close(int fd) {
		close(fd);
	}

	int _read(int fd, char* buf, int len) {
		return read(fd, buf, len);
	}

	int _write(int fd, char* buf, int len) {
		return write(fd, buf, len);
	}

}


Serial::Serial(const std::string& dev, int speed) :
	m_dev(dev),
	m_fd(0),
	m_speed(speed) {
}

bool Serial::open(const std::string& dev) {
	m_dev = dev;
	return open();
}

int Serial::write(char* buf, int len) {
	return util::_write(m_fd, buf, len);
}

int Serial::read(char* buf, int len) {
	return util::_read(m_fd, buf, len);
}

int Serial::read(std::vector<char>& buf) {
	int len = buf.size();
	std::cout << "reading " << len << "\n";
	int red = util::_read(m_fd, buf.data(), len);
	std::cout << "read " << red << " of " << len << "\n";
	return red;
}

int Serial::write(std::vector<char>& buf, int len) {
	if(len == -1)
		len = (int) buf.size();
	return util::_write(m_fd, buf.data(), len);
}

bool Serial::open() {
	if(m_dev.empty())
		return false;

	if((m_fd = util::_open(m_dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		std::cerr << "Error opening serial device at: " << m_dev << " (" << strerror(errno) << ")\n";
		return false;
	}
	
	struct termios tty;
	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(m_fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
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
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5;

	if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
		std::cerr << "Error from tcsetattr: " << strerror(errno) << ".\n";
		return false;
	}

	return true;
}

bool Serial::setBlocking(bool blocking) {
	return fcntl(m_fd, F_SETFL, blocking ? FNDELAY : 0) != -1;
}

void Serial::close() {
	util::_close(m_fd);
}

Serial::~Serial() {
	close();
}
	
