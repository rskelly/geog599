#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>    // read/write usleep
#include <linux/i2c-dev.h> // I2C bus definitions
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <termios.h>

#include <vector>
#include <string>
#include <iostream>

#include "serial.hpp"


int util::_open(const char* path, int flags) {
	return open(path, flags);
}

void util::_close(int fd) {
	close(fd);
}

int util::_read(int fd, char* buf, int len) {
	return read(fd, buf, len);
}

int util::_write(int fd, char* buf, int len) {
	return write(fd, buf, len);
}


Serial::Serial(const Properties& props) :
	m_fd(0),
	m_props(props) {
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
	switch(m_props.type) {
	case USB:
		return openUSB();
	case I2C:
		return openI2C();
	default:
		return false;
	}
}

bool Serial::openUSB() {

	std::cout << "Opening USB\n";

	const USBProperties& props = static_cast<USBProperties&>(m_props);

	if((m_fd = util::_open(props.dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC)) < 0) {
		std::cerr << "Error opening USB device at: " << props.dev << " (" << strerror(errno) << ")\n";
		return false;
	}

	/*
	struct termios tty;
	memset (&tty, 0, sizeof tty);

	if (tcgetattr (m_fd, &tty) != 0) {
			std::cerr << "Error from tcgetattr: " << strerror(errno) << "\n";
			return false;
	}

	cfsetospeed (&tty, props.speed);
	cfsetispeed (&tty, props.speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read blocks
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);		// ignore modem controls, enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= props.parity;			// set parity
	tty.c_cflag &= ~CSTOPB;					// single stop bit
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (m_fd, TCSANOW, &tty) != 0) {
			std::cerr << "Error from tcsetattr: " << strerror(errno) << "\n";
			return false;
	}
	*/

    struct termios tty;

    if (tcgetattr(m_fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t) props.speed);
    cfsetispeed(&tty, (speed_t) props.speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return false;
    }

	return true;
}

bool Serial::openI2C() {

	const I2CProperties& props = static_cast<I2CProperties&>(m_props);

	std::cout << "Opening I2C device at " << props.dev << ".\n";
	if((m_fd = util::_open(props.dev.c_str(), O_RDWR)) < 0) {
		std::cerr << "Failed to open device at " << props.dev << ".\n";
		return false;
	}
	std::cerr << "Connecting to I2C device at " << props.addr << " (" << m_fd << ").\n";
	if(ioctl(m_fd, I2C_SLAVE, props.addr) < 0) {
		perror("Failed to configure device");
		util::_close(m_fd);
		return false;
	}
	return true;
}

void Serial::close() {
	util::_close(m_fd);
}

Serial::~Serial() {
	close();
}
	
