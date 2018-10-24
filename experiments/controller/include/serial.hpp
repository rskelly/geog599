/*
 * i2c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SERIAL_HPP_
#define INCLUDE_SERIAL_HPP_

#include <string>
#include <vector>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


namespace util {

	int _open(const char* path, int flags);

	void _close(int fd);

	int _read(int fd, char* buf, int len);

	int _write(int fd, char* buf, int len);

}

enum ConnectionType {
	I2C,
	USB,
};

class Properties {
public:
	ConnectionType type;
	std::string dev;

	Properties(const std::string& dev, ConnectionType type) :
		type(type), dev(dev) {}
};

class USBProperties : public Properties {
public:
	int parity;
	int speed;

	USBProperties(const std::string& dev, int parity = 0, int speed = B115200) :
		Properties(dev, USB),
		parity(parity), speed(speed) {}
};

class I2CProperties : public Properties {
public:
	int addr;

	I2CProperties(const std::string& dev, int addr) :
		Properties(dev, I2C),
		addr(addr) {}
};

/**
 * An class for accessing Serial (I2C, USB, etc.) devices.
 */
class Serial {
protected:
	int m_fd;			///<! File handle.
	const Properties* m_props;	///<! The Properties object that gives parameters for the connection.

	bool openUSB();

	bool openI2C();

public:

	/**
	 * Construct an unconfigured serial. Must be configured before use.
	 */
	Serial();

	/**
	 * Configure a serial device endpoint at the given path with the given address.
	 *
	 * @param props An appropriate Properties subclass with values appropriate to the type of conneciton desired.
	 */
	Serial(const Properties& props);

	/**
	 * Configure the device using the given properties object.
	 */
	void configure(const Properties* props);

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	bool open();

	/**
	 * A wrapper for write, for internal use.
	 */
	int write(char* buf, int len);

	/**
	 * A wrapper for read, for internal use.
	 */
	int read(char* buf, int len);

	/**
	 * Read from the device into the given buffer. The size
	 * of the read is taken from the buffer's reserved size,
	 * so an unreserved vector will read nothing.
	 *
	 * @param buf A char vector with a non-zero size.
	 * @return The number of bytes read.
	 */
	int read(std::vector<char>& buf);

	/**
	 * Write to the device from the given buffer. If
	 * len is -1, the size of the write is taken from the
	 * buffer's reserved size, so an unreserved vector will
	 * write nothing.
	 *
	 * @param buf A char vector with a non-zero size.
	 * @param len The length to write, or -1 to use the buffer's length.
	 * @return The number of bytes written.
	 */
	int write(std::vector<char>& buf, int len = -1);

	/**
	 * Disconnect from the device.
	 */
	void close();

	~Serial();

};

#endif /* INCLUDE_SERIAL_HPP_ */
