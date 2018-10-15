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

/**
 * An class for accessing Serial (I2C, USB, etc.) devices.
 */
class Serial {
protected:
	std::string m_dev;	///<! The device path; something like "/dev/ttyi2c-1"
	long m_addr;		///<! The address of the device if needed.
	int m_fd;			///<! The file handle of the device.

	/**
	 * A protected wrapper for write, for internal use.
	 */
	int write(char* buf, int len);

	/**
	 * A protected wrapper for read, for internal use.
	 */
	int read(char* buf, int len);

public:

	/**
	 * Configure a serial device endpoint at the given path with the given address.
	 *
	 * @param dev The device path; something like "/dev/i2c-1".
	 * @param addr The device address.
	 */
	Serial(const std::string& dev, long addr = -1) :
		m_dev(dev),
		m_addr(addr),
		m_fd(0) {
	}

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	virtual bool open() = 0;

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
	virtual void close() = 0;

	virtual ~Serial() {
		close();
	}

};

class I2C : public Serial {
public:

	/**
	 * Configure an I2C device endpoint at the given path with the given address.
	 *
	 * @param dev The device path; something like "/dev/i2c-1".
	 * @param addr The device address.
	 */
	I2C(const std::string& dev, long addr = -1) :
		m_dev(dev),
		m_addr(addr),
		m_fd(0) {
	}

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	virtual bool open() = 0;

	/**
	 * Disconnect from the device.
	 */
	virtual void close() = 0;

};

#endif /* INCLUDE_SERIAL_HPP_ */
