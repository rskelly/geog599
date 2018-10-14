/*
 * i2c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_I2C_HPP_
#define INCLUDE_I2C_HPP_

#include <string>
#include <vector>

/**
 * An class for accessing I2C devices.
 */
class I2C {
protected:
	std::string m_dev;	///<! The device path; something like "/dev/ttyi2c-1"
	long m_addr;		///<! The address of the I2C device.
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
	 * Configure an I2C device endpoint at the given path with the given address.
	 *
	 * @param dev The device path; something like "/dev/i2c-1".
	 * @param addr The device address.
	 */
	I2C(const std::string& dev, long addr) :
		m_dev(dev),
		m_addr(addr),
		m_fd(0) {
	}

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	bool open();

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

};


#endif /* INCLUDE_I2C_HPP_ */
