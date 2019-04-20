/*
 * i2c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SERIAL_HPP_
#define INCLUDE_SERIAL_HPP_

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

#include <string>
#include <vector>

namespace comm {

/**
 * An class for accessing serial devices.
 */
class Serial {
protected:
	std::string m_dev;	///<! Path to device.
	int m_fd;			///<! File handle.
	int m_speed;		///<! The serial baud rate.

public:

	/**
	 * Configure a serial device endpoint at the given device path.
	 *
	 * @param dev The device path.
	 * @param speed The baud rate.
	 */
	Serial(const std::string& dev, int speed = B115200);

	/**
	 * Create an unconfigured device.
	 */
	Serial();

	/**
	 * Returns the number of bytes available for reading from the buffer.
	 *
	 * @eturn The number of bytes available for reading from the buffer.
	 */
	int available();

	/**
	 * Configure and open the device using the given device path.
	 *
	 * @param dev The device path.
	 * @param speed The baud rate.
	 * @return True if connection is successful.
	 */
	bool open(const std::string& dev, int speed = B115200);

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
	 * Set the read call to blocking or non-blocking.
	 *
	 * @param blocking True to set read calls to block.
	 * @return True if set successfully.
	 */
	bool setBlocking(bool blocking);

	/**
	 * Disconnect from the device.
	 */
	void close();

	~Serial();

};

} // comm

#endif /* INCLUDE_SERIAL_HPP_ */
