/*
 * i2c.hpp
 *
 *  Created on: Oct 24, 2018
 *      Author: rob
 */

#ifndef INCLUDE_I2C_HPP_
#define INCLUDE_I2C_HPP_

#include <string>
#include <vector>

namespace serial {

/**
 * https://www.mjmwired.net/kernel/Documentation/i2c/dev-interface
 */
class I2C {
protected:
	int m_fd;						///<! File handle.
	std::string m_dev;				///<! Device path.
	int m_addr;						///<! Device address.

	bool readByte();

	bool writeByte(uint8_t value);

	bool readByteData(uint8_t cmd, uint8_t& value);

	bool writeByteData(uint8_t cmd, uint8_t value);

	bool readWordData(uint8_t cmd, uint16_t& value);

	bool writeWordData(uint8_t cmd, uint16_t value);

	bool readBlockData(uint8_t cmd, std::vector<uint8_t>& data, int& len);

	bool writeBlockData(uint8_t cmd, const std::vector<uint8_t>& data, int len);

	bool readBlockData(uint8_t cmd, uint8_t* data, int& len);

	bool writeBlockData(uint8_t cmd, const uint8_t* data, int len);


public:

	/**
	 * Construct an unconfigured serial. Must be configured before use.
	 */
	I2C();

	/**
	 * Configure a serial device endpoint at the given path with the given address.
	 *
	 * @param props An appropriate Properties subclass with values appropriate to the type of conneciton desired.
	 */
	I2C(const std::string& dev, uint8_t addr);

	/**
	 * Configure and open the device using the given properties object.
	 *
	 * @param props The properties object.
	 * @return True if the device was successfully opened.
	 */
	bool open(const std::string& dev, uint8_t addr);

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	bool open();

	/**
	 * Disconnect from the device.
	 */
	void close();

	~I2C();

};

} // serial

#endif /* INCLUDE_I2C_HPP_ */
