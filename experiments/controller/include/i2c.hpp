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
	uint8_t m_addr;						///<! Device address.
	std::string m_dev;				///<! Device path.

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

	// These methods wrap the smbus i2c methods.

	/**
	 * Read and return a byte from the device.
	 *
	 * @param value The value to be updated.
	 * @return True if successful.
	 */
	bool readByte(uint8_t& value);

	/**
	 * Write the byte to the device.
	 *
	 * @param value The byte to write.
	 * @return True if successful.
	 */
	bool writeByte(uint8_t value);

	/**
	 * Read and return a byte from the device. Will write the
	 * register value first, then read the result.
	 *
	 * @param cmd The register value to read.
	 * @param value The value to be updated.
	 * @return True if successful.
	 */
	bool readByteData(uint8_t cmd, uint8_t& value);

	/**
	 * Write the byte to the device.
	 *
	 * @param cmd The register to write to.
	 * @param value The byte to write.
	 * @return True if successful.
	 */
	bool writeByteData(uint8_t cmd, uint8_t value);

	/**
	 * Read and return a word from the device. Will write the
	 * register value first, then read the result.
	 *
	 * @param cmd The register value to read.
	 * @param value The value to be updated.
	 * @return True if successful.
	 */
	bool readWordData(uint8_t cmd, uint16_t& value);

	/**
	 * Write the word to the device.
	 *
	 * @param cmd The register to write to.
	 * @param value The word to write.
	 * @return True if successful.
	 */
	bool writeWordData(uint8_t cmd, uint16_t value);

	/**
	 * Read and return available bytes from the device. Will write the
	 * register value first, then read the result.
	 *
	 * @param cmd The register value to read.
	 * @param value The value to be updated.
	 * @param len The length of data read.
	 * @return True if successful.
	 */
	bool readBlockData(uint8_t cmd, std::vector<uint8_t>& data, uint8_t& len);

	/**
	 * Write the bytes to the device.
	 *
	 * @param cmd The register to write to.
	 * @param value The bytes to write.
	 * @param len The number of bytes to write.
	 * @return True if successful.
	 */
	bool writeBlockData(uint8_t cmd, const std::vector<uint8_t>& data, uint8_t len);

	/**
	 * Read and return available bytes from the device. Will write the
	 * register value first, then read the result.
	 *
	 * @param cmd The register value to read.
	 * @param value The value to be updated.
	 * @param len The length of data read.
	 * @return True if successful.
	 */
	bool readBlockData(uint8_t cmd, uint8_t* data, uint8_t& len);

	/**
	 * Write the bytes to the device.
	 *
	 * @param cmd The register to write to.
	 * @param value The bytes to write.
	 * @param len The number of bytes to write.
	 * @return True if successful.
	 */
	bool writeBlockData(uint8_t cmd, const uint8_t* data, uint8_t len);

	~I2C();

};

} // serial

#endif /* INCLUDE_I2C_HPP_ */
