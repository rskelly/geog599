/*
 * ads1115.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_ADS1115_HPP_
#define INCLUDE_ADS1115_HPP_

#include <string>

#include "serial.hpp"

/**
 * A class for interoperating with a Texas Instruments ADS1115 analog-to-digital
 * convertor over I2C.
 *
 * Datasheet: http://www.ti.com/lit/ds/symlink/ads1115.pdf
 */
class ADS1115 : public Serial {
private:
	uint16_t m_config;	///<! The contents of the configuration register.

public:

	/**
	 * Configure the device at the given path. The addresses can be configured
	 * by connecting the ADDR pin to one of the other pins:
	 *
	 * GND  -->  1001000 (0x48)
	 * VDD  -->  1001001 (0x49)
	 * SDA  -->  1001010 (0x50)
	 * SCL  -->  1001011 (0x51)
	 *
	 * @param props An I2CProperties object configured for this device.
	 */
	ADS1115(const Properties& props);

	/**
	 * Save the contents of the config variable to the configuration register.
	 *
	 * @return True if the save succeeds.
	 */
	bool saveConfig();

	/**
	 * Read the contents of the config variable from the configuration register.
	 *
	 * @return True if the load succeeds.
	 */
	bool loadConfig();

	/**
	 * Connect to the device. Loads the configuration.
	 *
	 * @return True if the connection and configuration load succeed.
	 */
	bool open();

	/**
	 * Initiate a conversion.
	 */
	void startConversion();

	/**
	 * Returns true if a conversion is in progress.
	 *
	 * @return True if a conversion is in progress.
	 */
	bool isConverting() const;

	/**
	 * Select the slot to convert. The slots are as follows, in binary.
	 *
	 * 0b100 (0x4)  -->  AIN0
	 * 0b101 (0x5)  -->  AIN1
	 * 0b110 (0x6)  -->  AIN2
	 * 0b111 (0x7)  -->  AIN3
	 *
	 * Each of these assumes that GND is used as the negative voltage reference.
	 * There are alternatives; see the datasheet.
	 *
	 * @param value The slot.
	 */
	void setSlot(int value);

	/**
	 * Return the current slot.
	 *
	 * @see ADS1115::setSlot()
	 *
	 * @return The current slot.
	 */
	int slot() const;

	/**
	 * Set the voltage gain.
	 *
	 * 0b000 (0x0)  -->  ±6.144V
	 * 0b001 (0x1)  -->  ±4.096V
	 * 0b010 (0x2)  -->  ±2.048V (default)
	 * 0b011 (0x3)  -->  ±1.024V
	 * 0b100 (0x4)  -->  ±0.512V
	 * 0b101 (0x5)  -->  ±0.256V
	 * 0b110 (0x6)  -->  ±0.256V
	 * 0b111 (0x7)  -->  ±0.256V
	 *
	 * @param value The voltage gain.
	 */
	void setGain(int value);

	/**
	 * Return the voltage gain.
	 *
	 * @see ADS1115::setGain(int)
	 *
	 * @return The voltage gain.
	 */
	int gain() const;

	/**
	 * Set the device for continuous reading.
	 */
	void setContinuous();

	/**
	 * Return true if the device is set for continuous reading.
	 *
	 * @return True if the device is set for continuous reading.
	 */
	bool isContinuous() const;

	/**
	 * Set the device for one-shot reading.
	 */
	void setOneShot();

	/**
	 * Return true if the device is set for one-shot reading.
	 *
	 * @return True if the device is set for one-shot reading.
	 */
	bool isOneShot() const;

	/**
	 * Set the data rate.
	 *
	 * 0b000 (0x0)  -->  8 SPS
	 * 0b001 (0x1)  -->  16 SPS
	 * 0b010 (0x2)  -->  32 SPS
	 * 0b011 (0x3)  -->  64 SPS
	 * 0b100 (0x4)  -->  128 SPS (default)
	 * 0b101 (0x5)  -->  250 SPS
	 * 0b110 (0x6)  -->  475 SPS
	 * 0b111 (0x7)  -->  860 SPS
	 *
	 * @param rate The data rate setting.
	 */
	void setDataRate(int rate);

	/**
	 * Get the current data rate setting.
	 *
	 * @see ADS1115::setDataRate(int)
	 *
	 * @return The data rate setting.
	 */
	int dataRate() const;

	/**
	 * Get the current data rate as the number of signals per second.
	 *
	 * @see ADS1115::dataRate
	 *
	 * @return The current data rate as the number of signals per second.
	 */
	int dataRateS() const;

	/**
	 * Read the value for the given slot into the given integer reference.
	 *
	 * If the slot is different from that used on the previous read, will
	 * reconfigure automatically, which implies a small wait.
	 *
	 * @param slot The slot to read from.
	 * @param value A reference to write the converted value to.
	 * @return True if the read succeeds.
	 */
	bool readValue(int slot, int& value);

};



#endif /* INCLUDE_ADS1115_HPP_ */
