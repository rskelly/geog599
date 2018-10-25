/*
 * ads1115.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_MINIMU9V5_HPP_
#define INCLUDE_MINIMU9V5_HPP_

#include <string>

#include "i2c.hpp"

/**
 * Register addresses for the gyroscope chip.
 * LSM6DS33
 */
enum GyroReg {
	FUNC_CFG_ACCESS	= 0x01,

	// FIFO queue configuration.
	FIFO_CTRL1		= 0x06,
	FIFO_CTRL2		= 0x07,
	FIFO_CTRL3		= 0x08,
	FIFO_CTRL4		= 0x09,
	FIFO_CTRL5		= 0x0A,

	// Sign and orientation for pitch/roll/yaw.
	ORIENT_CFG_G	= 0x0B,

	// Pad control registers.
	INT1_CTRL		= 0x0D,
	INT2_CTRL		= 0x0E,

	// Reads the ID. Fixed at 0x69.
	G_WHO_AM_I		= 0x0F,

	// Accel and gyro control
	CTRL1_XL		= 0x10,			// Accel: data rate, power mode; full-scale selection; anti-aliasing filter bandwidth selection
	CTRL2_G			= 0x11,			// Gyro: data rate; full-scale selection; full-scale at 125dps
	CTRL3_C			= 0x12,			// Reboot memory; block data update; interrupt activation level; etc.
	CTRL4_C			= 0x13,			// Accel bandwidth; gyro sleep mode; interrupts; temperature in fifo; disable i2c; etc.
	CTRL5_C			= 0x14,			// Circular burst mode rounding; self tests
	CTRL6_C			= 0x15,			// Gyro: edge-sensitive trigger; data level trigger; level-sensitive latch; high-perf disable for accel
	CTRL7_G			= 0x16,			// Gyro: high-perf disable; high-pass filter enable; high-pass filter reset; rounding; high-pass cutoff
	CTRL8_XL		= 0x17,			// Accel: low-pass filter selection; slope filter, high-pass filter; slope filter; low-pass on 6D
	CTRL9_XL		= 0x18,			// Accel: axis output enable
	CTRL10_C		= 0x19,			// Gyro axis enable; embedded functions, etc.

	WAKE_UP_SRC        = 0x1B,
	TAP_SRC            = 0x1C,
	D6D_SRC            = 0x1D,
	G_STATUS_REG         = 0x1E,

	// Temperature registers (2 bytes)
	OUT_TEMP_L         = 0x20,
	OUT_TEMP_H         = 0x21,

	// Accel/gyro outputs
	OUTX_L_G           = 0x22,		// Gyro: x-axis (2 bytes)
	OUTX_H_G           = 0x23,
	OUTY_L_G           = 0x24,		// Gyro: y-axis (2 bytes)
	OUTY_H_G           = 0x25,
	OUTZ_L_G           = 0x26,		// Gyro: z-axis (2 bytes)
	OUTZ_H_G           = 0x27,
	OUTX_L_XL          = 0x28,		// Accel: x-axis (2 bytes)
	OUTX_H_XL          = 0x29,
	OUTY_L_XL          = 0x2A,		// Accel: y-axis (2 bytes)
	OUTY_H_XL          = 0x2B,
	OUTZ_L_XL          = 0x2C,		// Accel: z-axis (2 bytes)
	OUTZ_H_XL          = 0x2D,
	
	// FIFO stuff
	FIFO_STATUS1       = 0x3A,
	FIFO_STATUS2       = 0x3B,
	FIFO_STATUS3       = 0x3C,
	FIFO_STATUS4       = 0x3D,	
	FIFO_DATA_OUT_L	   = 0x3E,
	FIFO_DATA_OUT_H	   = 0x3F,
	
	// Timestamp (24-bit word)
	TIMESTAMP0_REG	   = 0x40,
	TIMESTAMP1_REG	   = 0x41,
	TIMESTAMP2_REG	   = 0x42,

	// Step counter and timestamp
	STEP_TIMESTAMP_L   = 0x49,
	STEP_TIMESTAMP_H   = 0x4A,
	STEP_COUNTER_L	   = 0x4B,
	STEP_COUNTER_H	   = 0x4C,

	// Detect significant motion
	FUNC_SRC           = 0x53,

	// Tap-related
	TAP_CFG	           = 0x58,
	TAP_THS_6D		   = 0x59,
	INT_DUR2           = 0x5A,
	WAKE_UP_THS        = 0x5B,
	WAKE_UP_DUR        = 0x5C,

	// Free fall duration.
	FREE_FALL          = 0x5D,

	// Etc.
	MD1_CFG            = 0x5E,
	MD2_CFG            = 0x5F,
};

/**
 * Register addresses for the magnetometer chip.
 * LIS3MDL
 */
enum MagReg {

	// Reads the device ID. Fixed at 0x3D
	M_WHO_AM_I 	= 0x0F,

	CTRL_REG1 	= 0x20,		// Temp sensor enable; x and y operative mode; output data rate; fast odr; self-test
	CTRL_REG2 	= 0x21,		// Full-scale config; reboot memory; config/user reset function
	CTRL_REG3 	= 0x22,		// Low-powe mode config; spi mode select; operating mode select (continuous, single, etc.)
	CTRL_REG4 	= 0x23,		// z-axis operative mode select; big/little endian select.
	CTRL_REG5 	= 0x24,		// Fast-read; block-data update (continuous vs. no update until MSB and LSB have been read)

	M_STATUS_REG 	= 0x27,

	OUT_X_L 	= 0x28,		// Output x-axis (2 bytes)
	OUT_X_H 	= 0x29,
	OUT_Y_L 	= 0x2A,		// Output y-axis (2 bytes)
	OUT_Y_H 	= 0x2B,
	OUT_Z_L 	= 0x2C,		// Output z-axis (2 bytes)
	OUT_Z_H 	= 0x2D,
	TEMP_OUT_L  = 0x2E,		// Temperature (2 bytes)
	TEMP_OUT_H 	= 0x2F,

	INT_CFG 	= 0x30,
	INT_SRC 	= 0x31,
	INT_THS_L 	= 0x32,
	INT_THS_H 	= 0x33,
};

/**
 * Stores information about the latest IMU update.
 */
class MinIMU9v5State {
public:
	uint16_t linearX;
	uint16_t linearY;
	uint16_t linearZ;
	uint16_t angularX;
	uint16_t angularY;
	uint16_t angularZ;
	uint32_t timestamp;
	bool linearUpdate;
	bool angularUpdate;
	bool timestampUpdate;

	/**
	 * Reset the update flags to false. If any update flag is true,
	 * the updated() method returns true.
	 */
	void reset() {
		linearUpdate = angularUpdate = timestampUpdate = false;
	}

	/**
	 * Returns true if the object has been updated.
	 *
	 * @return True if the object has been updated.
	 */
	bool updated() {
		return linearUpdate || angularUpdate || timestampUpdate;
	}

	/**
	 * Set the linear acceleration component using the 6-byte array.
	 *
	 * @param values A 6-byte array.
	 */
	void setLinear(uint8_t* values) {
		linearX = (values[0] << 8) | values[1];
		linearY = (values[2] << 8) | values[3];
		linearZ = (values[4] << 8) | values[5];
		linearUpdate = true;
	}

	/**
	 * Set the angular acceleration component using the 6-byte array.
	 *
	 * @param values A 6-byte array.
	 */
	void setAngular(uint8_t* values) {
		angularX = (values[0] << 8) | values[1];
		angularY = (values[2] << 8) | values[3];
		angularZ = (values[4] << 8) | values[5];
		angularUpdate = true;
	}

	/**
	 * Set the timestamp using the 3-byte array.
	 *
	 * @param values A 3-byte array.
	 */
	void setTimestamp(uint8_t* values) {
		timestamp = (values[0] << 16) | (values[1] << 8) | values[2];
		timestampUpdate = true;
	}

};

using namespace serial;

/**
 * A class for interoperating with a Pololu MinIMU9-v5 IMU via I2C.
 * The MinIMU9-v5 has a gyroscope/accelerometer/thermometer on one chip,
 * and a magnetometer/thermometer on the other chip, each accessible
 * via an I2C address.
 */
class MinIMU9v5 {
private:
	I2C m_gyro;
	I2C m_mag;

protected:

	/**
	 * Convenience method for configuring the gyroscope.
	 * 
	 * @param reg The register to configure.
	 * @param val The value.
	 * @return True on success.
	 */
	bool configGyro(uint8_t reg, uint8_t val);
	
	/**
	 * Convenience method for configuring the gyroscope.
	 * 
	 * @param reg The register to configure.
	 * @param val The value.
	 * @return True on success.
	 */
	bool configMag(uint8_t reg, uint8_t val);

public:

	/**
	 * @param dev The I2C device path.
	 * @param gyroAddr The address of the gyroscope.
	 * @param magAddr The address of the magnetometer.
	 */
	MinIMU9v5(const std::string& dev, uint8_t gyroAddr, uint8_t magAddr);

	MinIMU9v5();

	/**
	 * Connect to the device.
	 *
	 * @param dev The I2C device path.
	 * @param gyroAddr The address of the gyroscope.
	 * @param magAddr The address of the magnetometer.
	 *
	 * @return True if the connection succeeded.
	 */
	bool open(const std::string& dev, int gyroAddr, int magAddr);

	/**
	 * Connect to the device.
	 *
	 * @return True if the connection succeeded.
	 */
	bool open();

	/**
	 * Disconnect the device.
	 */
	void close();

	/**
	 * Returns true if the gyroscope/accelerometer is available.
	 *
	 * @return True if the gyroscope/accelerometer is available.
	 */
	bool hasGyro();

	/**
	 * Returns true if the magnetometer is available.
	 *
	 * @return True if the magnetometer is available.
	 */
	bool hasMag();

	/**
	 * Collect the current state of the instruments.
	 *
	 * @param state An object containing the instrument state.
	 * @return True if at least some of the state was successfully collected.
	 */
	bool getState(MinIMU9v5State& state);

	~MinIMU9v5();

};



#endif /* INCLUDE_MINIMU9V5_HPP_ */
