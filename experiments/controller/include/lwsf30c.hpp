/*
 * lwsf30c.hpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#ifndef INCLUDE_LWSF30C_HPP_
#define INCLUDE_LWSF30C_HPP_

#include <string>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>

#include "serial.hpp"

#define LWSF30C_STREAM_CHANNEL_COUNT	5

/**
 * Provides interactivity with the LightWare Optoelectronics SF30/C
 * laser rangefinder over USB or I2C.
 */
class LWSF30C : public Serial {
private:

public:

	enum Command {
		LWC_NONE,
		LWC_NO_RESPONSE,

		LWC_STREAM_CLEAR,

		LWC_STREAM_1,
		LWC_STREAM_2,
		LWC_STREAM_3,
		LWC_STREAM_4,
		LWC_STREAM_5,

		LWC_SAVE_ALL,

		LWC_PRODUCT,

		LWC_LASER_DISTANCE_FIRST,
		LWC_LASER_DISTANCE_LAST,
		LWC_LASER_SIGNAL_STRENGTH_FIRST,
		LWC_LASER_SIGNAL_STRENGTH_LAST,
		LWC_LASER_OFFSET,
		LWC_LASER_ALARM_A_DISTANCE,
		LWC_LASER_ALARM_B_DISTANCE,
		LWC_LASER_ALARM_HYSTERESIS,
		LWC_LASER_MODE,
		LWC_LASER_FIRING,
		LWC_LASER_TEMPERATURE,
		LWC_LASER_BACKGROUND_NOISE,
		LWC_LASER_ENCODING_PATTERN,
		LWC_LASER_LOST_CONFIRMATIONS,
		LWC_LASER_GAIN_BOOST,

		LWC_SERVO_CONNECTED,
		LWC_SERVO_SCANNING,
		LWC_SERVO_SCAN,
		LWC_SERVO_POSITION,
		LWC_SERVO_PWM_MIN,
		LWC_SERVO_PWM_MAX,
		LWC_SERVO_PWM_SCALE,
		LWC_SERVO_TYPE,
		LWC_SERVO_STEPS,
		LWC_SERVO_LAG,
		LWC_SERVO_FOV_LOW,
		LWC_SERVO_FOV_HIGH,
		LWC_SERVO_ALARM_A_LOW,
		LWC_SERVO_ALARM_A_HIGH,
		LWC_SERVO_ALARM_B_LOW,
		LWC_SERVO_ALARM_B_HIGH,

		LWC_ALARM_STATE_BOTH,
		LWC_ALARM_STATE_A,
		LWC_ALARM_STATE_B,

		LWC_COMS_BAUD_RATE,
		LWC_COMS_I2C_ADDRESS,

		LWC_ENERGY_POWER_CONSUMPTION,
	};

	enum PulseType {
		LWPT_FIRST,
		LWPT_LAST,
	};

	enum ReturnFilter {
		LWRF_MEDIAN		= 0,
		LWRF_RAW 		= 1,
		LWRF_CLOSEST	= 2,
		LWRF_FURTHEST	= 3,
	};

	enum BaudRate {
		LWBR_9600		= 0,
		LWBR_19200		= 1,
		LWBR_38400		= 2,
		LWBR_57600		= 3,
		LWBR_115200		= 4,
		LWBR_230400		= 5,
		LWBR_460800		= 6,
		LWBR_921600		= 7,
	};

	enum ModeSpeed {
		LWMS_388		= 1,
		LWMS_194		= 2,
		LWMS_129		= 3,
		LWMS_97			= 4,
		LWMS_77			= 5,
		LWMS_64			= 6,
		LWMS_55			= 7,
		LWMS_48			= 8,
	};

	enum EncodingPattern {
		LWEP_NONE		= 0,
		LWEP_PATTERN_A	= 1,
		LWEP_PATTERN_B	= 2,
		LWEP_PATTERN_C	= 3,
		LWEP_PATTERN_D	= 4,
	};

	enum ScanType {
		LWST_BIDIRECTIONAL = 0,
		LWST_UNIDIRECTIONAL = 1,
	};

	enum EventLoopStatus {
		LWELR_SLEEP,
		LWELR_SEND_PACKET,
		LWELR_GET_PACKET,
		LWELR_FEEDBACK,
		LWELR_COMPLETED,
		LWELR_ERROR,
		LWELR_TIMEOUT,
	};

	enum SensorState {
		LWIS_SET_MMI,
		LWIS_WAIT_MMI,
		LWIS_STOP_STREAMING,
		LWIS_WAIT_STOP_STREAMING,
		LWIS_STOP_SCANNING,
		LWIS_WAIT_STOP_SCANNING,
		LWIS_GET_PRODUCT,
		LWIS_SENT_GET_PRODUCT,
		LWIS_WAIT_GET_PRODUCT,
		LWIS_INITED,

		LWIS_SENDING_COMMAND,
		LWIS_WAITING_FOR_RESPONSE,
	};

	struct EventLoopResult {
		EventLoopStatus		status;
		int32_t				timeMS;
	};

	enum ResolvePacketStatus {
		LWRPS_AGAIN,
		LWRPS_ERROR,
		LWRPS_COMPLETE,
	};

	struct AlarmState {
		bool 				alarmA;
		bool 				alarmB;
	};

	struct ProductInfo {
		char				model[8];
		float				firmwareVersion;
		float 				softwareVersion;
	};

	struct ScanSample {
		float				angle;
		float				firstPulse;
		float				lastPulse;
	};

	struct CmdPacket {
		Command				type;
		uint8_t				buffer[32];
		int32_t				length;
	};

	struct ResponsePacket
	{
		CmdPacket			data;
		Command 			type;
		bool				streaming;
		int32_t				filterType;

		union
		{
			int32_t			intValue;
			float			floatValue;
			ProductInfo 	product;
			ScanSample		scanSample;
			AlarmState		alarmState;
		};
	};

	struct LWSFC30
	{
		SensorState			state;
		ProductInfo 		product;
		CmdPacket 			command;
		ResponsePacket 		response;
		Command				streamCommands[LWSF30C_STREAM_CHANNEL_COUNT];
		void*				userData;
	};

	struct lwParser
	{
		uint8_t*			packetBuf;
		int 				packetLen;
		int 				packetIdx;
		int 				nextChar;
		int 				lexemeStart;
		int 				lexemeLength;
	};

	struct lwResolvePacketResult
	{
		ResolvePacketStatus status;
		int32_t bytesRead;
	};


	/**
	 * Connect to the device at the given path, with the given address.
	 *
	 * @param dev The device path; something like "/dev/ttyUSB0" or "/dev/i2c-1".
	 * @param addr The device address. Optional with USB.
	 */
	LWSF30C(const std::string& dev, int addr = -1) :
		m_dev(dev),
		m_addr(addr) {
	}

	/**
	 * Connect to the device.
	 *
	 * @return True if connection is successful.
	 */
	bool open() {
		std::cout << "Opening USB device at " << m_dev << ".\n";
		if((m_fd = util::_open(m_dev.c_str(), O_RDWR|O_SYNC)) < 0) {
			std::cerr << "Failed to open device at " << m_dev << ".\n";
			return false;
		}
		/*
		std::cerr << "Connecting to I2C device at " << m_addr << " (" << m_fd << ").\n";
		if(ioctl(m_fd, I2C_SLAVE, m_addr) < 0) {
			perror("Failed to configure device");
			util::_close(m_fd);
			return false;
		}
		*/
		return true;
	}

	bool sendCommand(char mnemonic, int value = -1) {
		char buf[32];
		if(value > -1) {
			sprintf(buf, "#%c%d:", mnemonic, value);
		} else {
			sprintf(buf, "#%c:", mnemonic);
		}
		int len = strlen(buf);
		return util::_write(m_fd, buf, len) == len;
	}


	/**
	 * Set the resolution; one of:
	 *
	 * 0 = 0.25 m
	 * 1 = 0.12 m
	 * 2 = 0.06 m
	 * 3 = 0.03 m
	 * 4 = Smoothed
	 */
	bool setResolution(int value) {
		return sendCommand('R', value);
	}

	bool setSerialRate(int rate) {
		return sendCommand('U', rate);
	}

	bool startLaser() {
		return sendCommand('Y');
	}

	bool stopLaser() {
		return sendCommand('N');
	}

	bool setDistance() {
		return sendCommand('p', 0);
	}

	bool setSpeed() {
		return sendCommand('p', 1);
	}

	/**
	 * Disconnect from the device.
	 */
	void close() {
		util::_close(m_fd);
	}

};


#endif /* INCLUDE_LWSF30C_HPP_ */
