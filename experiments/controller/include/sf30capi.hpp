//-------------------------------------------------------------------------
// LightWare SF30C API V0.7.2
// Written by: Robert Gowans, rob@lightware.co.za
//-------------------------------------------------------------------------

// Features & Characteristics:
//-------------------------------------------------------------------------
// - Support for Arduino, RaspberryPI, Windows & Linux. (8bit, 32bit, 64bit)
// - Single header file library.
// - Zero external dependencies.
// - Zero dynamic memory allocations.
// - Minimal C++ style.

// How to use single file header only libraries:
//-------------------------------------------------------------------------
// You can include this file as normal when you would include any other
// header file. However, you need to define the implementation in one
// C or C++ compilation unit file. You can do this with:
//
// #define SF30C_API_IMPLEMENTATION
// #include "sf30capi.h"
//-------------------------------------------------------------------------

#ifndef LIGHTWARE_INCLUDE_SF30CAPI_H
#define LIGHTWARE_INCLUDE_SF30CAPI_H

#define SF30C_STREAM_CHANNEL_COUNT	5

#include <cstdint>

enum lwCommand
{
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

enum lwPulseType
{
	LWPT_FIRST,
	LWPT_LAST,
};

enum lwReturnFilter
{
	LWRF_MEDIAN		= 0,
	LWRF_RAW 		= 1,
	LWRF_CLOSEST	= 2,
	LWRF_FURTHEST	= 3,
};

enum lwBaudRate
{
	LWBR_9600		= 0,
	LWBR_19200		= 1,
	LWBR_38400		= 2,
	LWBR_57600		= 3,
	LWBR_115200		= 4,
	LWBR_230400		= 5,
	LWBR_460800		= 6,
	LWBR_921600		= 7,
};

enum lwModeSpeed
{
	LWMS_388		= 1,
	LWMS_194		= 2,
	LWMS_129		= 3,
	LWMS_97			= 4,
	LWMS_77			= 5,
	LWMS_64			= 6,
	LWMS_55			= 7,
	LWMS_48			= 8,
};

enum lwEncodingPattern
{
	LWEP_NONE		= 0,
	LWEP_PATTERN_A	= 1,
	LWEP_PATTERN_B	= 2,
	LWEP_PATTERN_C	= 3,
	LWEP_PATTERN_D	= 4,
};

enum lwScanType
{
	LWST_BIDIRECTIONAL = 0,
	LWST_UNIDIRECTIONAL = 1,
};

enum lwEventLoopStatus
{
	LWELR_SLEEP,
	LWELR_SEND_PACKET,
	LWELR_GET_PACKET,
	LWELR_FEEDBACK,
	LWELR_COMPLETED,
	LWELR_ERROR,
	LWELR_TIMEOUT,
};

enum lwSensorState
{
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

struct lwEventLoopResult
{
	lwEventLoopStatus	status;
	int32_t				timeMS;
};

enum lwResolvePacketStatus
{
	LWRPS_AGAIN,
	LWRPS_ERROR,
	LWRPS_COMPLETE,
};

struct lwAlarmState
{
	bool 				alarmA;
	bool 				alarmB;
};

struct lwProductInfo
{
	char				model[8];
	float				firmwareVersion;
	float 				softwareVersion;
};

struct lwScanSample
{
	float				angle;
	float				firstPulse;
	float				lastPulse;
};

struct lwCmdPacket
{
	lwCommand			type;
	uint8_t				buffer[32];
	int32_t				length;
};

struct lwResponsePacket
{
	lwCmdPacket			data;
	lwCommand 			type;
	bool				streaming;
	int32_t				filterType;

	union
	{
		int32_t			intValue;
		float			floatValue;
		lwProductInfo 	product;
		lwScanSample	scanSample;
		lwAlarmState	alarmState;
	};
};

struct lwSF30C
{
	lwSensorState		state;
	lwProductInfo 		product;
	lwCmdPacket 		command;
	lwResponsePacket 	response;
	lwCommand			streamCommands[SF30C_STREAM_CHANNEL_COUNT];
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
	lwResolvePacketStatus status;
	int32_t bytesRead;
};

typedef bool(*sf30cSendPacketCallback)(lwSF30C* sf30c, lwCmdPacket* Packet);
typedef bool(*sf30cGetPacketCallback)(lwSF30C* sf30c, lwResponsePacket* Packet);
typedef bool(*sf30cSleepCallback)(lwSF30C* sf30c, int32_t TimeMS);
typedef bool(*sf30cStreamCallback)(lwSF30C* sf30c, lwResponsePacket* Packet);

struct lwServiceContext
{
	sf30cSendPacketCallback	sendPacketCallback;
	sf30cGetPacketCallback	getPacketCallback;
	sf30cSleepCallback		sleepCallback;
	sf30cStreamCallback		streamCallback;
};

int32_t sf30cBaudRateToInt(lwBaudRate BaudRate);
int32_t sf30cModeSpeedToInt(lwModeSpeed ModeSpeed);
const char* sf30cScanTypeToStr(lwScanType ScanType);

#endif

#ifdef SF30C_API_IMPLEMENTATION

//-------------------------------------------------------------------------
// Helper functions.
//-------------------------------------------------------------------------
int32_t sf30cBaudRateToInt(lwBaudRate BaudRate)
{
	int32_t baudTable[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
	return baudTable[BaudRate];
}

int32_t sf30cModeSpeedToInt(lwModeSpeed ModeSpeed)
{
	int32_t speedTable[] = { 0, 388, 194, 129, 97, 77, 64, 55, 48 };
	return speedTable[ModeSpeed];
}

const char* sf30cScanTypeToStr(lwScanType ScanType)
{
	const char* scanTable[] = { "Bidirectional", "Unidirectional" };
	return scanTable[ScanType];
}

//-------------------------------------------------------------------------
// Parsing.
//-------------------------------------------------------------------------
bool isCharNumber(int C)
{
	return ((C >= '0' && C <= '9') || C == '-' || C == '.');
}

bool isCharIdentifier(int C)
{
	return ((C >= 'a' && C <= 'z') || (C >= 'A' && C <= 'Z') || C == '$' || C == '%' || isCharNumber(C));
}

void getNextChar(lwParser* Parser)
{
	if (Parser->packetIdx < Parser->packetLen - 1)
		Parser->nextChar = Parser->packetBuf[Parser->packetIdx++];
	else
		Parser->nextChar = -1;
}

bool checkIdentSingle(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]);
}

bool checkIdentDouble(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]) &&
		((Parser->packetBuf + Parser->lexemeStart)[1] == B[1]);
}

bool checkIdentTriple(lwParser* Parser, const char* B)
{
	return ((Parser->packetBuf + Parser->lexemeStart)[0] == B[0]) &&
		((Parser->packetBuf + Parser->lexemeStart)[1] == B[1]) &&
		((Parser->packetBuf + Parser->lexemeStart)[2] == B[2]);
}

bool expectPacketDelimeter(lwParser* Parser)
{
	if (Parser->nextChar == ':')
	{
		getNextChar(Parser);
		return true;
	}

	return false;
}

bool expectParamDelimeter(lwParser* Parser)
{
	if (Parser->nextChar == ',')
	{
		getNextChar(Parser);
		return true;
	}

	return false;
}

bool expectCharLiterals(lwParser* Parser, const char* Chars)
{
	while (*Chars != 0)
	{
		if (Parser->nextChar == *Chars++)
			getNextChar(Parser);
		else
			return false;
	}

	return true;
}

bool expectIdentifier(lwParser* Parser)
{
	if (isCharIdentifier(Parser->nextChar))
	{
		Parser->lexemeStart = Parser->packetIdx - 1;
		Parser->lexemeLength = 1;
		getNextChar(Parser);
		while (isCharIdentifier(Parser->nextChar))
		{
			++Parser->lexemeLength;
			getNextChar(Parser);
		}

		return true;
	}

	return false;
}

int32_t expectIdentifier(lwParser* Parser, char* Buffer, int32_t BufferSize)
{
	int32_t identSize = 0;
	if ((identSize < BufferSize) && isCharIdentifier(Parser->nextChar))
	{
		++identSize;
		(Buffer++)[0] = Parser->nextChar;
		Parser->lexemeStart = Parser->packetIdx - 1;
		Parser->lexemeLength = 1;
		getNextChar(Parser);
		while ((identSize < BufferSize) && (isCharIdentifier(Parser->nextChar) || isCharNumber(Parser->nextChar)))
		{
			++identSize;
			(Buffer++)[0] = Parser->nextChar;
			++Parser->lexemeLength;
			getNextChar(Parser);
		}

		Buffer[0] = 0;
		return identSize;
	}

	Buffer[0] = 0;
	return 0;
}

bool expectNumber(lwParser* Parser, float* Number)
{
	if (isCharNumber(Parser->nextChar))
	{
		Parser->lexemeStart = Parser->packetIdx - 1;

		int part = 0;
		bool neg = false;

		while (Parser->nextChar != -1 && (Parser->nextChar < '0' || Parser->nextChar > '9') && Parser->nextChar != '-' && Parser->nextChar != '.')
			getNextChar(Parser);

		if (Parser->nextChar == '-')
		{
			neg = true;
			getNextChar(Parser);
		}

		while (Parser->nextChar != -1 && !(Parser->nextChar > '9' || Parser->nextChar < '0'))
		{
			part = part * 10 + (Parser->nextChar - '0');
			getNextChar(Parser);
		}

		*Number = neg ? (float)(part * -1) : (float)part;

		if (Parser->nextChar == '.')
		{
			getNextChar(Parser);

			double mul = 1;
			part = 0;

			while (Parser->nextChar != -1 && !(Parser->nextChar > '9' || Parser->nextChar < '0'))
			{
				part = part * 10 + (Parser->nextChar - '0');
				mul *= 10;
				getNextChar(Parser);
			}

			if (neg)
				*Number -= (float)part / (float)mul;
			else
				*Number += (float)part / (float)mul;
		}

		Parser->lexemeLength = Parser->packetIdx - 1 - Parser->lexemeStart;

		return true;
	}

	return false;
}

bool expectNumber(lwParser* Parser, int32_t* Number)
{
	float temp;
	bool result = expectNumber(Parser, &temp);
	*Number = (int32_t)temp;
	return result;
}

bool parseResponseInt(lwParser* Parser, const char* ResponseString, int32_t* ResponseData)
{
	float value = 0;

	if (!checkIdentSingle(Parser, ResponseString)) return false;
	if (!expectPacketDelimeter(Parser)) return false;
	if (!expectNumber(Parser, &value)) return false;

	if (ResponseData)
		*ResponseData = (int32_t)value;

	return true;
}

bool parseResponseFloat(lwParser* Parser, const char* ResponseString, float* ResponseData)
{
	float value = 0;

	if (!checkIdentSingle(Parser, ResponseString)) return false;
	if (!expectPacketDelimeter(Parser)) return false;
	if (!expectNumber(Parser, &value)) return false;

	if (ResponseData)
		*ResponseData = value;

	return true;
}

bool parseResponse(lwResponsePacket* Packet)
{
	lwParser parser = {};
	parser.packetBuf = Packet->data.buffer;
	parser.packetLen = Packet->data.length;
	parser.packetIdx = 0;

	getNextChar(&parser);

	char identifierBuffer[8];
	char* identBuf = identifierBuffer;

	int32_t identSize = expectIdentifier(&parser, identifierBuffer, sizeof(identifierBuffer));

	if (identSize == 0)
		return false;

	// Identify streaming packets vs stream channel responses, and clear streaming channel response.
	if (identBuf[0] == '$' && !(identBuf[1] >= '0' && identBuf[1] <= '9') && identSize != 1)
	{
		--identSize;
		identBuf = identifierBuffer + 1;
		Packet->streaming = true;
	}
	else
	{
		Packet->streaming = false;
	}

	if (identSize == 1)
	{
		if (identBuf[0] == '$')
		{
			Packet->type = LWC_STREAM_CLEAR;
			return true;
		}
		else if (identBuf[0] == 'p')
		{
			Packet->type = LWC_PRODUCT;

			if (!expectPacketDelimeter(&parser)) return false;
			if (!expectIdentifier(&parser, Packet->product.model, 8)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &Packet->product.softwareVersion)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &Packet->product.firmwareVersion)) return false;

			return true;
		}
		else if (identBuf[0] == 'a')
		{
			Packet->type = LWC_ALARM_STATE_BOTH;

			int32_t alarmA;
			int32_t alarmB;

			if (!expectPacketDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &alarmA)) return false;
			if (!expectParamDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &alarmB)) return false;

			Packet->alarmState.alarmA = (alarmA != 0);
			Packet->alarmState.alarmB = (alarmB != 0);

			return true;
		}
		else if (identBuf[0] == 'e')
		{
			Packet->type = LWC_ENERGY_POWER_CONSUMPTION;

			if (!expectPacketDelimeter(&parser)) return false;
			if (!expectNumber(&parser, &Packet->intValue)) return false;

			return true;
		}
	}
	else if (identSize == 2)
	{
		if (identBuf[0] == '$')
		{
			if (identBuf[1] == '1') Packet->type = LWC_STREAM_1;
			else if (identBuf[1] == '2') Packet->type = LWC_STREAM_2;
			else if (identBuf[1] == '3') Packet->type = LWC_STREAM_3;
			else if (identBuf[1] == '4') Packet->type = LWC_STREAM_4;
			else if (identBuf[1] == '5') Packet->type = LWC_STREAM_5;
			else return false;

			return true;
		}
		else if (identBuf[0] == 'l')
		{
			if (identBuf[1] == 'm')
			{
				Packet->type = LWC_LASER_MODE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 't')
			{
				Packet->type = LWC_LASER_TEMPERATURE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'o')
			{
				Packet->type = LWC_LASER_OFFSET;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'f')
			{
				Packet->type = LWC_LASER_FIRING;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'n')
			{
				Packet->type = LWC_LASER_BACKGROUND_NOISE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 't')
			{
				Packet->type = LWC_LASER_TEMPERATURE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'e')
			{
				Packet->type = LWC_LASER_ENCODING_PATTERN;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'c')
			{
				Packet->type = LWC_LASER_LOST_CONFIRMATIONS;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'b')
			{
				Packet->type = LWC_LASER_GAIN_BOOST;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
		}
		else if (identBuf[0] == 's')
		{
			if (identBuf[1] == 's')
			{
				if (Packet->streaming)
				{
					Packet->type = LWC_SERVO_SCAN;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.angle)) return false;
					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.firstPulse)) return false;
					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->scanSample.lastPulse)) return false;

					return true;
				}
				else
				{
					Packet->type = LWC_SERVO_SCANNING;
					return true;
				}
			}
			else if (identBuf[1] == 'c')
			{
				Packet->type = LWC_SERVO_CONNECTED;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'r')
			{
				Packet->type = LWC_SERVO_STEPS;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'l')
			{
				Packet->type = LWC_SERVO_LAG;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'p')
			{
				Packet->type = LWC_SERVO_POSITION;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->floatValue)) return false;

				return true;
			}
			else if (identBuf[1] == 't')
			{
				Packet->type = LWC_SERVO_TYPE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
		}
		else if (identBuf[0] == 'a')
		{
			if (identBuf[1] == 'a')
			{
				Packet->type = LWC_ALARM_STATE_A;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'b')
			{
				Packet->type = LWC_ALARM_STATE_B;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
		}
		else if (identBuf[0] == 'c')
		{
			if (identBuf[1] == 'b')
			{
				Packet->type = LWC_COMS_BAUD_RATE;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
			else if (identBuf[1] == 'i')
			{
				Packet->type = LWC_COMS_I2C_ADDRESS;

				if (!expectPacketDelimeter(&parser)) return false;
				if (!expectNumber(&parser, &Packet->intValue)) return false;

				return true;
			}
		}
	}
	else if (identSize == 3)
	{
		if (identBuf[0] == 'l')
		{
			if (identBuf[1] == 'a')
			{
				if (identBuf[2] == 'a' || identBuf[2] == 'b')
				{
					if (identBuf[2] == 'a') Packet->type = LWC_LASER_ALARM_A_DISTANCE;
					else if (identBuf[2] == 'b') Packet->type = LWC_LASER_ALARM_B_DISTANCE;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
				else if (identBuf[2] == 'h')
				{
					Packet->type = LWC_LASER_ALARM_HYSTERESIS;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'd')
			{
				if (identBuf[2] == 'f' || identBuf[2] == 'l')
				{
					if (identBuf[2] == 'f') Packet->type = LWC_LASER_DISTANCE_FIRST;
					else if (identBuf[2] == 'l') Packet->type = LWC_LASER_DISTANCE_LAST;

					if (!expectParamDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->filterType)) return false;
					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'h')
			{
				if (identBuf[2] == 'f' || identBuf[2] == 'l')
				{
					if (identBuf[2] == 'f') Packet->type = LWC_LASER_SIGNAL_STRENGTH_FIRST;
					else if (identBuf[2] == 'l') Packet->type = LWC_LASER_SIGNAL_STRENGTH_LAST;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
		}
		else if (identBuf[0] == 's')
		{
			if (identBuf[1] == 'w')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h' || identBuf[2] == 's')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_PWM_MIN;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_PWM_MAX;
					else if (identBuf[2] == 's') Packet->type = LWC_SERVO_PWM_SCALE;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->floatValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'f')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_FOV_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_FOV_HIGH;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'a')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_ALARM_A_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_ALARM_A_HIGH;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
			else if (identBuf[1] == 'b')
			{
				if (identBuf[2] == 'l' || identBuf[2] == 'h')
				{
					if (identBuf[2] == 'l') Packet->type = LWC_SERVO_ALARM_B_LOW;
					else if (identBuf[2] == 'h') Packet->type = LWC_SERVO_ALARM_B_HIGH;

					if (!expectPacketDelimeter(&parser)) return false;
					if (!expectNumber(&parser, &Packet->intValue)) return false;

					return true;
				}
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------------
// Packet Building.
//-------------------------------------------------------------------------
// NOTE: We always assume there is enough space in the buffer to write.

void packetWriteChar(lwCmdPacket* Packet, char Char)
{
	Packet->buffer[Packet->length++] = Char;
}

void packetWriteString(lwCmdPacket* Packet, const char* String)
{
	while (*String)
		Packet->buffer[Packet->length++] = String++[0];
}

void packetWriteDigits(lwCmdPacket* Packet, int32_t Number)
{
	int32_t startIndex = Packet->length;

	if (Number == 0)
	{
		Packet->buffer[Packet->length++] = '0';
	}
	else
	{
		while (Number > 0)
		{
			Packet->buffer[Packet->length++] = (Number % 10) + '0';
			Number /= 10;
		}

		int32_t length = Packet->length - startIndex;
		int32_t halfLength = length / 2;

		for (int32_t i = 0; i < halfLength; ++i)
		{
			uint8_t temp = Packet->buffer[startIndex + i];
			Packet->buffer[startIndex + i] = Packet->buffer[startIndex + (length - i) - 1];
			Packet->buffer[startIndex + (length - i) - 1] = temp;
		}
	}
}

void packetWriteFloat(lwCmdPacket* Packet, float Number)
{
	if (Number < 0)
	{
		packetWriteChar(Packet, '-');
		Number = -Number;
	}

	int32_t whole = (int32_t)Number;
	int32_t frac = (int32_t)((Number - (float)whole) * 100.0f);

	packetWriteDigits(Packet, whole);
	packetWriteChar(Packet, '.');
	packetWriteDigits(Packet, frac);
}

void packetWriteInt(lwCmdPacket* Packet, int32_t Number)
{
	if (Number < 0)
		packetWriteChar(Packet, '-');

	packetWriteDigits(Packet, Number * (Number < 0 ? -1 : 1));
}

void packetClear(lwCmdPacket* Packet)
{
	Packet->length = 0;
	Packet->type = LWC_NONE;
}

//-------------------------------------------------------------------------
// Command Packet Writers.
//-------------------------------------------------------------------------

void packetWrite_GetProduct(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?\r");
	Packet->type = LWC_PRODUCT;
}

void packetWrite_GetLaserDistanceFirst(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ldf\r");
	Packet->type = LWC_LASER_DISTANCE_FIRST;
}

void packetWrite_GetLaserDistanceLast(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ldl\r");
	Packet->type = LWC_LASER_DISTANCE_LAST;
}

void packetWrite_GetLaserDistance(lwCmdPacket* Packet, lwPulseType PulseType, lwReturnFilter Filter)
{
	packetClear(Packet);

	if (PulseType == LWPT_FIRST)
	{
		packetWriteString(Packet, "?ldf,");
		packetWriteInt(Packet, (int)Filter);
		packetWriteChar(Packet, '\r');
		Packet->type = LWC_LASER_DISTANCE_FIRST;
	}
	else if (PulseType == LWPT_LAST)
	{
		packetWriteString(Packet, "?ldl,");
		packetWriteInt(Packet, (int)Filter);
		packetWriteChar(Packet, '\r');
		Packet->type = LWC_LASER_DISTANCE_LAST;
	}
}

void packetWrite_GetLaserSignalStrengthFirst(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lhf\r");
	Packet->type = LWC_LASER_SIGNAL_STRENGTH_FIRST;
}

void packetWrite_GetLaserSignalStrengthLast(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lhl\r");
	Packet->type = LWC_LASER_SIGNAL_STRENGTH_LAST;
}

void packetWrite_GetLaserOffset(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lo\r");
	Packet->type = LWC_LASER_OFFSET;
}

void packetWrite_SetLaserOffset(lwCmdPacket* Packet, float Offset)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lo,");
	packetWriteFloat(Packet, Offset);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_OFFSET;
}

void packetWrite_GetLaserAlarmA(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?laa\r");
	Packet->type = LWC_LASER_ALARM_A_DISTANCE;
}

void packetWrite_SetLaserAlarmA(lwCmdPacket* Packet, float Distance)
{
	packetClear(Packet);
	packetWriteString(Packet, "#laa,");
	packetWriteFloat(Packet, Distance);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_ALARM_A_DISTANCE;
}

void packetWrite_GetLaserAlarmB(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lab\r");
	Packet->type = LWC_LASER_ALARM_B_DISTANCE;
}

void packetWrite_SetLaserAlarmB(lwCmdPacket* Packet, float Distance)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lab,");
	packetWriteFloat(Packet, Distance);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_ALARM_B_DISTANCE;
}

void packetWrite_GetLaserAlarmHysteresis(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lah\r");
	Packet->type = LWC_LASER_ALARM_HYSTERESIS;
}

void packetWrite_SetLaserAlarmHysteresis(lwCmdPacket* Packet, float Distance)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lah,");
	packetWriteFloat(Packet, Distance);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_ALARM_HYSTERESIS;
}

void packetWrite_GetLaserMode(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lm\r");
	Packet->type = LWC_LASER_MODE;
}

void packetWrite_SetLaserMode(lwCmdPacket* Packet, lwModeSpeed ModeSpeed)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lm,");
	packetWriteInt(Packet, (int)ModeSpeed);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_MODE;
}

void packetWrite_GetLaserFiring(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lf\r");
	Packet->type = LWC_LASER_FIRING;
}

void packetWrite_SetLaserFiring(lwCmdPacket* Packet, bool Firing)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lf,");
	packetWriteInt(Packet, Firing ? 1 : 0);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_FIRING;
}

void packetWrite_GetLaserNoise(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ln\r");
	Packet->type = LWC_LASER_BACKGROUND_NOISE;
}

void packetWrite_GetLaserTemperature(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lt\r");
	Packet->type = LWC_LASER_TEMPERATURE;
}

void packetWrite_GetLaserEncoding(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?le\r");
	Packet->type = LWC_LASER_ENCODING_PATTERN;
}

void packetWrite_SetLaserEncoding(lwCmdPacket* Packet, lwEncodingPattern Encoding)
{
	packetClear(Packet);
	packetWriteString(Packet, "#le,");
	packetWriteInt(Packet, (int)Encoding);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_ENCODING_PATTERN;
}

void packetWrite_GetLaserLostConfirmations(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lc\r");
	Packet->type = LWC_LASER_LOST_CONFIRMATIONS;
}

void packetWrite_SetLaserLostConfirmations(lwCmdPacket* Packet, int Confirmations)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lc,");
	packetWriteInt(Packet, Confirmations);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_LOST_CONFIRMATIONS;
}

void packetWrite_GetLaserGain(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?lb\r");
	Packet->type = LWC_LASER_GAIN_BOOST;
}

void packetWrite_SetLaserGain(lwCmdPacket* Packet, float Gain)
{
	packetClear(Packet);
	packetWriteString(Packet, "#lb,");
	packetWriteFloat(Packet, Gain);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_LASER_GAIN_BOOST;
}

void packetWrite_GetServoConnected(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sc\r");
	Packet->type = LWC_SERVO_CONNECTED;
}

void packetWrite_SetServoConnected(lwCmdPacket* Packet, bool Connected)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sc,");
	packetWriteInt(Packet, Connected ? 1 : 0);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_CONNECTED;
}

void packetWrite_GetServoScanning(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ss\r");
	Packet->type = LWC_SERVO_SCANNING;
}

void packetWrite_SetServoScanning(lwCmdPacket* Packet, bool Scanning)
{
	packetClear(Packet);
	packetWriteString(Packet, "#ss,");
	packetWriteInt(Packet, Scanning ? 1 : 0);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_SCANNING;
}

void packetWrite_GetServoPosition(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sp\r");
	Packet->type = LWC_SERVO_POSITION;
}

void packetWrite_SetServoPosition(lwCmdPacket* Packet, float Position)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sp,");
	packetWriteFloat(Packet, Position);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_POSITION;
}

void packetWrite_GetServoPwmMin(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?swl\r");
	Packet->type = LWC_SERVO_PWM_MIN;
}

void packetWrite_SetServoPwmMin(lwCmdPacket* Packet, float Time)
{
	packetClear(Packet);
	packetWriteString(Packet, "#swl,");
	packetWriteFloat(Packet, Time);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_PWM_MIN;
}

void packetWrite_GetServoPwmMax(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?swh\r");
	Packet->type = LWC_SERVO_PWM_MAX;
}

void packetWrite_SetServoPwmMax(lwCmdPacket* Packet, float Time)
{
	packetClear(Packet);
	packetWriteString(Packet, "#swh,");
	packetWriteFloat(Packet, Time);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_PWM_MAX;
}

void packetWrite_GetServoPwmScale(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sws\r");
	Packet->type = LWC_SERVO_PWM_SCALE;
}

void packetWrite_SetServoPwmScale(lwCmdPacket* Packet, float Scale)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sws,");
	packetWriteFloat(Packet, Scale);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_PWM_SCALE;
}

void packetWrite_GetServoScanType(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?st\r");
	Packet->type = LWC_SERVO_TYPE;
}

void packetWrite_SetServoScanType(lwCmdPacket* Packet, lwScanType Type)
{
	packetClear(Packet);
	packetWriteString(Packet, "#st,");
	packetWriteInt(Packet, (int)Type);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_TYPE;
}

void packetWrite_GetServoSteps(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sr\r");
	Packet->type = LWC_SERVO_STEPS;
}

void packetWrite_SetServoSteps(lwCmdPacket* Packet, int StepsPerReading)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sr,");
	packetWriteInt(Packet, StepsPerReading);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_STEPS;
}

void packetWrite_GetServoLag(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sl\r");
	Packet->type = LWC_SERVO_LAG;
}

void packetWrite_SetServoLag(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sl,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_LAG;
}

void packetWrite_GetServoFovLow(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sfl\r");
	Packet->type = LWC_SERVO_FOV_LOW;
}

void packetWrite_SetServoFovLow(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sfl,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_FOV_LOW;
}

void packetWrite_GetServoFovHigh(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sfh\r");
	Packet->type = LWC_SERVO_FOV_HIGH;
}

void packetWrite_SetServoFovHigh(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sfh,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_FOV_HIGH;
}

void packetWrite_GetServoAlarmALow(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sal\r");
	Packet->type = LWC_SERVO_ALARM_A_LOW;
}

void packetWrite_SetServoAlarmALow(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sal,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_ALARM_A_LOW;
}

void packetWrite_GetServoAlarmAHigh(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sah\r");
	Packet->type = LWC_SERVO_ALARM_A_HIGH;
}

void packetWrite_SetServoAlarmAHigh(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sah,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_ALARM_A_HIGH;
}

void packetWrite_GetServoAlarmBLow(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sbl\r");
	Packet->type = LWC_SERVO_ALARM_B_LOW;
}

void packetWrite_SetServoAlarmBLow(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sbl,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_ALARM_B_LOW;
}

void packetWrite_GetServoAlarmBHigh(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?sbh\r");
	Packet->type = LWC_SERVO_ALARM_B_HIGH;
}

void packetWrite_SetServoAlarmBHigh(lwCmdPacket* Packet, float Angle)
{
	packetClear(Packet);
	packetWriteString(Packet, "#sbh,");
	packetWriteFloat(Packet, Angle);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_SERVO_ALARM_B_HIGH;
}

void packetWrite_GetAlarmStateBoth(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?a\r");
	Packet->type = LWC_ALARM_STATE_BOTH;
}

void packetWrite_GetAlarmStateA(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?aa\r");
	Packet->type = LWC_ALARM_STATE_A;
}

void packetWrite_GetAlarmStateB(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ab\r");
	Packet->type = LWC_ALARM_STATE_B;
}

void packetWrite_GetComsBaudRate(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?cb\r");
	Packet->type = LWC_COMS_BAUD_RATE;
}

void packetWrite_SetComsBaudRate(lwCmdPacket* Packet, lwBaudRate BaudRate)
{
	packetClear(Packet);
	packetWriteString(Packet, "#cb,");
	packetWriteInt(Packet, (int)BaudRate);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_COMS_BAUD_RATE;
}

void packetWrite_GetComsAddress(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?ci\r");
	Packet->type = LWC_COMS_I2C_ADDRESS;
}

void packetWrite_SetComsAddress(lwCmdPacket* Packet, int Address)
{
	packetClear(Packet);
	packetWriteString(Packet, "#ci,");
	packetWriteInt(Packet, Address);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_COMS_I2C_ADDRESS;
}

void packetWrite_GetEnergyPower(lwCmdPacket* Packet)
{
	packetClear(Packet);
	packetWriteString(Packet, "?e\r");
	Packet->type = LWC_ENERGY_POWER_CONSUMPTION;
}

void packetWrite_SetEnergyPower(lwCmdPacket* Packet, bool Power)
{
	packetClear(Packet);
	packetWriteString(Packet, "#e,");
	packetWriteInt(Packet, Power ? 1 : 0);
	packetWriteChar(Packet, '\r');
	Packet->type = LWC_ENERGY_POWER_CONSUMPTION;
}

//-------------------------------------------------------------------------
// Primary Functionality.
//-------------------------------------------------------------------------

lwSF30C createSF30C()
{
	lwSF30C sf30c = {};
	return sf30c;
}

lwResolvePacketResult sf30cResolvePacket(lwResponsePacket* Packet, uint8_t* Buffer, int32_t BufferSize)
{
	lwResolvePacketResult result = {};

	if (Packet->type != LWC_NONE)
	{
		Packet->type = LWC_NONE;
		Packet->data.length = 0;
	}

	for (int i = 0; i < BufferSize; ++i)
	{
		result.bytesRead++;

		uint8_t c = Buffer[i];
		if (c == '\n')
		{
			Packet->data.length = 0;
		}
		else if (c == '\r')
		{
			if (Packet->data.length > 0)
			{
				if (parseResponse(Packet))
				{
					Packet->data.length = 0;
					result.status = LWRPS_COMPLETE;
				}
				else
				{
					Packet->data.length = 0;
					result.status = LWRPS_AGAIN;
				}

				break;
			}
		}
		else
		{
			if (Packet->data.length >= 32)
			{
				Packet->data.length = 0;
				result.status = LWRPS_AGAIN;
				break;
			}
			else
			{
				Packet->data.buffer[Packet->data.length++] = c;
			}
		}
	}

	return result;
}

lwEventLoopResult sf30cPumpEventLoop(lwSF30C* sf30c)
{
	lwEventLoopResult result = {};
	result.status = LWELR_COMPLETED;
	lwCmdPacket* packet = &sf30c->command;

	if (sf30c->state >= LWIS_INITED)
	{
		if (sf30c->state == LWIS_INITED)
		{
			if (sf30c->command.type != LWC_NONE)
			{
				sf30c->state = LWIS_SENDING_COMMAND;
				result.status = LWELR_SEND_PACKET;
			}
			else
			{
				if (sf30c->response.streaming)
				{
					sf30c->response.streaming = false;
					result.status = LWELR_FEEDBACK;
				}
				else
				{
					result.status = LWELR_COMPLETED;
				}
			}
		}
		else if (sf30c->state == LWIS_SENDING_COMMAND)
		{
			sf30c->state = LWIS_WAITING_FOR_RESPONSE;
			result.status = LWELR_GET_PACKET;
		}
		else if (sf30c->state == LWIS_WAITING_FOR_RESPONSE)
		{
			if (sf30c->response.type == sf30c->command.type)
			{
				sf30c->state = LWIS_INITED;
				sf30c->command.type = LWC_NONE;
				result.status = LWELR_COMPLETED;
			}
			else
			{
				result.status = LWELR_GET_PACKET;
			}
		}
	}
	else
	{
		if (sf30c->state == LWIS_SET_MMI)
		{
			sf30c->state = LWIS_WAIT_MMI;
			packetClear(packet);
			packetWriteChar(packet, '`');
			result.status = LWELR_SEND_PACKET;
		}
		else if (sf30c->state == LWIS_WAIT_MMI)
		{
			sf30c->state = LWIS_STOP_STREAMING;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (sf30c->state == LWIS_STOP_STREAMING)
		{
			sf30c->state = LWIS_WAIT_STOP_STREAMING;
			packetClear(packet);
			packetWriteString(packet, "$\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (sf30c->state == LWIS_WAIT_STOP_STREAMING)
		{
			sf30c->state = LWIS_STOP_SCANNING;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (sf30c->state == LWIS_STOP_SCANNING)
		{
			sf30c->state = LWIS_WAIT_STOP_SCANNING;
			packetClear(packet);
			packetWriteString(packet, "#SS,0\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (sf30c->state == LWIS_WAIT_STOP_SCANNING)
		{
			sf30c->state = LWIS_GET_PRODUCT;
			result.timeMS = 100;
			result.status = LWELR_SLEEP;
		}
		else if (sf30c->state == LWIS_GET_PRODUCT)
		{
			sf30c->state = LWIS_SENT_GET_PRODUCT;
			packetClear(packet);
			packetWriteString(packet, "?\r");
			result.status = LWELR_SEND_PACKET;
		}
		else if (sf30c->state == LWIS_SENT_GET_PRODUCT)
		{
			sf30c->state = LWIS_WAIT_GET_PRODUCT;
			result.status = LWELR_GET_PACKET;
		}
		else if (sf30c->state == LWIS_WAIT_GET_PRODUCT)
		{
			if (sf30c->response.type == LWC_PRODUCT)
			{
				sf30c->product = sf30c->response.product;
				sf30c->state = LWIS_INITED;
				sf30c->command.type = LWC_NONE;
				sf30c->response.type = LWC_NONE;
				result.status = LWELR_COMPLETED;
			}
			else
			{
				result.status = LWELR_GET_PACKET;
			}
		}
	}

	return result;
}

bool runEventLoop(lwSF30C* sf30c, lwServiceContext* Service, bool Streaming = false)
{
	while (true)
	{
		lwEventLoopResult result = sf30cPumpEventLoop(sf30c);

		switch (result.status)
		{
			case LWELR_SLEEP:
			{
				Service->sleepCallback(sf30c, result.timeMS);
			} break;

			case LWELR_SEND_PACKET:
			{
				if (!Service->sendPacketCallback(sf30c, &sf30c->command))
					return false;
			} break;

			case LWELR_GET_PACKET:
			{
				if (!Service->getPacketCallback(sf30c, &sf30c->response))
					return false;
			} break;

			case LWELR_ERROR:
			case LWELR_TIMEOUT:
			{
				return false;
			} break;

			case LWELR_FEEDBACK:
			{
				if (Streaming)
				{
					Service->streamCallback(sf30c, &sf30c->response);
					Service->getPacketCallback(sf30c, &sf30c->response);
				}
				else
				{
					return true;
				}

			} break;

			case LWELR_COMPLETED:
			{
				if (Streaming)
				{
					Service->getPacketCallback(sf30c, &sf30c->response);
				}
				else
				{
					return true;
				}

			} break;
		};
	}
}

void executeCommand(lwSF30C* sf30c, lwServiceContext* Service, const char* Command, lwCommand ResponseType)
{
	packetClear(&sf30c->command);
	packetWriteString(&sf30c->command, Command);
	sf30c->command.type = ResponseType;
	runEventLoop(sf30c, Service);
}

lwProductInfo executeCmd_GetProduct(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetProduct(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.product;
}

float executeCmd_GetLaserDistanceFirst(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserDistanceFirst(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

float executeCmd_GetLaserDistanceLast(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserDistanceLast(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

float executeCmd_GetLaserDistance(lwSF30C* sf30c, lwServiceContext* Service, lwPulseType PulseType, lwReturnFilter Filter)
{
	packetWrite_GetLaserDistance(&sf30c->command, PulseType, Filter);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

int executeCmd_GetLaserSignalStrengthFirst(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserSignalStrengthFirst(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.intValue;
}

int executeCmd_GetLaserSignalStrengthLast(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserSignalStrengthLast(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.intValue;
}

float executeCmd_GetLaserOffset(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserOffset(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetLaserOffset(lwSF30C* sf30c, lwServiceContext* Service, float Offset)
{
	packetWrite_SetLaserOffset(&sf30c->command, Offset);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetLaserAlarmA(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserAlarmA(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetLaserAlarmA(lwSF30C* sf30c, lwServiceContext* Service, float Distance)
{
	packetWrite_SetLaserAlarmA(&sf30c->command, Distance);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetLaserAlarmB(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserAlarmB(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetLaserAlarmB(lwSF30C* sf30c, lwServiceContext* Service, float Distance)
{
	packetWrite_SetLaserAlarmB(&sf30c->command, Distance);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetLaserAlarmHysteresis(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserAlarmHysteresis(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetLaserAlarmHysteresis(lwSF30C* sf30c, lwServiceContext* Service, float Distance)
{
	packetWrite_SetLaserAlarmHysteresis(&sf30c->command, Distance);
	runEventLoop(sf30c, Service);
}

lwModeSpeed executeCmd_GetLaserMode(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserMode(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (lwModeSpeed)sf30c->response.intValue;
}

void executeCmd_SetLaserMode(lwSF30C* sf30c, lwServiceContext* Service, lwModeSpeed Mode)
{
	packetWrite_SetLaserMode(&sf30c->command, Mode);
	runEventLoop(sf30c, Service);
}

bool executeCmd_GetLaserFiring(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserFiring(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue == 1);
}

void executeCmd_SetLaserFiring(lwSF30C* sf30c, lwServiceContext* Service, bool Firing)
{
	packetWrite_SetLaserFiring(&sf30c->command, Firing);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetLaserNoise(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserNoise(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

float executeCmd_GetLaserTemperature(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserTemperature(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

lwEncodingPattern executeCmd_GetLaserEncoding(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserEncoding(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (lwEncodingPattern)sf30c->response.intValue;
}

void executeCmd_SetLaserEncoding(lwSF30C* sf30c, lwServiceContext* Service, lwEncodingPattern Encoding)
{
	packetWrite_SetLaserEncoding(&sf30c->command, Encoding);
	runEventLoop(sf30c, Service);
}

int executeCmd_GetLaserLostConfirmations(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserLostConfirmations(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.intValue;
}

void executeCmd_SetLaserLostConfirmations(lwSF30C* sf30c, lwServiceContext* Service, int Confirmations)
{
	packetWrite_SetLaserLostConfirmations(&sf30c->command, Confirmations);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetLaserGain(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetLaserGain(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetLaserGain(lwSF30C* sf30c, lwServiceContext* Service, float Gain)
{
	packetWrite_SetLaserGain(&sf30c->command, Gain);
	runEventLoop(sf30c, Service);
}

bool executeCmd_GetServoConnected(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoConnected(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue == 1);
}

void executeCmd_SetServoConnected(lwSF30C* sf30c, lwServiceContext* Service, bool Connected)
{
	packetWrite_SetServoConnected(&sf30c->command, Connected);
	runEventLoop(sf30c, Service);
}

bool executeCmd_GetServoScanning(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoScanning(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue == 1);
}

void executeCmd_SetServoScanning(lwSF30C* sf30c, lwServiceContext* Service, bool Scanning)
{
	packetWrite_SetServoScanning(&sf30c->command, Scanning);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoPosition(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoPosition(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoPosition(lwSF30C* sf30c, lwServiceContext* Service, float Position)
{
	packetWrite_SetServoPosition(&sf30c->command, Position);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoPwmMin(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoPwmMin(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoPwmMin(lwSF30C* sf30c, lwServiceContext* Service, float Time)
{
	packetWrite_SetServoPwmMin(&sf30c->command, Time);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoPwmMax(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoPwmMax(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoPwmMax(lwSF30C* sf30c, lwServiceContext* Service, float Time)
{
	packetWrite_SetServoPwmMax(&sf30c->command, Time);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoPwmScale(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoPwmScale(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoPwmScale(lwSF30C* sf30c, lwServiceContext* Service, float Scale)
{
	packetWrite_SetServoPwmScale(&sf30c->command, Scale);
	runEventLoop(sf30c, Service);
}

lwScanType executeCmd_GetServoScanType(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoScanType(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (lwScanType)sf30c->response.intValue;
}

void executeCmd_SetServoScanType(lwSF30C* sf30c, lwServiceContext* Service, lwScanType Type)
{
	packetWrite_SetServoScanType(&sf30c->command, Type);
	runEventLoop(sf30c, Service);
}

int executeCmd_GetServoSteps(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoSteps(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.intValue;
}

void executeCmd_SetServoSteps(lwSF30C* sf30c, lwServiceContext* Service, int StepsPerReading)
{
	packetWrite_SetServoSteps(&sf30c->command, StepsPerReading);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoLag(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoLag(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoLag(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoLag(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoFovLow(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoFovLow(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoFovLow(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoFovLow(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoFovHigh(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoFovHigh(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoFovHigh(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoFovHigh(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoAlarmALow(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoAlarmALow(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoAlarmALow(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoAlarmALow(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoAlarmAHigh(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoAlarmAHigh(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoAlarmAHigh(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoAlarmAHigh(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoAlarmBLow(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoAlarmBLow(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoAlarmBLow(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoAlarmBLow(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

float executeCmd_GetServoAlarmBHigh(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetServoAlarmBHigh(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.floatValue;
}

void executeCmd_SetServoAlarmBHigh(lwSF30C* sf30c, lwServiceContext* Service, float Angle)
{
	packetWrite_SetServoAlarmBHigh(&sf30c->command, Angle);
	runEventLoop(sf30c, Service);
}

lwAlarmState executeCmd_GetAlarmStateBoth(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetAlarmStateBoth(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.alarmState;
}

bool executeCmd_GetAlarmStateA(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetAlarmStateA(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue != 0);
}

bool executeCmd_GetAlarmStateB(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetAlarmStateB(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue != 0);
}

lwBaudRate executeCmd_GetComsBaudRate(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetComsBaudRate(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (lwBaudRate)sf30c->response.intValue;
}

void executeCmd_SetComsBaudRate(lwSF30C* sf30c, lwServiceContext* Service, lwBaudRate BaudRate)
{
	packetWrite_SetComsBaudRate(&sf30c->command, BaudRate);
	runEventLoop(sf30c, Service);
}

int executeCmd_GetComsAddress(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetComsAddress(&sf30c->command);
	runEventLoop(sf30c, Service);
	return sf30c->response.intValue;
}

void executeCmd_SetComsAddress(lwSF30C* sf30c, lwServiceContext* Service, int Address)
{
	packetWrite_SetComsAddress(&sf30c->command, Address);
	runEventLoop(sf30c, Service);
}

bool executeCmd_GetEneryPower(lwSF30C* sf30c, lwServiceContext* Service)
{
	packetWrite_GetEnergyPower(&sf30c->command);
	runEventLoop(sf30c, Service);
	return (sf30c->response.intValue != 0);
}

void executeCmd_SetEnergyPower(lwSF30C* sf30c, lwServiceContext* Service, bool Power)
{
	packetWrite_SetEnergyPower(&sf30c->command, Power);
	runEventLoop(sf30c, Service);
}

#endif
