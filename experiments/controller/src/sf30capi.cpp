
//-------------------------------------------------------------------------
// LightWare SF30C Linux Example
// Stolen from https://raw.githubusercontent.com/LightWare-Optoelectronics/SF30C-Api/master/Linux/main.cpp
//-------------------------------------------------------------------------


#include <iostream>

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>

#define SF30C_API_IMPLEMENTATION

#include "sf30capi.hpp"


struct lwSerialPort
{
	int fd;
	bool connected;
};

struct lwSensorContext
{
	lwSF30C			sf30c;
	lwSerialPort 	serialPort;
	uint8_t			inputBuffer[128];
	int32_t			inputBufferSize;
};

//-------------------------------------------------------------------------
// Platform Specific Functions.
//-------------------------------------------------------------------------
inline int64_t PlatformGetMicrosecond()
{
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);

	return time.tv_sec * 1000000 + time.tv_nsec / 1000;
}

inline int32_t PlatformGetMS()
{
	return (PlatformGetMicrosecond() / 1000);
}

//-------------------------------------------------------------------------
// Com Port Implementation.
//-------------------------------------------------------------------------
bool serialDisconnect(lwSerialPort* ComPort)
{
	if (ComPort != 0 && ComPort->fd >= 0)
	{
		close(ComPort->fd);
	}

	ComPort->fd = -1;
	ComPort->connected = false;

	return true;
}

bool serialConnect(lwSerialPort* ComPort, const char* Name, int BitRate)
{
	int fd = -1;
	printf("Attempt com connection: %s\n", Name);

	fd = open(Name, O_RDWR | O_NOCTTY | O_SYNC);

	if (fd < 0)
	{
		printf("Couldn't open serial port!\n");
		return false;
	}

	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0)
	{
		printf("Error from tcgetattr\n");
		return false;
	}

	cfsetospeed(&tty, BitRate);
	cfsetispeed(&tty, BitRate);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag |= 0;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_iflag &= ~IGNBRK;
	tty.c_iflag &= ~ICRNL;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("Error from tcsetattr\n");
		return false;
	}

	printf("Connected\n");
	ComPort->fd = fd;
	ComPort->connected = true;

	return true;
}

int serialWrite(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize)
{
	if (!ComPort)
	{
		printf("Can't write to null coms\n");
		return -1;
	}

	if (!ComPort->connected)
	{
		printf("Can't write to non connected coms\n");
		return -1;
	}

	int writtenBytes = write(ComPort->fd, Buffer, BufferSize);

	if (writtenBytes != BufferSize)
	{
		printf("Could not send all bytes!\n");
		return -1;
	}

	return writtenBytes;
}

int32_t serialRead(lwSerialPort* ComPort, char *Buffer, int32_t BufferSize)
{
	if (!ComPort)
	{
		printf("Can't read from null coms\n");
		return -1;
	}

	if (!ComPort->connected)
	{
		printf("Can't read from non connected coms\n");
		return -1;
	}

	errno = 0;
	int readBytes = read(ComPort->fd, Buffer, BufferSize);

	//if (readBytes == 0)
		//printf("No Data: %d Error %d (%s)\n", readBytes, errno, strerror(errno));

	return readBytes;
}

//-------------------------------------------------------------------------

bool sendPacket(lwSF30C* sf30c, lwCmdPacket* Packet)
{
	//std::cout << "Send Packet " << Packet->length << "\n";
	lwSensorContext* context = (lwSensorContext*)sf30c->userData;
	if (serialWrite(&context->serialPort, (char*)Packet->buffer, Packet->length) != -1)
		return true;

	return false;
}

bool getPacket(lwSF30C* sf30c, lwResponsePacket* Packet)
{
	lwSensorContext* context = (lwSensorContext*)sf30c->userData;

	while (true)
	{
		if (context->inputBufferSize == 0)
		{
			int bytesRead = 0;
			if ((bytesRead = serialRead(&context->serialPort, (char*)context->inputBuffer, sizeof(context->inputBuffer))) != -1)
				context->inputBufferSize = bytesRead;
			else
				return false;
		}

		lwResolvePacketResult packetResolve = sf30cResolvePacket(&context->sf30c.response, context->inputBuffer, context->inputBufferSize);

		// NOTE: You can use a circular buffer or so to avoid the shuffle.
		if (packetResolve.bytesRead > 0)
		{
			int32_t remaining = context->inputBufferSize - packetResolve.bytesRead;
			for (int i = 0; i < remaining; ++i)
				context->inputBuffer[i] = context->inputBuffer[packetResolve.bytesRead + i];

			context->inputBufferSize = remaining;
		}

		if (packetResolve.status == LWRPS_COMPLETE)
		{
			return true;
		}
	}

	return true;
}

bool sleep(lwSF30C* sf30c, int32_t TimeMS)
{
	usleep(TimeMS * 1000);
	return true;
};

bool streamResponse(lwSF30C* sf30c, lwResponsePacket* Packet)
{
	if (Packet->type == LWC_SERVO_SCAN)
	{
		std::cout << "Scan: " << Packet->scanSample.angle << " " << Packet->scanSample.firstPulse << " " << Packet->scanSample.lastPulse << "\n";
	}
	else if (Packet->type == LWC_SERVO_POSITION)
	{
		std::cout << "Pos: " << Packet->floatValue << "\n";
	}
	else if (Packet->type == LWC_LASER_TEMPERATURE)
	{
		std::cout << "Temp: " << Packet->floatValue << "\n";
	}

	return true;
};

//-------------------------------------------------------------------------
// Application Entry.
//-------------------------------------------------------------------------
int main(int args, char **argv)
{
	std::cout << "SF30C Api\n";

	lwSensorContext context = {};
	context.sf30c = createSF30C();
	context.sf30c.userData = &context;
	serialConnect(&context.serialPort, "/dev/ttyUSB0", B115200);

	lwServiceContext serviceContext = {};
	serviceContext.sendPacketCallback = sendPacket;
	serviceContext.getPacketCallback = getPacket;
	serviceContext.sleepCallback = sleep;
	serviceContext.streamCallback = streamResponse;

	// NOTE: Run event loop for first time init.
	runEventLoop(&context.sf30c, &serviceContext);

	lwProductInfo productInfo = executeCmd_GetProduct(&context.sf30c, &serviceContext);
	std::cout << "Product: " << context.sf30c.response.product.model << "\n";

	executeCmd_SetLaserMode(&context.sf30c, &serviceContext, LWMS_48);

	while (true)
	{
		float distance = executeCmd_GetLaserDistanceFirst(&context.sf30c, &serviceContext);
		std::cout << "Distance: " << distance << "\n";
	}

	serialDisconnect(&context.serialPort);

	std::cout << "Press Enter to Exit...\n";
	std::cin.ignore();
	return 0;
}
