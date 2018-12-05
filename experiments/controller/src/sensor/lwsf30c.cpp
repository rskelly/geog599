/*
 * lwsf30c.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <chrono>

#include "sensor/lwsf30c.hpp"

using namespace sensor;


LWSF30C::LWSF30C(const std::string& dev, int speed) :
	Serial(dev, speed) {
}

LWSF30C::LWSF30C() :
	Serial() {
}

bool LWSF30C::open() {
	return Serial::open();
}

bool LWSF30C::open(const std::string& dev, int speed) {
	return Serial::open(dev, speed);
}

bool LWSF30C::sendCommand(char mnemonic, int value) {
	char buf[32];
	if(value > -1) {
		sprintf(buf, "#%c%d:\r", mnemonic, value);
	} else {
		sprintf(buf, "#%c\r", mnemonic);
	}
	int len = strlen(buf);
	int r;
	if((r = write(buf, len)) < 0) 
	       std::cerr << strerror(errno) << "\n";
	usleep(len * 100);
	if((r = read(buf, 2)) < 0) {
		std::cerr << strerror(errno) << "\n";
	} else {
		buf[len] = '\0';
		std::cout << buf << "\n";
	}
	return r > -1;
}

int LWSF30C::range() {
	while(available() < 2);		// wait for available bytes
	char buf[2];
	int r;
	if((r = read(buf, 2)) < 0) {	// read bytes into the buffer
		if(errno != EAGAIN)	// the the error is not "try again" print error
			std::cerr << "Read error: " << strerror(errno) << "\n";
		return -1;
	} else if(r == 2) {
		if(buf[1] & 0x80) { 				// check for flag bit in second element
			buf[0] = buf[1];			// shift byte
			while(!available());			// wait for a byte
			if((r = read(&buf[1], 1)) < 0) {	// read again
				if(errno != EAGAIN)
					std::cerr << "Read error: " << strerror(errno) << "\n";
				return -1;
			}
		}
		// The first bit in the high byte is a flag; remove it.
		return ((buf[0] & 0x7f) << 7) | (buf[1] & 0x7f);
	} else {
		return 0;
	}
}

bool LWSF30C::setResolution(int value) {
	return sendCommand('R', value);
}

bool LWSF30C::setSerialRate(int rate) {
	return sendCommand('U', rate);
}

bool LWSF30C::startLaser() {
	return sendCommand('Y');
}

bool LWSF30C::stopLaser() {
	return sendCommand('N');
}

bool LWSF30C::setDistance() {
	return sendCommand('p', 0);
}

bool LWSF30C::setSpeed() {
	return sendCommand('p', 1);
}

/*
int main(int argc, char** argv) {

	LWSF30C l("/dev/tty1");

	if(l.open()) {
		std::cout << "Connected to laser\n";
	} else {
		std::cerr << "Failed to connect to laser\n";
	}


	if(l.stopLaser()) {
		std::cout << "Stopped laser.\n";
	} else {
		std::cerr << "Failed to stop laser\n";
	}

	if(l.startLaser()) {
		std::cout << "Started laser.\n";
	} else {
		std::cerr << "Failed to start laser\n";
		return 0;
	}

	std::vector<char> buf(32);
	while(true) {
		usleep(32 * 100);
		int read = l.read(buf);
		for(int i = 0; i < read; ++i)
			std::cout << buf[i] << "|";
		std::cout << "\n";
	}

}

*/
