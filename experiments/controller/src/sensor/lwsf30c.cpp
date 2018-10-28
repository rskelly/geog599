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

uint64_t _micros() {
	return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void _bits(std::ostream& out, char* b, int len) {
	for(int j = 0; j < len; ++j) {
		for(int i = 7; i >= 0; --i)
			out << ((b[j] >> i) & 0x1);
		out << " - ";
	}
}	

double LWSF30C::range() {
	static char buf[2];
	static int r;
	while(available() < 2);		// wait for available bytes
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
		uint8_t h = (uint8_t) (buf[0] & 0x7f); // first bit is a flag
		uint8_t l = (uint8_t) (buf[1] & 0xff);
		//std::cout << "bits (bytes 0 and 1): "; _bits(std::cout, buf, 2);
		//std::cout << "; values (bytes 0 and 1):  " << (int) h << ", " << (int) l << "\n";
		return 0.01 * ((h << 7) | l); 
	} else {
		return 0;
	}
	/*
	char buf[3];
	while(available()) read(buf, 1);
	while(available() == 0);
	uint64_t t0 = _micros();
	while(available() == 1);
	uint64_t t1 = _micros();
	while(available() == 2);
	uint64_t t2 = _micros();
	int r = read(buf, 3);
	if(r < 0) {
		if(errno != EAGAIN)
			std::cerr << "Read error: " << strerror(errno) << "\n";
		return -1;
	} else {
		if(t1 - t0 > t2 - t1) {
			std::cout << "a: ";
			_bits(std::cout, buf, 3);
			std::cout << "\n";
			return buf[2] + buf[1] / 256.0;
		} else {
			std::cout << "b: ";
			_bits(std::cout, buf, 3);
			std::cout << "\n";
			return buf[1] + buf[0] / 256.0;
		}
	}
	return 0;
	*/
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
