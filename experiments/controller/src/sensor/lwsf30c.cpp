/*
 * lwsf30c.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include "sensor/lwsf30c.hpp"

using namespace sensor;


LWSF30C::LWSF30C(const std::string& dev, int speed) :
	Serial(dev, speed) {
}

bool LWSF30C::sendCommand(char mnemonic, int value) {
	char buf[32];
	if(value > -1) {
		sprintf(buf, "#%c%d:", mnemonic, value);
	} else {
		sprintf(buf, "#%c:", mnemonic);
	}
	int len = strlen(buf);
	int wrt = write(buf, len);
	usleep(len * 100);
	return wrt > -1;
}

double LWSF30C::getMeasurement() {
	char buf[2];
	if(read(buf, 2) == 2)
		return (buf[0] << 8) + buf[1] / 250.0;
	return false;
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


int main(int argc, char** argv) {

	LWSF30C l("/dev/tty1");

	if(l.open()) {
		std::cout << "Connected to laser\n";
	} else {
		std::cerr << "Failed to connect to laser\n";
	}

	/*
	if(l.stopLaser()) {
		std::cout << "Stopped laser.\n";
	} else {
		std::cerr << "Failed to stop laser\n";
	}
	*/
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


