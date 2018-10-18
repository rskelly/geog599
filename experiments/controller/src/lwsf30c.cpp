/*
 * lwsf30c.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include "lwsf30c.hpp"

int main(int argc, char** argv) {

	USBProperties props("/dev/ttyUSB0");
	LWSF30C l(props);

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


