/*
 * lwsf30c.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <chrono>

#include "sensor/teensy.hpp"

using namespace sensor;


Teensy::Teensy(const std::string& dev, int speed) :
	Serial(dev, speed) {
}

Teensy::Teensy() :
	Serial() {
}

bool Teensy::open() {
	return Serial::open();
}

bool Teensy::open(const std::string& dev, int speed) {
	return Serial::open(dev, speed);
}

inline unsigned long __readULong(uint8_t* buf, size_t start) {
	unsigned long u = 0;
	for(int i = 0; i < 8; ++i)
		u |= (buf[start + i] << ((7 - i) * 8));
	return u;
}

inline short __readShort(uint8_t* buf, size_t start) {
	unsigned long s = 0;
	for(int i = 0; i < 2; ++i)
		s |= (buf[start + i] << ((1 - i) * 8));
	return s;
}

// Read short low byte first
inline short __readShortR(uint8_t* buf, size_t start) {
	unsigned long s = 0;
	for(int i = 0; i < 2; ++i)
		s |= (buf[start + i] << (i * 8));
	return s;
}

bool Teensy::readData(int& range, int& rangeTime, float& angle, int& angleTime, int* gyro, int* acc) {
	uint8_t buf[32];
	int r;
	int need = 1;
	int mode = 0;
	while(true) {
		while(available() < need);
		if((r = read((char*) buf, need)) < 0) {
			if(errno != EAGAIN)	// the the error is not "try again" print error
				std::cerr << "Read error: " << strerror(errno) << "\n";
			continue;
		}
		if(mode == 0) {
			if(buf[0] != '#') {
				mode = 0;
				need = 1;
			} else {
				mode = 1;
				need = 32;
			}
		} else if(mode == 1) {
			rangeTime = __readULong(buf, 0);
			range = __readShort(buf, 8);
			angleTime = __readULong(buf, 10);
			angle = __readShort(buf, 18);
			gyro[0] = __readShortR(buf, 20);
			gyro[1] = __readShortR(buf, 22);
			gyro[2] = __readShortR(buf, 24);
			acc[0] = __readShortR(buf, 26);
			acc[1] = __readShortR(buf, 28);
			acc[2] = __readShortR(buf, 30);
			mode = 0;
			need = 1;
			break;
		}
	}
	return true;
}

