/*
 * lwsf30c.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <chrono>
#include <thread>

#include "sensor/teensy.hpp"

using namespace sensor;

constexpr size_t BUFFER_SIZE = 10000;
constexpr size_t PACKET_SIZE = 13;
constexpr size_t HEADER_SIZE = 2;

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
	for(size_t i = 0; i < 8; ++i)
		u |= (buf[(start + i) % BUFFER_SIZE] << ((7 - i) * 8));
	return u;
}

inline unsigned short __readUShort(uint8_t* buf, size_t start) {
	unsigned short s = 0;
	for(size_t i = 0; i < 2; ++i)
		s |= (buf[(start + i) % BUFFER_SIZE] << ((1 - i) * 8));
	return s;
}

// Read short low byte first
inline short __readShortR(uint8_t* buf, size_t start) {
	short s = 0;
	for(size_t i = 0; i < 2; ++i)
		s |= (buf[(start + i) % BUFFER_SIZE] << (i * 8));
	return s;
}


bool Teensy::readData(Range& range, Orientation& orientation) {
	static uint8_t buf[BUFFER_SIZE];
	static uint8_t readBuf[BUFFER_SIZE];
	static size_t bufIdx = 0;
	static size_t bufEnd = 0;
	static int mode = 0;
	static size_t need = 1;

	range.setStatus(1);

	while(true) {

		while((bufEnd - bufIdx) < need) {
			std::this_thread::yield();
			int rd = read((char *) readBuf, BUFFER_SIZE);
			if(rd > 0) {
				for(int i = 0; i < rd; ++i) {
					buf[bufEnd % BUFFER_SIZE] = readBuf[i];
					++bufEnd;
				}
			}
		}
		
		if(mode == 0) {
			while(bufIdx < bufEnd) {
				if(buf[bufIdx % BUFFER_SIZE] == '#') {
					mode = 1;
					need = 1;
					++bufIdx;
					break;
				}
				++bufIdx;
			}
			continue;
		}

		if(mode == 1) {
			need = buf[bufIdx % BUFFER_SIZE];
			mode = 2;
			++bufIdx;
			continue;
		}

		if(mode == 2) {
			while(bufIdx < bufEnd) {
				char t0 = (char) buf[bufIdx % BUFFER_SIZE];			++bufIdx;
				if(t0 != '!') {
					--bufIdx;
					mode = 0;
					need = 1;
					return false;
				}
				char t1 = (char) buf[bufIdx % BUFFER_SIZE];			++bufIdx;
				switch(t1) {
				case 'R':
					range.addRange(__readUShort(buf, bufIdx));    	bufIdx += 2;
					range.addAngle(__readUShort(buf, bufIdx)); 		bufIdx += 2;
					range.setTimestamp(__readULong(buf, bufIdx));	bufIdx += 8;
					range.setStatus(range.range() == 9999);
					return true;
				//case 'I':
					//gyroTime = __readULong(buf, i);   i += 8;
					//gyro[0] = __readShortR(buf, i);	  i += 2;
					//gyro[1] = __readShortR(buf, i);	  i += 2;
					//gyro[2] = __readShortR(buf, i);	  i += 2;
					//acc[0] = __readShortR(buf, i);	  i += 2;
					//acc[1] = __readShortR(buf, i);	  i += 2;
					//acc[2] = __readShortR(buf, i);	  i += 2;
					break;
				default: 
					bufIdx -= 2;
					mode = 0;
					need = 1;
					return false;
				}
			}
		}
	}
	return false;
}

