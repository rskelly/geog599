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

constexpr size_t BUFFER_SIZE = 2048;
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
	for(int i = 0; i < 8; ++i)
		u |= (buf[(start + i) % BUFFER_SIZE] << ((7 - i) * 8));
	return u;
}

inline unsigned short __readUShort(uint8_t* buf, size_t start) {
	unsigned short s = 0;
	for(int i = 0; i < 2; ++i)
		s |= (buf[(start + i) % BUFFER_SIZE] << ((1 - i) * 8));
	return s;
}

// Read short low byte first
inline short __readShortR(uint8_t* buf, size_t start) {
	short s = 0;
	for(int i = 0; i < 2; ++i)
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

		std::this_thread::yield();

		//std::cerr << bufIdx << ", " << bufEnd << ", " << mode << ", " << need << "\n";

		if((bufEnd - bufIdx) < need) {
			int rd = read((char *) readBuf, BUFFER_SIZE);
			if(rd <= 0)
				return false;
			for(size_t i = 0; i < rd; ++i) {
				buf[bufEnd % BUFFER_SIZE] = readBuf[i];
				++bufEnd;
			}
			if((bufEnd - bufIdx) < need)
				continue;
		}
		
		if(mode == 0) {
			while(bufIdx < bufEnd) {
				if(buf[bufIdx % BUFFER_SIZE] == '#') {
					mode = 1;
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
			char t = (char) buf[bufIdx % BUFFER_SIZE]; 		++bufIdx;
			//std::cerr << t << "\n";
			switch(t) {
			case 'R':
				range.addRange(__readUShort(buf, bufIdx));    	bufIdx += 2;
				range.addAngle(__readUShort(buf, bufIdx)); 		bufIdx += 2;
				range.setTimestamp(__readULong(buf, bufIdx));	bufIdx += 8;
				range.setStatus(0);
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
				--bufIdx;
				mode = 0;
				break;
			}
		}
		return false;
	}
	return false;
}

