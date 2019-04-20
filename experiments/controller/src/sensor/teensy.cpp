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

inline uint64_t __readULong(uint8_t* buf, size_t start) {
	size_t end = start + 8;
	uint64_t u = 0;
	uint8_t b;
	while(start < end) {
		b = buf[start % BUFFER_SIZE];
		u |= (b << ((end - start - 1) * 8));
		++start;
	}
	return u;
}

inline unsigned short __readUShort(uint8_t* buf, size_t start) {
	return (buf[start % BUFFER_SIZE] << 8) | buf[(start + 1)  % BUFFER_SIZE];
}

inline short __readShort(uint8_t* buf, size_t start) {
	return (buf[start % BUFFER_SIZE] << 8) | buf[(start + 1) % BUFFER_SIZE];
}

// Read short low byte first
inline short __readShortR(uint8_t* buf, size_t start) {
	return (buf[(start + 1) % BUFFER_SIZE] << 8) | buf[start % BUFFER_SIZE];
}


bool Teensy::readData(std::vector<Range>& ranges, Orientation& orientation) {
	static uint8_t buf[BUFFER_SIZE];		// The buffer of all read bytes.
	static uint8_t readBuf[BUFFER_SIZE];	// The buffer for the current read.
	static size_t bufIdx = 0;				// The buffer pointer.
	static size_t bufEnd = 0;				// The last index of read bytes.
	static int mode = 0;					// The current packet read mode.
	static size_t need = 3;					// The number of bytes required to complete the current packet.

	size_t nRanges, nErrors, nIMU;			// The number of ranges, errors or IMU reads expected.
	char headChar;							// The current header character (R, E or I).
	uint64_t time;							// The timestamp of the current packet.
	uint16_t range;							// the current range value.
	int16_t g0, g1, g2;						// The current gyro value.
	int16_t a0, a1, a2;						// The current accel value.

	while(true) {

		// Read the available bytes and write them to the buffer.
		while(bufEnd < bufIdx || (bufEnd - bufIdx) < need) {
			std::this_thread::yield();
			int rd = read((char *) readBuf, BUFFER_SIZE);
			if(rd > 0) {
				for(int i = 0; i < rd; ++i) {
					buf[bufEnd % BUFFER_SIZE] = readBuf[i];
					++bufEnd;
				}
			}
		}
		
		// Start mode. Detect the beginning of a packet.
		if(mode == 0) {
			while(bufIdx < bufEnd) {
				char a = buf[bufIdx % BUFFER_SIZE];		++bufIdx;
				char b = buf[bufIdx % BUFFER_SIZE];		++bufIdx;
				char c = buf[bufIdx % BUFFER_SIZE];		++bufIdx;
//				std::cerr << a << ", " << b << ", " << (int) c << "\n";
				if(a == '#' && b == '!') {
					mode = 1;
					need = c > 0 ? (size_t) c : 0;
					if(need == 0) { // Something is seriously wrong.
						mode = 0;
						need = 1;
					}
					break;
				}
				bufIdx -= 2; // Back up and try again.
			}
			continue;
		}

		// Read the current packet.
		if(mode == 1) {
			while(bufIdx < bufEnd) {
				headChar = (char) buf[bufIdx % BUFFER_SIZE];		++bufIdx;
				switch(headChar) {
				case 'E':
					// Reading an error packet.
					if(bufEnd - bufIdx < 1)
						goto need_more;
					// Get the number of records to read.
					nErrors = (size_t) buf[bufIdx % BUFFER_SIZE];	++bufIdx;
					if(bufEnd - bufIdx < nErrors * 2)
						goto need_more;
					// Read the errors. Each one is a short.
					for(size_t i = 0; i < nErrors; ++i) {
						int error = __readShort(buf, bufIdx);		bufIdx += 2;
						std::cerr << "Error " << i << ": " << error << "\n";
					}
					break;
				case 'R':
					// Reading a range packet.
					if(bufEnd - bufIdx < 1)
						goto need_more;
					// Get the number of ranges to read.
					nRanges = (size_t) buf[bufIdx % BUFFER_SIZE];	++bufIdx;
					if(bufEnd - bufIdx < nRanges * 10)
						goto need_more;
					// Read the ranges. Each one has a ushort range and a ulong time.
					for(size_t i = 0; i < nRanges; ++i) {
						range = __readUShort(buf, bufIdx);			bufIdx += 2;
						time = __readULong(buf, bufIdx); 			bufIdx += 8;
						ranges.emplace_back(range, time);
					}
					break;
				case 'I':
					// Reading an IMU packet.
					if(bufEnd - bufIdx < 20)
						goto need_more;
					// Get the number of IMU records.
					nIMU = (size_t) buf[bufIdx % BUFFER_SIZE];	++bufIdx;
					if(bufEnd - bufIdx < nIMU * 20)
						goto need_more;
					// Each IMU record has 6 short components plus a ulong time.
					for(size_t i = 0; i < nIMU; ++i) {
						g0 = __readShort(buf, bufIdx);		bufIdx += 2;
						g1 = __readShort(buf, bufIdx);		bufIdx += 2;
						g2 = __readShort(buf, bufIdx);		bufIdx += 2;
						a0 = __readShort(buf, bufIdx);		bufIdx += 2;
						a1 = __readShort(buf, bufIdx);		bufIdx += 2;
						a2 = __readShort(buf, bufIdx);		bufIdx += 2;
						time = __readULong(buf, bufIdx);   	bufIdx += 8;
						orientation.update(g0, g1, g2, a0, a1, a2, time);
					}
					break;
				need_more:
					// There's not enough data in the buffer. Read more.
					std::cerr << "need more\n";
					--bufIdx;
					mode = 0;
					need = 3;
					return true; // TODO: Is this right?
				default: 
					// Nothing happened. Move to the next read.
					--bufIdx;
					mode = 0;
					need = 3;
					return true;
				}
			}
			return true;
		}
	}
	return false;
}

