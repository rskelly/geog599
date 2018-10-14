/*
 * ads1115.cpp
 *
 *  Created on: Oct 14, 2018
 *      Author: rob
 */

#include <iostream>
#include <string>

#include <unistd.h>

#include "ads1115.hpp"

ADS1115::ADS1115(const std::string& dev, int addr) :
	I2C(dev, addr),
	m_config(0) {
}

bool ADS1115::saveConfig() {
	// 1) Config register address, 2) MSB of config data, 3) LSB of config data.
	char conf[3] = {1, (char) ((m_config >> 8) & 0xff), (char) (m_config & 0xff)};
	if(!write(conf, 3)) {
		std::cerr << "Failed to save configuration.\n";
		return false;
	}
	return true;
}

bool ADS1115::loadConfig() {
	char conf[1] = {1};
	char value[2] = {0, 0};
	if(!write(conf, 1)) {
		std::cerr << "Failed to load configuration.\n";
		return false;
	}
	if(!read(value, 2)) {
		std::cerr << "Failed to read configuration.\n";
		return false;
	}
	m_config = (value[0] << 8) | value[1];
	return true;
}

bool ADS1115::open() {
	if(I2C::open())
		return loadConfig();
	return false;
}

void ADS1115::startConversion() {
	m_config |= 0b1000000000000000;
}

bool ADS1115::isConverting() const {
	return (m_config & 0b1000000000000000) == 0;
}

void ADS1115::setSlot(int value) {
	// (multiplexer)
	m_config = (m_config & 0b1000111111111111) | (value & 0b111) << 12;
}

int ADS1115::slot() const {
	return (m_config & 0b0111000000000000) >> 12;
}

void ADS1115::setGain(int value) {
	m_config = (m_config & 0b1111000111111111) | (value & 0b111) << 9;
}

int ADS1115::gain() const {
	return (m_config & 0b0000111000000000) >> 9;
}

void ADS1115::setContinuous() {
	m_config &= 0b1111111011111111;
}

bool ADS1115::isContinuous() const {
	return ~((m_config & 0b0000000100000000) >> 8);
}

void ADS1115::setOneShot() {
	m_config |= 0b0000000100000000;
}

bool ADS1115::isOneShot() const {
	return (m_config & 0b0000000100000000) >> 8;
}

void ADS1115::setDataRate(int rate) {
	m_config = (m_config & 0b1111111100011111) | (rate & 0b111) << 5;
}

int ADS1115::dataRate() const {
	return (m_config & 0b0000000011100000) >> 5;
}

int ADS1115::dataRateS() const {
	int rate = dataRate();
	switch(rate) {
	case 0b000: return 8;
	case 0b001: return 16;
	case 0b010: return 32;
	case 0b011: return 64;
	case 0b100: return 128;
	case 0b101: return 250;
	case 0b110: return 475;
	case 0b111: return 860;
	default: return 0;
	}
}

bool ADS1115::readValue(int slot, int& value) {

	if(slot != this->slot()) {
		setSlot(slot);
		setOneShot();
		startConversion();
		saveConfig();
	}

	usleep((int) (1000000.0 / dataRateS()));

	// Select the conversion address.
	char addr[1] = {0};
	if(!write(addr, 1)) {
		std::cerr << "Failed to select conversion register.\n";
		return false;
	}

	// Read the register.
	char buf[2] = {0, 0};
	if(read(buf, 2)) {
		value = (buf[0] << 8) | buf[1];
		return true;
	} else {
		std::cerr << "Failed to read conversion register (2).\n";
		return false;
	}
}



int main(int argc, char** argv) {

	ADS1115 conn("/dev/i2c-1");
	if(conn.open()) {

		int x = 100000;
		int cur = 0, max = 0;
		conn.setGain(0b000);
		conn.setDataRate(0b111);

		while(--x) {
			if(!conn.readValue(0b100, cur))
				break;
			if(!conn.readValue(0b101, max))
				break;
			std::cout <<  cur << ", " << max << ", " << 360.0 * cur / max << "\n";
		}
	}
	conn.close();
	return 0;
}

