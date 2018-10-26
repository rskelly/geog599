/*
 * controller.cpp
 *
 * This program runs the sensor package and maintains its state.
 *
 * 1) Collect ranges from the scanning laser.
 * 2) Collect ranges from the nadir laser.
 * 3) Collect angles from the encoder (static angles between encoder and body frame are configured.)
 * 4) Collect accelerometer data for pointing down.
 * 5) Drive angle motor.
 * 6) Drive gimbal motor.
 * 8) Get position information from the DJI API.
 * 7) Compute flight control inputs.
 * 9) Send flight control inputs to the DJI API.
 *
 *  Created on: Oct 25, 2018
 *      Author: rob
 */

#include "sensor/ads1115.hpp"
#include "sensor/lwsf30c.hpp"
#include "sensor/minimu9v5.hpp"


class Controller {
private:
	sensor::ADS1115 m_adc;
	sensor::LWSF30C m_scanner;
	sensor::MinIMU9v5 m_imu;

public:

	Controller() {}

	bool start() {
		if(!m_imu.open("/dev/i2c-1", 0x6b, 0x1e)) {
			std::cerr << "Failed to connect to IMU.\n";
			return false;
		}
		if(m_scanner.open("/dev/ttyS0", B115200)) {
			//if(!m_scanner.setBlocking(false)) 
			//	std::cerr << "Failed to set blocking.\n";
			if(!m_scanner.startLaser()) {
				std::cerr << "Failed to start laser.\n";
				return false;
			}
		} else {
			std::cerr << "Failed to connect to scanner.\n";
			return false;
		}
		return true;
	}

	void stop() {
		m_imu.close();
	}

	bool step() {
		const sensor::MinIMU9v5State& imu = m_imu.getState();
		if(!imu.updated())
			imu.print(std::cout);
		double r = m_scanner.getMeasurement();
		std::cout << r << "m\n";
		return true;
	}

	~Controller() {
		stop();
	}

};

int main(int argc, char** argv) {

	Controller cont;
	if(cont.start()) {
		while(true) {
			if(!cont.step())
				break;
		}
		cont.stop();
	}

	return 0;
}
