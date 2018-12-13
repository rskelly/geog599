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

#include <list>
#include <thread>

#include "sensor/ads1115.hpp"
#include "sensor/teensy.hpp"
//#include "sensor/minimu9v5.hpp"

constexpr double PI = 3.141592653589793;

class Controller {
private:
	//sensor::ADS1115 m_adc;
	sensor::Teensy m_scanner;
	//sensor::MinIMU9v5 m_imu;

public:

	Controller() {}

	bool start() {
		using namespace sensor;
		/*
		m_imu.setGyroDataRate(GDR_1660Hz);
		m_imu.setGyroFullScale(GFS_245dps);
		m_imu.setAccelDataRate(ADR_1660Hz);
		m_imu.setAccelFullScale(AFS_2g);
		if(!m_imu.open("/dev/i2c-1", 0x6b, 0x1e)) {
			std::cerr << "Failed to connect to IMU.\n";
			return false;
		}
		*/
		if(!m_scanner.open("/dev/ttyACM0", B921600)) {
			std::cerr << "Failed to connect to scanner.\n";
			return false;
		}
		/*
		if(m_adc.open("/dev/i2c-1", 0x48)) {
			m_adc.setGain(0);
			m_adc.setDataRate(0b111);
			m_adc.setOneShot();
			if(!m_adc.saveConfig()) {
				std::cerr << "Failed to save ADC config.\n";
				return false;
			}
		} else {
			std::cerr << "Failed to connect to ADC.\n";
			return false;
		}
		*/
		return true;
	}

	void stop() {
		//m_imu.close();
	}

	std::list<double> __range;
	std::list<double> __angle;

	inline double __avg(std::list<double>& l) {
		double s = 0;
		for(double d : l)
			s += d;
		return s / l.size();
	}

	inline void __push(std::list<double>& l, double v) {
		l.push_back(v);
		if(l.size() > 5)
			l.pop_front();
	}

	bool step(double t) {
		// IMU
		//const sensor::MinIMU9v5State& imu = m_imu.getState();
		//if(imu.updated())
		//	imu.print(std::cout);

		// Laser
		sensor::Range range;
		sensor::Orientation orientation;
		if(m_scanner.readData(range, orientation)) {
			if(range.status() == 0)
				std::cout << range.angle() << ", " << range.range() << ", " << range.x() << ", " << range.y() << ", " << range.timestamp() << "\n";
			//std::cout << range.x() << ", " << range.y() << ", " << range.z() << "\n";
		}

		// Encoder
		/*
		int cur, max;
		if(!m_adc.readValue(0b101, cur))
			return false;
		if(!m_adc.readValue(0b100, max))
			return false;
		std::cout << cur << ", " << max << "\n";
		std::cout << "Angle: " << (double) cur / max * 360.0 << "\n";
		*/
		return true;
	}

	~Controller() {
		stop();
	}

};

int main(int argc, char** argv) {

	Controller cont;
	double t = 0.0;
	if(cont.start()) {
		while(true) {
			t += 1.0;
			if(!cont.step(t))
				break;
			std::this_thread::yield();
		}
		cont.stop();
	}

	return 0;
}


