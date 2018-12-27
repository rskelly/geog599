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

//constexpr double PI = 3.141592653589793;

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
		if(!m_scanner.open("/dev/ttyACM0", B115200)) {//B921600)) {
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

	bool step(double t, std::vector<sensor::Range>& ranges, sensor::Orientation& orientation) {
		// Laser package (laser/IMU)
		return m_scanner.readData(ranges, orientation);
	}

	~Controller() {
		stop();
	}

};

#define G 9.80665

int main(int argc, char** argv) {

	Controller cont;
	std::vector<sensor::Range> ranges;
	sensor::Orientation orientation;

	Eigen::Vector3d g(0.0, 0.0, 1.0); // Down

	double px = 0, py = 0, pz = 0;
	double rx = 0, ry = 0, rz = 0;
	double vpx = 0, vpy = 0, vpz = 0;
	double vrx = 0, vry = 0, vrz = 0;
	uint64_t ot0 = 0, ot1 = 0;
	double t = 0.0;
	if(cont.start()) {
		while(true) {
			t += 1.0;
			if(!cont.step(t, ranges, orientation)) {
				break;
			} else {
				for(sensor::Range& r : ranges) {
					//std::cerr << "Range: " << r.range() << ", " << r.timestamp() << "\n";
				}
				ranges.clear();
				if(ot0 == 0) {
					ot0 = ot1 = orientation.timestamp();
				} else {
					ot1 = orientation.timestamp();
				}
				Eigen::Vector3d accel = orientation.accel();
				Eigen::Vector3d gyro = orientation.gyro();
				std::cerr << "Accel: " << accel.norm() << ", " << accel[0] << ", " << accel[1] << ", " << accel[2] << ", " << ot1 << "\n";
				std::cerr << "Gyro: " << gyro.norm() << ", " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << ", " << ot1 << "\n";
				if(ot1 - ot0 > 0) {
					double ot = (double) (ot1 - ot0) / 1000000.0;
					vpx += accel[0] * ot;
					vpy += accel[1] * ot;
					vpz += accel[2] * ot;
					vrx += gyro[0] * ot;
					vry += gyro[1] * ot;
					vrz += gyro[2] * ot;
					px += vpx * ot;
					py += vpy * ot;
					pz += vpz * ot;
					rx += vrx * ot;
					ry += vry * ot;
					rz += vrz * ot;
					//std::cerr << "Orient: gyro: " << ot << ", " << o.gyro() << ", accel: " << a << ", " << o.timestamp() << "\n";
					//std::cerr << "Pos: " << px << ", " << py << ", " << pz << "\n";
					//std::cerr << "Ori: " << rx << ", " << ry << ", " << rz << "\n";
					//std::cerr << "a " << accel[0] << ", " << accel[1] << ", " << accel[2] << "\n";
					//std::cerr << "g " << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << "\n";
				}
				ot0 = ot1;
			}
			std::this_thread::yield();
		}
		cont.stop();
		std::cerr << "Stopped.\n";
	}

	return 0;
}


