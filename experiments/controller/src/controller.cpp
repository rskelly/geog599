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

	void step() {

	}

};


