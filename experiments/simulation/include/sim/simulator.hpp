/*
 * simulator.hpp
 *
 *  Created on: Jun 2, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_SIMULATOR_HPP_
#define INCLUDE_SIM_SIMULATOR_HPP_

#include <thread>
#include <string>

#include <Eigen/Core>

#include "sim/platform.hpp"
#include "sim/terrain.hpp"
#include "controller.hpp"

namespace uav {
namespace sim {

class Simulator {
private:
	bool m_running;
	uav::sim::Terrain* m_terrain;
	uav::sim::Platform* m_platform;
	uav::Controller* m_controller;

public:
	Simulator();
	void start();
	void stop();
	void setTerrainFile(const std::string& file);
	void setGimbalAngle(double angle);
	uav::sim::Terrain* terrain();
	uav::Platform* platform();
	~Simulator();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_SIMULATOR_HPP_ */
