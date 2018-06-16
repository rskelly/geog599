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

class Simulator;

class SimulatorObserver {
public:
	virtual void simUpdate(Simulator& sim) = 0;
	virtual ~SimulatorObserver() {}
};

class Simulator {
private:
	bool m_running;
	uav::sim::Terrain* m_terrain;
	uav::sim::Platform* m_platform;
	uav::Controller* m_controller;
	std::thread m_thread;
	std::vector<uav::sim::SimulatorObserver*> m_obs;

public:
	Simulator();
	void run();
	void start();
	void stop();
	void setTerrainFile(const std::string& file);
	void setGimbalAngle(double angle);
	void addObserver(SimulatorObserver* obs);
	uav::sim::Terrain* terrain();
	uav::Platform* platform();
	~Simulator();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_SIMULATOR_HPP_ */
