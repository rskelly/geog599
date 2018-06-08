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

#include "sim/platform.hpp"
#include "sim/terrain.hpp"

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
	std::thread m_thread;
	std::vector<uav::sim::SimulatorObserver*> m_obs;

public:
	Simulator();
	void run();
	void start();
	void stop();
	void setTerrainFile(const std::string& file);
	void addObserver(SimulatorObserver* obs);
	uav::sim::Terrain* terrain();
	uav::Platform* platform();
	~Simulator();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_SIMULATOR_HPP_ */
