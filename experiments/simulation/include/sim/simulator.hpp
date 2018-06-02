/*
 * simulator.hpp
 *
 *  Created on: Jun 2, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_SIMULATOR_HPP_
#define INCLUDE_SIM_SIMULATOR_HPP_

#include <memory>
#include <thread>
#include <string>

#include "platform.hpp"

using namespace uav::sim;

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
	Platform m_platform;
	std::unique_ptr<std::thread> m_thread;
	std::vector<SimulatorObserver*> m_obs;

public:
	Simulator();
	void run();
	void start();
	void stop();
	void setTerrainFile(const std::string& file);
	void addObserver(SimulatorObserver* obs);
	~Simulator();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_SIMULATOR_HPP_ */
