/*
 * controller.hpp
 *
 *  Created on: Jun 9, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_CONTROLLER_HPP_
#define INCLUDE_SIM_CONTROLLER_HPP_

#include <thread>

#include "../controller.hpp"
#include "../platform.hpp"

namespace uav {
namespace sim {

class Controller : public uav::Controller {
private:
	uav::Platform* m_platform;
	std::thread m_thread;
	bool m_running;

public:
	Controller();

	void tick(double time);

	void start();

	void stop();

	void setPlatform(uav::Platform* platform);

	~Controller();
};

} // sim
} // uav



#endif /* INCLUDE_SIM_CONTROLLER_HPP_ */
