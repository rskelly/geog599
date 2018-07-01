/*
 * controller.hpp
 *
 *  Created on: Jun 9, 2018
 *      Author: rob
 */

#ifndef INCLUDE_CONTROLLER_HPP_
#define INCLUDE_CONTROLLER_HPP_

#include "platform.hpp"

namespace uav {

/**
 * This class represents the controller of the
 * UAV and is responsible for ascertaining the dynamic
 * state of the platform, calculating control inputs
 * and sending them to the platform.
 *
 * As this is the "brain" of the operation, it also
 * generates the time signal and uses it to drive the
 * Platform.
 */
class Controller {
public:

	/**
	 * Start the controller. Starts the "brain" of
	 * the platform. Controller "ticks" will commence
	 * when this is called.
	 */
	virtual void start() = 0;

	/**
	 * Stop the controller.
	 */
	virtual void stop() = 0;

	/**
	 * Set the Platform instance. The controller
	 * does not control the instance.
	 *
	 * @param platform A pointer to the Platform instance.
	 *
	 */
	virtual void setPlatform(uav::Platform* platform) = 0;

	virtual ~Controller() {}
};

}



#endif /* INCLUDE_CONTROLLER_HPP_ */
