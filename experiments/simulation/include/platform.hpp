/*
 * platform.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_PLATFORM_HPP_
#define INCLUDE_PLATFORM_HPP_

#include <Eigen/Core>

/**
 * This interface provides information about the platform in real-time.
 * Information such as the position and orientation, time sync signal
 * battery life and other flight parameters is available through the
 * methods in this interface.
 */
class Platform {
public:

	/**
	 * Get the current state of orientation of the platform. Relative
	 * to the inertial frame.
	 *
	 * @return A Vector3d containing the orientation.
	 */
	virtual const Eigen::Vector3d& orientation() const = 0;

	/**
	 * Get the current position of the platform. Relative to the inertial frame.
	 *
	 * @return A Vector3d containing the position.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	virtual ~Platform() {}

};


#endif /* INCLUDE_PLATFORM_HPP_ */
