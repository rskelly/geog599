/*
 * platform.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_PLATFORM_HPP_
#define INCLUDE_PLATFORM_HPP_

#include <Eigen/Core>

#include "gimbal.hpp"
#include "rangefinder.hpp"

namespace uav {

/**
 * This interface provides information about the platform in real-time.
 * Information such as the position and orientation, time sync signal
 * battery life and other flight parameters is available through the
 * methods in this interface.
 */
class Platform {
public:

	/**
	 * Return a pointer to the gimbal.
	 *
	 * @return A pointer to a Gimbal.
	 */
	virtual const Gimbal* gimbal() const = 0;

	/**
	 * Return a pointer to the rangefinder.
	 *
	 * @return A pointer to the rangefinder.
	 */
	virtual const Rangefinder* rangefinder() const = 0;

	/**
	 * Get the current state of orientation of the platform. This is the orientation
	 * of the platform around its center of mass. Relative to the inertial frame.
	 * Euler angles.
	 *
	 * @return The platform's orientation.
	 */
	virtual const Eigen::Vector3d& orientation() const = 0;

	/**
	 * Get the current position of the platform. This is the position of the
	 * platform's center of mass relative to the inertial frame.
	 *
	 * @return The platform's position.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	virtual ~Platform() {}

};

} // uav


#endif /* INCLUDE_PLATFORM_HPP_ */
