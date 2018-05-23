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
 * Information such as the position and rotation, time sync signal
 * battery life and other flight parameters is available through the
 * methods in this interface.
 */
class Platform {
public:

	/**
	 * Get the current state of rotation of the platform. This is the rotation
	 * of the platform around its center of mass. Relative to the inertial frame.
	 *
	 * @return The platform's rotation.
	 */
	virtual const Eigen::Matrix3d& rotation() const = 0;

	/**
	 * Get the current position of the platform. This is the position of the
	 * platform's center of mass relative to the inertial frame.
	 *
	 * @return The platform's position.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	virtual ~Platform() {}

};


#endif /* INCLUDE_PLATFORM_HPP_ */
