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
	 * Get the current state of rotation of the platform. (Is this relative?)
	 */
	const Eigen::Matrix3d& rotation() const;

	/**
	 * Get the current position of the platform. (Is this geographic or relative?)
	 */
	const Eigen::Matrix3d& position() const;

};


#endif /* INCLUDE_PLATFORM_HPP_ */
