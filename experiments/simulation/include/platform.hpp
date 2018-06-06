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
	 * Set a Gimbal instance on the Platform. The caller
	 * maintains ownership of the Gimbal.
	 *
	 * @param gimbal A pointer to a gimbal.
	 */
	virtual void setGimbal(uav::Gimbal* gimbal) = 0;

	/**
	 * Return a pointer to the gimbal.
	 *
	 * @return A pointer to a Gimbal.
	 */
	virtual const Gimbal* gimbal() const = 0;

	/**
	 * Set a Rangefinder instance on the Platform. The caller
	 * maintains ownership of the Rangefinder.
	 *
	 * @param rangefinder A pointer to a rangefinder.
	 */
	virtual void setRangefinder(uav::Rangefinder* rangefinder) = 0;

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

	/**
	 * The update method is the primary driver of the Platform.
	 * The update method may be called from without or within,
	 * but it must be implemented to drive the Platform's
	 * functions. The parameter is the time in seconds since
	 * the epoch.
	 *
	 * @param time The time since the epoch in seconds.
	 */
	virtual void update(double time) = 0;

	/**
	 * Return the caculated laser position. Only available after an update.
	 *
	 * @return The caculated laser position. Only available after an update.
	 */
	virtual const Eigen::Vector3d laserPosition() const = 0;

	/**
	 * Return the caculated laser direction. Only available after an update.
	 *
	 * @return The caculated laser direction. Only available after an update.
	 */
	virtual const Eigen::Vector3d laserDirection() const = 0;

	virtual ~Platform() {}

};

} // uav


#endif /* INCLUDE_PLATFORM_HPP_ */
