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
#include "surface.hpp"

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
	 * Set a Gimbal instance on the Platform. The Platform takes
	 * ownership of the Gimbal.
	 *
	 * @param gimbal A pointer to a gimbal.
	 */
	virtual void setGimbal(uav::Gimbal* gimbal) = 0;

	/**
	 * Return a pointer to the gimbal.
	 *
	 * @return A pointer to a Gimbal.
	 */
	virtual uav::Gimbal* gimbal() const = 0;

	/**
	 * Set a Rangefinder instance on the Platform. The Platform
	 * takes ownership of the Rangefinder.
	 *
	 * @param rangefinder A pointer to a Rangefinder.
	 */
	virtual void setRangefinder(uav::Rangefinder* rangefinder) = 0;

	/**
	 * Return a pointer to the Rangefinder.
	 *
	 * @return A pointer to the Rangefinder.
	 */
	virtual uav::Rangefinder* rangefinder() const = 0;

	/**
	 * Set a Rangefinder instance on the Platform for elevation measurement. The caller
	 * maintains ownership of the Rangefinder.
	 *
	 * @param rangefinder A pointer to a nadir Rangefinder.
	 */
	virtual void setNadirRangefinder(uav::Rangefinder* rangefinder) = 0;

	/**
	 * Return a pointer to the nadir Rangefinder.
	 *
	 * @return A pointer to the nadir Rangefinder.
	 */
	virtual uav::Rangefinder* nadirRangefinder() const = 0;

	/**
	 * Return the platform elevation as measured by the nadir rangefinder.
	 *
	 * @return The platform elevation as measured by the nadir rangefinder.
	 */
	virtual double elevation() = 0;

	/**
	 * Set a pointer to the surface generator. The Platform takes ownership.
	 *
	 * @param surface A pointer to the surface generator.
	 */
	virtual void setSurface(uav::surface::Surface* surface) = 0;

	/**
	 * Return a pointer to the surface generator.
	 *
	 * @return A pointer to the surface generator.
	 */
	virtual uav::surface::Surface* surface() const = 0;

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
