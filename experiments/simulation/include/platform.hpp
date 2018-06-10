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

class Platform;

/**
 * An object that contains information about a
 * Platform that the platform has seen fit
 * to report. Retrieved from the Platform by a
 * Controller instance.
 */
class PlatformState {
public:

	/**
	 * The position of the platform relative to the inertial frame.
	 *
	 * @return The position of the platform relative to the inertial frame.
	 */
	virtual const Eigen::Vector3d& position() const = 0;

	/**
	 * The orientation of the platform relative to the inertial frame.
	 *
	 * @return The orientation of the platform relative to the inertial frame.
	 */
	virtual const Eigen::Vector3d& orientation() const = 0;

	/**
	 * The linear velocity of the platform relative to the inertial frame.
	 *
	 * @return The linear velocity of the platform relative to the inertial frame.
	 */
	virtual const Eigen::Vector3d& linearVelocity() const = 0;

	/**
	 * The angular velocity of the platform relative to the inertial frame.
	 *
	 * @return The angular velocity of the platform relative to the inertial frame.
	 */
	virtual const Eigen::Vector3d& angularVelocity() const = 0;

	/**
	 * The current battery level, between 0 and 1.
	 *
	 * @return The current battery level, between 0 and 1.
	 */
	virtual double batteryLevel() const = 0;

	/**
	 * The mass of the platform in kg.
	 *
	 * @return The mass of the platform in kg.
	 */
	virtual double mass() const = 0;

	/**
	 * Returns the altitude.
	 *
	 * @return The altitude.
	 */
	virtual double altitude() const = 0;

	/**
	 * Return the exact time the altitude was measured.
	 *
	 * @return The exact time the altitude was measured.
	 */
	virtual double altitudeTime() const = 0;

	virtual ~PlatformState() {}
};

/**
 * Represents the state of the rangefinder.
 */
class RangefinderState {
public:

	/**
	 * Return the caculated laser position. Only available after an update.
	 *
	 * @return The caculated laser position. Only available after an update.
	 */
	virtual const Eigen::Vector3d& laserPosition() const = 0;

	/**
	 * Return the caculated laser direction. Only available after an update.
	 *
	 * @return The caculated laser direction. Only available after an update.
	 */
	virtual const Eigen::Vector3d& laserDirection() const = 0;

	virtual ~RangefinderState() {}
};

/**
 * An object containing a set of inputs to the
 * platforms controls. This could operate on the
 * low level (e.g. throttle inputs) or a higher
 * level (e.g. increase elevation). Passed from
 * a Controller object to the Platform.
 */
class PlatformControlInput {
public:
	/**
	 * Set the target elevation of the platform.
	 *
	 * @param elevation The target elevation of the platform.
	 */
	virtual void setAltitude(double altitude) = 0;

	virtual ~PlatformControlInput() {}
};

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
	 * Returns the RangefinderState object for the nadir rangefinder.
	 *
	 * @return The RangefinderState object for the nadir rangefinder.
	 */
	virtual const uav::RangefinderState& nadirRangefinderState() const = 0;

	/**
	 * Returns the RangefinderState object for the rangefinder.
	 *
	 * @return The RangefinderState object for the rangefinder.
	 */
	virtual const uav::RangefinderState& rangefinderState() const = 0;

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
	 * Return a reference the PlatformState object for this Platform.
	 *
	 * @return A reference the PlatformState object for this Platform.
	 */
	virtual const uav::PlatformState& platformState() const = 0;

	/**
	 * Passes a control input to the Platform.
	 *
	 * @param input An instance of PlatformControlInput.
	 */
	virtual void setControlInput(const uav::PlatformControlInput& input) = 0;

	virtual ~Platform() {}

};

} // uav


#endif /* INCLUDE_PLATFORM_HPP_ */
