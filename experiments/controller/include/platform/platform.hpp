/*
 * platform.hpp
 *
 *  Created on: Jan 30, 2019
 *      Author: rob
 */

#ifndef INCLUDE_PLATFORM_PLATFORM_HPP_
#define INCLUDE_PLATFORM_PLATFORM_HPP_

#include <string>

namespace platform {

/**
 * Contains the current dynamic state of the platform.
 */
class PlatformState {
public:
	long timestamp;
	// Position
	double x;
	double y;
	double z;
	// Rotation
	double rx;
	double ry;
	double rz;
	// Control
	double throttle;
	double battery;
	// etc
};

/**
 * Implementors can receive updates from a platform.
 */
class PlatformListener {
public:
	/**
	 * Receive a platform state update from a platform.
	 * The PlatformState object contains all current information about
	 * the platform.
	 */
	virtual void platformState(const PlatformState& state) = 0;
	virtual ~PlatformListener() {}
};

/**
 * This class provides the control and telemetry interface
 * for an aircraft.
 */
class Platform {
public:

	/**
	 * Add a listener to the platform.
	 *
	 * @param listener An instance of an implementation of PlatformListener.
	 */
	virtual void addListener(PlatformListener* listener) = 0;

	/**
	 * Start the platform interface and register for control/events.
	 * Will throw an exception on failure.
	 *
	 * @param config A configuration file.
	 */
	virtual void startup(const std::string& config = "") = 0;

	/**
	 * De-register from control/events and shutdown the platform.
	 */
	virtual void shutdown() = 0;

	virtual ~Platform() {}
};

} // platform

#endif /* INCLUDE_PLATFORM_PLATFORM_HPP_ */
