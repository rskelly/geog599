/*
 * djiplatform.hpp
 *
 *  Created on: Jan 30, 2019
 *      Author: rob
 */

#ifndef INCLUDE_PLATFORM_DJIPLATFORM_HPP_
#define INCLUDE_PLATFORM_DJIPLATFORM_HPP_

#include <vector>

#include <djiosdk/dji_vehicle.hpp>

#include "platform/platform.hpp"

namespace platform {
namespace dji {

class DJIPlatform : public platform::Platform {
private:

	static constexpr int DJI_TIMEOUT = 1000; // Timeout for blocking calls.

	platform::PlatformListener* m_listener;
	DJI::OSDK::Vehicle* m_vehicle;
	DJI::OSDK::Vehicle::ActivateData m_activateData;

	std::vector<int> m_subPkgIds; // IDs for subscribed packages.

	/**
	 * Load the ActivateData from the configuration file.
	 *
	 * @param config A configuration file.
	 */
	void loadActivateData(const std::string& config);

public:

	DJIPlatform();

	/**
	 * Add a listener to the platform.
	 *
	 * @param listener An instance of an implementation of PlatformListener.
	 */
	void addListener(PlatformListener* listener);

	/**
	 * Start the platform interface and register for control/events.
	 * Will throw an exception on failure.
	 *
	 * @param config A configuration file.
	 */
	void startup(const std::string& config);

	/**
	 * De-register from control/events and shutdown the platform.
	 */
	void shutdown();

	~DJIPlatform();

};

} // dji
} // platform

#endif /* INCLUDE_PLATFORM_DJIPLATFORM_HPP_ */
