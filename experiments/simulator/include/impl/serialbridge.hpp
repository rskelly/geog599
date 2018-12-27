/*
 * SerialBridge.hpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#ifndef SRC_IMPL_SERIALBRIDGE_HPP_
#define SRC_IMPL_SERIALBRIDGE_HPP_

#include <thread>
#include <unordered_set>

#include "sensor/teensy.hpp"

namespace uav {
namespace impl {

class SerialBridge;

/**
 * PRovides an implementation for a listener for SerialBridge updates.
 */
class SerialBridgeListener {
public:

	/**
	 * Called when the SerialBridge has an update.
	 */
	virtual void serialBridgeUpdate(SerialBridge* bridge) = 0;

	virtual ~SerialBridgeListener() {}

};

/**
 * The SerialBridge provides a central way for device implementations to
 * receive data from the sensor hardware, including ranges and IMU
 * information.
 */
class SerialBridge {
private:
	std::unordered_set<SerialBridgeListener*> m_listeners;

	sensor::Teensy m_teensy;
	std::vector<sensor::Range> m_ranges;
	sensor::Orientation m_orientation;

	bool m_running;
	std::thread m_thread;

	SerialBridge();

public:

	/**
	 * Return the single instance of the SerialBridge.
	 *
	 * @return The single instance of the SerialBridge.
	 */
	static SerialBridge& getInstance();

	/**
	 * Returns true if the bridge is currently running.
	 *
	 * @return True if the bridge is currently running.
	 */
	bool running() const;

	/**
	 * Update the bridge with the current state of the hardware object.
	 *
	 * @param teensy The hardware object.
	 */
	void update(sensor::Teensy& teensy);

	/**
	 * Set the listener for SerialBridge updates.
	 *
	 * @param listener A listener for SerialBridge updates.
	 */
	void addListener(SerialBridgeListener* listener);

	/**
	 * Remove a listener from the listeners list.
	 *
	 * @param A listener to remove.
	 */
	void removeListener(SerialBridgeListener* listener);

	/**
	 * Start receiving information from the device.
	 */
	void start();

	/**
	 * Stop receiving information from the device.
	 */
	void stop();

	/**
	 * Return the reference to the Orientation.
	 *
	 * @return The reference to the Orientation.
	 */
	const sensor::Orientation& orientation() const;

	/**
	 * Return the reference to the Range vector.
	 *
	 * @return The reference to the Range vector.
	 */
	const std::vector<sensor::Range>& ranges() const;

};

} // impl
} // uav



#endif /* SRC_IMPL_SERIALBRIDGE_HPP_ */
