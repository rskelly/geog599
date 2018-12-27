/*
 * rangefindersf30c.hpp
 *
 *  Created on: Dec 26, 2018
 *      Author: rob
 */

#ifndef INCLUDE_IMPL_RANGEFINDERSF30C_HPP_
#define INCLUDE_IMPL_RANGEFINDERSF30C_HPP_

#include "rangefinder.hpp"
#include "impl/serialbridge.hpp"

namespace uav {
namespace impl {

class RangeSF30C : public uav::Range {
private:
	double m_range;
	double m_time;

public:

	void update(double range, double time);

	double range() const;

	double time() const;

	bool valid() const;
};

/**
 * Represents the hardware implementation of the Rangefinder using the LightWare
 * SF30/C. Communication is via the SerialBridge. This device does not
 * read raw range outputs, but interpreted data sent via the microcontroller.
 */
class RangefinderSF30C : public uav::Rangefinder, uav::impl::SerialBridgeListener {
private:
	RangefinderObserver* m_observer;
	uav::impl::RangeSF30C m_range;

public:

	/**
	 * Set a pointer to an object that will listen to events from this
	 * Rangefinder.
	 *
	 * @param obs A RangefinderObserver.
	 */
	void setObserver(RangefinderObserver* obs);

	/**
	 * Receive updates from the SerialBridge.
	 *
	 * @param bridge The SerialBridge.
	 */
	void serialBridgeUpdate(uav::impl::SerialBridge* bridge);

	/**
	 * Start the rangefinder.
	 */
	void start();

	/**
	 * Stop the rangefinder.
	 */
	void stop();

	~RangefinderSF30C();

};

} // impl
} // uav


#endif /* INCLUDE_IMPL_RANGEFINDERSF30C_HPP_ */
