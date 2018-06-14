/*
 * rangefinder.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_RANGEFINDER_HPP_
#define INCLUDE_RANGEFINDER_HPP_

namespace uav {

/**
 * This interface represents a range object. It may contain only
 * the distance from the range finder to the target, and the time
 * at which the pulse returned.
 */
class Range {
public:

	/**
	 * Return the range (in m), or distance from instrument to target.
	 *
	 * @return The range (in m), or distance from instrument to target.
	 */
	virtual double range() const = 0;

	/**
	 * Return the time at which the range measurement was taken.
	 *
	 * @return The time at which the range measurement was taken.
	 */
	virtual double time() const = 0;

	/**
	 * Return true if the range and time of the Range are
	 * valid. That is, not null etc.
	 *
	 * @return True if the Range is valid.
	 */
	virtual bool valid() const = 0;

	virtual ~Range() {}
};

class Rangefinder;

/**
 * Implemented by classes that wish to listen for rangefinder events.
 */
class RangefinderObserver {
public:
	/**
	 * The implementor will be notified of a range result from the given
	 * rangefinder.
	 *
	 * @param rangefinder The Rangefinder that generated the Range.
	 * @param range A Range.
	 */
	virtual void rangeUpdate(uav::Rangefinder* rangefinder, uav::Range* range) = 0;

	virtual ~RangefinderObserver() {}
};

/**
 * This interface represents an object that emits range measurements. The current
 * understanding of this device is that it knows the return distance and time of
 * each pulse, and nothing else. Any information about the orientation of the
 * instrument, the position of the platform or the transformation of space is external.
 */
class Rangefinder {
public:

	/**
	 * Adds the Range objects to the given vector. Since the rangefinder
	 * generates ranges at its own pace, the platform may not be able to
	 * request them faster than they're generated. These are all the ranges
	 * generated between this call and the last.
	 *
	 * The caller is responsible for the Range objects and their destruction.
	 *
	 * @return The number of ranges generated.
	 */
	virtual int getRanges(std::vector<uav::Range*>& ranges) = 0;

	/**
	 * Set a pointer to an object that will listen to events from this
	 * Rangefinder.
	 *
	 * @param obs A RangefinderObserver.
	 */
	virtual void setObserver(RangefinderObserver* obs) = 0;

	/**
	 * Start the rangefinder.
	 */
	virtual void start() = 0;

	/**
	 * Stop the rangefinder.
	 */
	virtual void stop() = 0;

	virtual ~Rangefinder() {}

};

} // uav

#endif /* INCLUDE_RANGEFINDER_HPP_ */
