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
	double range() const;

	/**
	 * Return the time at which the range measurement was taken.
	 *
	 * @return The time at which the range measurement was taken.
	 */
	double time() const;
};

/**
 * This interface represents an object that emits information about the
 * state of a range finder. The current understanding of this device
 * is that it knows the return distance and time of each pulse, and nothing else.
 * Any information about the rotation of the instrument, the position of the
 * platform or the transformation of space is external.
 */
class Rangefinder {
public:

	/**
	 * Returns a Range object containing the range information that was
	 * available when the call was made. If a null pointer is returned,
	 * no range measurement has been taken in the interval since the
	 * last call. A range can only be retrieved once. The caller owns the
	 * returned pointer.
	 *
	 * @return A pointer to a Range object or nullptr.
	 */
	virtual Range* range() = 0;


	virtual ~Rangefinder() {}

};

} // uav

#endif /* INCLUDE_RANGEFINDER_HPP_ */
