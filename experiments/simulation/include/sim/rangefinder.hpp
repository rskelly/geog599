/*
 * rangefinder.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_RANGEFINDER_HPP_
#define INCLUDE_SIM_RANGEFINDER_HPP_

#include <string>

#include <Eigen/Core>
#include <gdal_priv.h>

#include "../rangefinder.hpp"
#include "../util.hpp"

namespace uav {
namespace sim {

/**
 * Used by the simulator to calculate the range read from a DEM
 * so that it can be emitted by the rangefinder.
 */
class RangeBridge {
public:
	/**
	 * Return the distance from the laser to the DEM surface,
	 * given the transformation parameters of the system.
	 *
	 * @return The distance from laser to target.
	 */
	static double getRange();
};

/**
 * Rangefinder is a simulated range finder that emits ranges as if
 * they were read from a given elevation model. Though the definition of
 * Rangefinder forbids the emission of any information other than range
 * and time, the simulator must have information about the position and
 * rotation of the laser to read a DEM, so the rangefinder gets the
 * calculated range from the RangeBridge class which is a
 * singleton.
 */
class Rangefinder : public uav::Rangefinder {
private:

	double m_scanFreq;
	double m_pulseFreq;

	uav::util::Poisson m_poisson;
	double m_nextTime;

	/**
	 * Compute the measured range when the scanner is at the given angle
	 * at the current position and rotation.
	 *
	 * @param angle The scan angle in radians.
	 * @return The range in metres.
	 */
	double computeRange(double angle) const;

public:

	/**
	 * Create the rangefinder.
	 */
	Rangefinder();

	/**
	 * Set the measurement frequency. This should have noise added.
	 *
	 * @param The frequency as measurements per second.
	 */
	void setPulseFrequency(double freq);

	/**
	 * Set the scan frequency. This should have noise added.
	 *
	 * @param The frequency as oscillations per second.
	 */
	void setScanFrequency(double freq);

	Range* range();

	~Rangefinder();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_RANGEFINDER_HPP_ */
