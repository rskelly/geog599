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
#include "terrain.hpp"

namespace uav {
namespace sim {

/**
 * Used by the simulator to calculate the range read from a DEM
 * so that it can be emitted by the rangefinder.
 */
class RangeBridge {
private:
	Terrain m_terrain;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_direction;

public:
	RangeBridge();

	/**
	 * Return the distance from the laser to the DEM surface,
	 * given the transformation parameters of the system.
	 *
	 * @return The distance from laser to target.
	 */
	static double getRange();

	/**
	 * Set the laser position and direction for the next range retrieval.
	 *
	 * @param position The laser position.
	 * @param direction The laser beam direction.
	 */
	static void setLaser(const Eigen::Vector3d& position, const Eigen::Vector3d& direction);

	/**
	 * The terrain DEM file.
	 *
	 * @param file The terrain DEM file.
	 */
	static void setTerrainFile(const std::string& file);

};

/**
 * Rangefinder is a simulated range finder that emits ranges as if
 * they were read from a given elevation model. The simulated
 * rangefinder gets the calculated range from the RangeBridge class which
 * is a singleton.
 */
class Rangefinder : public uav::Rangefinder {
private:

	double m_pulseFreq;

	uav::util::Poisson m_poisson;
	double m_nextTime;

	/**
	 * Compute the measured range when the scanner is at the given angle
	 * at the current position and orientation.
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

	Range* range();

	~Rangefinder();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_RANGEFINDER_HPP_ */
