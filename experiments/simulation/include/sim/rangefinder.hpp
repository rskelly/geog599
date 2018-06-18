/*
 * rangefinder.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_RANGEFINDER_HPP_
#define INCLUDE_SIM_RANGEFINDER_HPP_

#include <string>
#include <thread>

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
	Terrain* m_terrain;
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_direction;
	Eigen::Vector3d m_prevPosition;  ///<! These are kept to interpolate between the last position and the current one.
	Eigen::Vector3d m_prevDirection; ///<! These are kept to interpolate between the last position and the current one.
	double m_lastTime;               ///<! Time of the last request for ranges.

public:
	RangeBridge();

	/**
	 * Return the distance from the laser to the DEM surface,
	 * given the transformation parameters of the system.
	 *
	 * @return The distance from laser to target.
	 */
	double getRange();

	/**
	 * Calculate the interpolated ranges and times between the current
	 * time and the last request.
	 *
	 * @param pulseCount The number of pulses per second.
	 * @param ranges A vector to hold the interpolated ranges.
	 * @param times A vector to hold the interpolated times.
	 */
	void getInterpRanges(int pulseCount, std::vector<double>& ranges, std::vector<double>& times);

	/**
	 * Set the laser position and direction for the next range retrieval.
	 *
	 * @param position The laser position.
	 * @param direction The laser beam direction.
	 */
	void setLaser(const Eigen::Vector3d& position, const Eigen::Vector3d& direction);

	/**
	 * Set the terrain.
	 *
	 * @param terrain The terrain.
	 */
	void setTerrain(uav::sim::Terrain* terrain);

	/**
	 * Return a pointer to the terrain.
	 *
	 * @return A pointer to the terrain.
	 */
	Terrain* terrain();

};

/**
 * Rangefinder is a simulated range finder that emits ranges as if
 * they were read from a given elevation model. The simulated
 * rangefinder gets the calculated range from the RangeBridge class which
 * is a singleton.
 */
class Rangefinder : public uav::util::ClockObserver, public uav::Rangefinder {
private:
	uav::util::Poisson m_poisson;
	uav::util::Gaussian m_gauss;
	uav::RangefinderObserver* m_obs;
	uav::sim::RangeBridge* m_bridge;
	double m_pulseFreq;
	double m_nextTime;
	bool m_running;
	std::thread m_thread;

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
	 * Generate a pulse.
	 */
	void generatePulse();

	/**
	 * Set the measurement frequency. This should have noise added.
	 *
	 * @param The frequency as measurements per second.
	 */
	void setPulseFrequency(double freq);

	/**
	 * Set a RangeBridge which the Rangefinder can use to measure a simulated
	 * terrain. Takes ownership of the object.
	 *
	 * @param bridge A RangeBridge instance.
	 */
	void setRangeBridge(uav::sim::RangeBridge* bridge);

	/**
	 * Return a pointer to the RangeBridge instance.
	 *
	 * @return A pointer to the RangeBridge instance.
	 */
	uav::sim::RangeBridge* rangeBridge() const;

	void setObserver(uav::RangefinderObserver* obs);

	void start();

	void stop();

	void tick(double time);

	~Rangefinder();
};

} // sim
} // uav

#endif /* INCLUDE_SIM_RANGEFINDER_HPP_ */
