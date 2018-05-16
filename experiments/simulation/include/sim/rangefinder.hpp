/*
 * rangefinder.hpp
 *
 *  Created on: May 13, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_RANGEFINDER_HPP_
#define INCLUDE_SIM_RANGEFINDER_HPP_

#include <string>
#include <random>

#include <Eigen/Core>
#include <gdal_priv.h>

#include "../rangefinder.hpp"

/**
 * SimRangefinder is a simulated range finder that emits ranges as if
 * they were read from a given elevation model. Though the definition of
 * Rangefinder forbids the emission of any information other than range
 * and time, the simulator must have information about the orientation of the
 * platform in order to correctly calibrate the ranges. A real-world Rangefinder
 * implementation would not require this.
 */
class SimRangefinder : public Rangefinder {
public:
	enum ScanType {
		None,
		Scan // Requires params for scan angle and frequency.
	};
private:
	ScanType m_scanType;
	double m_scanParam1;
	double m_scanParam2;

	double m_scanFreq;
	double m_pulseFreq;

	GDALDataset m_demds;
	int m_demband;

	std::default_random_engine m_generator;
	std::poisson_distribution<double> m_distribution;
	double m_nextTime;

	/**
	 * Compute the scan angle given the time.
	 *
	 * @param time The time in seconds.
	 * @return The scan angle in radians.
	 */
	double computeScanAngle(double time) const;

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
	SimRangefinder();

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

	/**
	 * Set the filename of the DEM to be used for simulating ranges.
	 */
	void setDEM(const std::string& filename, int band = 1);

	/**
	 * Set the orientation. This happens repeatedly in real-time,
	 * every time a position update becomes available. This is the body or frame
	 * orientation relative to the inertial frame, plus the instrument orientation
	 * relative to the frame. Note that the range received is in no way corrected.
	 * The caller must still use the orientation to transform the range into a
	 * coordinate as it would with a real instrument.
	 */
	void setOrientation(const Eigen::Matrix3d& mtx);

	/**
	 * This is the position. Includes forward, lateral and vertical position.
	 */
	void setPosition(const Eigen::Matrix3d& mtx);

	/**
	 * Set the scan type.
	 */
	void setScanType(ScanType type, double param1 = 0, double param2 = 0);

	Range* range() const;

	~SimRangefinder() {}
};


#endif /* INCLUDE_SIM_RANGEFINDER_HPP_ */