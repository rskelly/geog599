/*
 * util.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_UTIL_HPP_
#define INCLUDE_UTIL_HPP_

#include <random>
#include <Eigen/Core>

namespace uav {
namespace util {

/**
 * Provides a simple method for retrieving a poisson-distributed
 * value given a mean and a frequency.
 */
class Poisson {
private:
	double m_mean;
	std::default_random_engine m_generator;
	std::poisson_distribution<int> m_distribution;

public:

	Poisson();

	Poisson(double mean);

	void setMean(double mean);

	double next(double freq = 1.0);

	double nextCentered(double freq = 1.0);

};

/**
 * Returns a orientation matrix for the orientation around the given axis vector by the
 * given angle (in radians). The axis i
 *
 * @param vec The axis of orientation. Will be normalized.
 * @param angle The orientation angle in radians.
 * @return A orientation matrix.
 */
Eigen::Matrix3d rotFromAxisAngle(const Eigen::Vector3d& vec, double angle);

Eigen::Matrix3d rotFromEuler(const Eigen::Matrix3d& mtx);

/**
 * Returns the angle in radians.
 *
 * @return The angle in radians.
 */
double toRad(double deg);

/**
 * Returns the angle in degrees.
 *
 * @return The angle in degrees.
 */
double toDeg(double rad);


} // util
} // uav

#endif /* INCLUDE_UTIL_HPP_ */
