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

	double nextCentred(double freq = 1.0);

};

class MatrixUtil {
public:

	/**
	 * Returns a rotation matrix for the rotation around the given axis vector by the
	 * given angle (in radians). The axis i
	 *
	 * @param vec The axis of rotation. Will be normalized.
	 * @param angle The rotation angle in radians.
	 * @return A rotation matrix.
	 */
	static Eigen::Matrix3d rotFromAxisAngle(const Eigen::Vector3d& vec, double angle);

	static double toRad(double deg);

	static double toDeg(double rad);

};

#endif /* INCLUDE_UTIL_HPP_ */
