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
 * Provides a simple method for retrieving a Gaussian-distributed
 * value given a mean.
 */
class Gaussian {
private:
	double m_mean;
	double m_stddev;
	std::mt19937 m_gen;
	std::normal_distribution<double> m_distribution;

public:

	Gaussian();

	Gaussian(double mean, double stddev);

	void setMean(double mean);

	void setStdDev(double stddev);

	double next();

};

/**
 * Get the current time in UTC seconds from epoch.
 *
 * @return The current time in UTC seconds from epoch.
 */
double uavtime();

void rotate(const Eigen::Vector3d& euler, Eigen::Vector3d& orientation);

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
 * Gives the rotation matrix around x for the given angle.
 *
 * @param r An angle.
 * @return A rotation matrix.
 */
Eigen::Matrix3d rotateX(double r);

/**
 * Gives the rotation matrix around y for the given angle.
 *
 * @param r An angle.
 * @return A rotation matrix.
 */
Eigen::Matrix3d rotateY(double r);

/**
 * Gives the rotation matrix around z for the given angle.
 *
 * @param r An angle.
 * @return A rotation matrix.
 */
Eigen::Matrix3d rotateZ(double r);

/**
 * Converts three Euler angles to a rotation matrix.
 *
 * @param lat Latitude.
 * @param lon Longitude.
 * @param twist Twist (around z axis.)
 * @return A rotation matrix.
 */
Eigen::Matrix3d rotMatrix(double lat, double lon, double twist);

/**
 * Converta a vector containing Euler angles to a rotation matrix.
 *
 * @param vec A vector containing Euler angles (z, y, z).
 * @return A rotation matrix.
 */
Eigen::Matrix3d eulerToMatrix(const Eigen::Vector3d& vec);

/**
 * Converts a rotation matrix to a vector containing Euler angles.
 *
 * @param mtx A rotation matrix.
 * @return A vector containing Euler angles (z, y, z).
 */
Eigen::Vector3d matrixToEuler(const Eigen::Matrix3d& mtx);

/**
 * Converts a vector of Euler angles to a direction vector.
 *
 * @param euler A vector containing Euler angles (z, y, z).
 * @return A vector containing a normalized direction vector.
 */
Eigen::Vector3d eulerToVector(const Eigen::Vector3d& euler);

/**
 * Calculate the angle of the vector given by x and y, in radians.
 *
 * @param x The x-component of the vector.
 * @param y The y-component of the vector.
 * @return The angle of the vector given by x and y, in radians.
 */
double angle(double x, double y);
/**
 * Prints a matrix to stdout.
 */
void printMatrix(const std::string& name, const Eigen::MatrixXd& mtx);

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
