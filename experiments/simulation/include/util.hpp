/*
 * util.hpp
 *
 *  Created on: May 16, 2018
 *      Author: rob
 */

#ifndef INCLUDE_UTIL_HPP_
#define INCLUDE_UTIL_HPP_

#include <random>
#include <mutex>
#include <thread>
#include <iostream>
#include <unordered_map>

#include <Eigen/Core>

namespace uav {

namespace thread {

void setPriority(std::thread &th, int policy, int priority);
void setAffinity(std::thread &th, int core);

} // thread

namespace util {

/**
 * Classes that implement ClockObserver receive
 * ticks from a Clock instance which can be used as
 * a time-based driver. The time is the Clock's elapsed
 * time, not the wall time.
 */
class ClockObserver {
public:

	/**
	 * Called by a Clock at a specified interval. The time is the
	 * Clock's elapsed time, not wall time.
	 */
	virtual void tick(double time) = 0;

	virtual ~ClockObserver() {}
};

/**
 * Maintains a pointer to a ClockObserver and information
 * about its tick frequency.
 */
class ClockObserverItem {
public:
	double delay;			///<! The delay between one tick and the next.
	double lastTick;		///<! The time of the last tick.
	ClockObserver* item;	///<! A pointer to the ClockObserver. This instance is not the owner.

	ClockObserverItem();

	/**
	 * Build a ClockObserverItem using the given pointer and tick frequency (per second.)
	 * The tick frequency is converted to a time delay.
	 *
	 * @param item The ClockObserver item.
	 * @param delay The delay in seconds between ticks.
	 */
	ClockObserverItem(ClockObserver* item, double delay);
};

/**
 * The Clock provides a means of driving processes by "ticking"
 * at a configured frequency. This is not a real-time clock, with every
 * tick, the time advances according to the configured frequency, not
 * the actual wall time.
 */
class Clock {
private:
	bool m_running;
	double m_minStep;
	double m_currentTime;
	std::thread m_thread;
	std::unordered_map<ClockObserver*, ClockObserverItem> m_observers;
	std::mutex m_mtx;

	Clock();

public:

	static Clock& instance();

	/**
	 * Return the current clock time.
	 *
	 * @return The current clock time.
	 */
	static double currentTime();

	/**
	 * Add an observer to the clock. The caller maintains ownership
	 * of the pointer. If the observer exists, will overwrite the tick frequency.
	 *
	 * @param obs A pointer to a ClockObserver.
	 * @param delay The delay in seconds between ticks.
	 */
	static void addObserver(ClockObserver* obs, double delay);

	/**
	 * Remove an observer.
	 *
	 * @param obs A pointer to the ClockObserver.
	 */
	static void removeObserver(ClockObserver* obs);

	/**
	 * Start the clock.
	 */
	static void start();

	/**
	 * Stop the clock.
	 */
	static void stop();
};


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
