/*
 * TrajectoryPlanner.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_TRAJECTORYPLANNER_HPP_
#define INCLUDE_TRAJECTORYPLANNER_HPP_

#include <sys/time.h>
#include <thread>
#include <cmath>

#include "PointSorter.hpp"
#include "PointSource.hpp"
#include "PointFilter.hpp"
#include "Octree.hpp"

namespace uav {

class Pt;

/**
 * Utilities for the TrajectoryPlanner to use.
 */
namespace trajectoryutils {

	/**
	 * Return the current time in microseconds.
	 */
	uint64_t microtime() {
		struct timeval time;
		gettimeofday(&time, NULL);
		return (time.tv_sec * 1000) + (time.tv_usec / 1000);
	}

} // trajectoryutils


/**
 * Represents a single Cartesian point in 3D space. Can be converted to a point
 * in 2D (y-z) space.
 */
class Pt {
private:
	double _x;
	double _y;
	double _z;
	uint64_t _time;

public:

	/**
	 * Create a default point with the current time.
	 */
	Pt() : Pt(0, 0, 0) {}

	/**
	 * Create a point at the given 3D coordinate and the current time.
	 */
	Pt(double x, double y, double z) : Pt(x, y, z, 0) {}

	/**
	 * Create a point at the given 3D coordinate and time.
	 */
	Pt(double x, double y, double z, uint64_t time) :
		_x(x), _y(y), _z(z),
		_time(time > 0 ? time : uav::trajectoryutils::microtime()) {}

	/**
	 * Copy constructor.
	 */
	Pt(const Pt& pt) :
		Pt(pt.x(), pt.y(), pt.z(), pt.time()) {}

	double x() const {
		return _x;
	}

	double y() const {
		return _y;
	}

	double z() const {
		return _z;
	}

	uint64_t time() const {
		return _time;
	}

	void x(double x) {
		_x = x;
	}

	void y(double y) {
		_y = y;
	}

	void z(double z) {
		_z = z;
	}

	void time(uint64_t time) {
		_time = time;
	}

	/**
	 * Set the time to the current time.
	 */
	void resetTime() {
		_time = uav::trajectoryutils::microtime();
	}

	/**
	 * Return a point converted to a 2d representation as measured from the given origin.
	 * Time and z are copied over.
	 */
	Pt as2D(const Pt& start) const {
		double yx = std::sqrt(std::pow(start.x() - x(), 2.0) + std::pow(start.y() - y(), 2.0));
		return Pt(0, yx, z(), time());
	}

	/**
	 * Update the current point as a 2D point from the given start coordinate.
	 */
	void to2D(const Pt& start) {
		double yx = std::sqrt(std::pow(start.x() - x(), 2.0) + std::pow(start.y() - y(), 2.0));
		x(0);
		y(yx);
	}
};


/**
 * The trajectory planner reads a stream of Cartesian points from a real or simlated
 * LiDAR device and develops a surface-following trajectory using cubic splines.
 */
template <class P>
class TrajectoryPlanner {
private:
	uav::PointSource<P>* m_ptSource;
	uav::PointFilter<P>* m_ptFilter;
	uav::PointSorter<P> m_ptSorter;
	std::list<P*> m_points;

	bool m_running;
	P m_start;
	std::thread m_pthread;
	std::thread m_gthread;

public:

	/**
	 * Set the source of points.
	 *
	 * @param psrc The PointSource.
	 */
	void setPointSource(uav::PointSource<P>* psrc) {
		m_ptSource = psrc;
	}

	void setPointFilter(uav::PointFilter<P>* pfilt) {
		m_ptFilter = pfilt;
	}

	void setStartPoint(P& pt) {
		m_start = pt;
	}

	/**
	 *
	 */
	void processPoints(const P& startPt) {

		P pt;

		while(m_running) {

			// Get the available points and sort them into the points list.
			if(m_ptSource->next(pt)) {
				pt.to2D(startPt);
				m_ptSorter.insert(new P(pt), m_points);
				// Filter the points.
				m_ptFilter->filter(m_points);
			}

			std::this_thread::sleep_for(std::chrono::duration<size_t, std::micro>(1000));
		}

	}

	void generateTrajectory(std::list<Pt>* pts, bool* running) {

		while(*running) {

		}
	}

	/**
	 * Run the trajectory planner. It uses two threads. One for the point retrieval and
	 * filtering, one for planning the trajectory. Each time a trajectory is completed,
	 * the listener is notified.
	 */
	void start() {
		using namespace uav::trajectoryutils;
		if(!m_running) {
			m_running = true;

			double bounds[6];
			m_ptSource->computeBounds(bounds);
			m_pthread = std::thread([this]{ this->processPoints(m_start); });
			//m_gthread = std::thread(generateTrajectory, &m_points, &m_running);
		}
	}

	void stop() {
		if(m_running) {
			m_running = false;
			if(m_pthread.joinable())
				m_pthread.join();
			if(m_gthread.joinable())
				m_gthread.join();
		}
	}

	~TrajectoryPlanner() {
		stop();
	}

};

} // uav



#endif /* INCLUDE_TRAJECTORYPLANNER_HPP_ */
