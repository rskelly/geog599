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

namespace trajectoryutils {

	uint64_t microtime() {
		struct timeval time;
		gettimeofday(&time, NULL);
		return (time.tv_sec * 1000) + (time.tv_usec / 1000);
	}

	template <class P>
	void processPoints(uav::PointSource<P>* ptSource, uav::PointSorter<P>* ptSorter, uav::PointFilter<P>* ptFilter,
			std::list<P>* pts, const P* start, bool* running) {

		P pt;

		while(*running) {

			// Get the available points and sort them into the points list.
			size_t c = 0;
			while(ptSource->next(pt) && ++c < 100)
				ptSorter->insert(pt, *pts);

			// Filter the points.
			ptFilter->filter(*pts);
		}

	}

	template <class P>
	void generateTrajectory(std::list<P>* pts, bool* running) {

		while(*running) {

		}
	}

} // trajectoryutils


class Pt {
private:
	double _x;
	double _y;
	double _z;
	uint64_t _time;

public:

	Pt() : Pt(0, 0, 0) {}

	Pt(double x, double y, double z) : Pt(x, y, z, 0) {}

	Pt(double x, double y, double z, uint64_t time) :
		_x(x), _y(y), _z(z),
		_time(time > 0 ? time : uav::trajectoryutils::microtime()) {}

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

	void resetTime() {
		_time = uav::trajectoryutils::microtime();
	}

	/**
	 * Return point converted to a 2d representation as measured from the given origin.
	 * Time and z are copied over.
	 */
	Pt as2D(const Pt& start) const {
		double yx = std::sqrt(std::pow(start.x() - x(), 2.0) + std::pow(start.y() - y(), 2.0));
		return Pt(0, yx, z(), time());
	}

	void to2D(const Pt& start) {
		double yx = std::sqrt(std::pow(start.x() - x(), 2.0) + std::pow(start.y() - y(), 2.0));
		x(0);
		y(yx);
	}
};


template <class P>
class TrajectoryPlanner {
private:
	uav::PointSource<P>* m_ptSource;
	uav::PointFilter<P>* m_ptFilter;
	uav::PointSorter<P> m_ptSorter;
	std::list<P> m_points;

	bool m_running;
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

	/**
	 * Run the trajectory planner. It uses two threads. One for the point retrieval and
	 * filtering, one for planning the trajectory. Each time a trajectory is completed,
	 * the listener is notified.
	 */
	void start() {
		using namespace uav::trajectoryutils;
		if(!m_running) {
			m_running = true;

			double bounds[6] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
			m_ptSource->computeBounds(bounds);

			m_pthread = std::thread(processPoints, m_ptSource, &m_ptSorter, m_ptFilter, &m_points, &m_running);
			m_gthread = std::thread(generateTrajectory, &m_points, &m_running);
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
