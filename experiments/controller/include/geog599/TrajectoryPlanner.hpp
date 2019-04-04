/*
 * TrajectoryPlanner.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_TRAJECTORYPLANNER_HPP_
#define INCLUDE_TRAJECTORYPLANNER_HPP_

#include <sys/time.h>
#include <cmath>
#include <list>
#include <iostream>
#include <fstream>

#include "geog599/PointSorter.hpp"
#include "geog599/PointSource.hpp"
#include "geog599/PointFilter.hpp"
#include "ds/Octree.hpp"
#include "math/SmoothSpline.hpp"

namespace uav {
namespace geog599 {

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

size_t __pt_time = 0;
/**
 * Represents a single Cartesian point in 3D space. Can be converted to a point
 * in 2D (y-z) space using an origin point.
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
	Pt(double x, double y, double z) : Pt(x, y, z, ++__pt_time) {}

	/**
	 * Create a point at the given 3D coordinate and time.
	 */
	Pt(double x, double y, double z, uint64_t time) :
		_x(x), _y(y), _z(z),
		_time(time > 0 ? time : trajectoryutils::microtime()) {}

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
	 * Returns the coordinate associated with the given index.
	 * idx % 3 == 0 --> x
	 * idx % 3 == 1 --> y
	 * idx % 3 == 2 --> z
	 *
	 * @param idx The index.
	 * @return The coordinate value.
	 */
	double operator[](int idx) const {
		switch(idx % 3) {
		case 0: return _x;
		case 1: return _y;
		default: return _z;
		}
	}

	/**
	 * Returns true if this point is less than the given point, according to y-z ordering.
	 * x is not considered.
	 *
	 * @param p A Pt.
	 * @return True if this point is less than the given point, according to y-z ordering.
	 */
	bool operator<(const Pt& p) {
		if(y() == p.y()) {
			return z() < p.z();
		} else {
			return y() < p.y();
		}
	}

	/**
	 * Returns true if this point is greater than the given point, according to y-z ordering.
	 * x is not considered.
	 *
	 * @param p A Pt.
	 * @return True if this point is greater than the given point, according to y-z ordering.
	 */
	bool operator>(const Pt& p) {
		if(y() == p.y()) {
			return z() > p.z();
		} else {
			return y() > p.y();
		}
	}

	/**
	 * Returns true if this point is less than or equal to the given point, according to y-z ordering.
	 * x is not considered.
	 *
	 * @param p A Pt.
	 * @return True if this point is less than or equal to the given point, according to y-z ordering.
	 */
	bool operator<=(const Pt& p) {
		if(y() == p.y()) {
			return z() <= p.z();
		} else {
			return y() <= p.y();
		}
	}

	/**
	 * Returns true if this point is greater than or equal to the given point, according to y-z ordering.
	 * x is not considered.
	 *
	 * @param p A Pt.
	 * @return True if this point is greater than or equal to the given point, according to y-z ordering.
	 */
	bool operator>=(const Pt& p) {
		if(y() == p.y()) {
			return z() >= p.z();
		} else {
			return y() >= p.y();
		}
	}

	/**
	 * Set the time to the current time.
	 */
	void resetTime() {
		_time = trajectoryutils::microtime();
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
 * The trajectory planner reads a stream of Cartesian points from a real or simulated
 * LiDAR device and develops a surface-following trajectory using cubic splines.
 */
template <class P>
class TrajectoryPlanner {
private:
	uav::geog599::filter::PointFilter<P>* m_ptFilter;	///!< A filter for the point stream.
	uav::geog599::PointSource<P>* m_ptSource;			///!< A source for 3D points, either fake or real.
	uav::geog599::PointSorter<P> m_ptSorter;			///!< A 2D sorter for the point stream.

	uav::math::SmoothSpline<P> m_spline;	///!< Computes and stores the spline coefficients.

	std::list<P> m_points;					///!< The list of all points.
	std::vector<P> m_surface;				///!< The list of surface points extracted from points list.
	double m_weight;						///!< The weight param for smoothing.
	double m_smooth;						///!< The smooth param for smoothing.

	/* TODO: For threading.
	bool m_running;							///!< True if the planner is currently running.
	bool m_procComplete;					///!< True when processing is complete.
	bool m_genComplete;						///!< True when generation is complete.
	*/

	P m_start;								///!< The start-point for the trajectory.
	double m_lastY;							///!< The y-coordinate from the most recent 2D point.

public:

	/**
	 * Default constructor.
	 */
	TrajectoryPlanner() :
		m_ptFilter(nullptr), m_ptSource(nullptr),
		m_weight(1), m_smooth(0.5),
		//m_running(false),
		//m_procComplete(false), m_genComplete(false),
		m_lastY(0) {
	}

	/**
	 * Set the weight to be used in the smoothing spline.
	 *
	 * @param w The weight.
	 */
	void setWeight(double w) {
		m_weight = w;
	}

	/**
	 * Set the smoothing factor to be used in the smoothing spline.
	 * If s==0, the spline is an interpolator.
	 *
	 * @param s The smoothing factor.
	 */
	void setSmooth(double s) {
		m_smooth = s;
	}

	/**
	 * Return the weight.
	 *
	 * @return The weight.
	 */
	double weight() const {
		return m_weight;
	}

	/**
	 * Return the smoothing factor.
	 *
	 * @return The smoothing factor.
	 */
	double smooth() const {
		return m_smooth;
	}

	/**
	 * Return a reference to the vector containing the points which constitute the hull-filtered surface.
	 *
	 * @return A reference to the vector containing the points which constitute the hull-filtered surface.
	 */
	const std::vector<P>& surface() const {
		return m_surface;
	}

	/**
	 * Return a reference to the current point-set.
	 *
	 * @return A reference to the current point-set.
	 */
	const std::list<P>& points() const {
		return m_points;
	}

	/**
	 * Return a list of the knots found by the spline algorithm, or an empty list
	 * if they are not available.
	 *
	 * @return A list of the knots found by the spline algorithm.
	 */
	std::list<P> knots() {
		std::list<P> lst;
		if(m_spline.valid()) {
			const std::vector<double>& t = m_spline.knots();
			std::vector<double> z(t.size());
			m_spline.evaluate(t, z, 0);
			for(size_t i = 0; i < t.size(); ++i)
				lst.emplace_back(0, t[i], z[i]);
		}
		return lst;
	}

	/**
	 * Return a reference to the SmoothSpline instance owned by this class.
	 *
	 * @return A reference to the SmoothSpline instance owned by this class.
	 */
	uav::math::SmoothSpline<P>& spline() {
		return m_spline;
	}

	/**
	 * Compute and return the 1st derivative (velocity) of the current trajectory,
	 * and return it as a list of P, where y occurs every step from the start to end
	 * points and z is the velocity.
	 *
	 * @param count The number of equally spaced abscissae to compute the velocity on.
	 */
	bool splineVelocity(std::list<P>& velocity, int count) {
		if(m_surface.empty())
			return false;
		std::vector<double> y(count);
		std::vector<double> z1(count);
		m_spline.linspace(m_surface[0].y(),m_surface[m_surface.size() - 1].y(), y, count);
		if(m_spline.evaluate(y, z1, 1)) {
			for(size_t i = 0; i < y.size(); ++i)
				velocity.emplace_back(0, y[i], z1[i], 0);
			return true;
		}
		return false;
	}

	/**
	 * Compute and return the 0th derivative (altitude) of the current trajectory,
	 * and return it as a list of P, where y occurs every step from the start to end
	 * points and z is the velocity.
	 *
	 * @param count The number of equally spaced abscissae to compute the altitude on.
	 */
	bool splineAltitude(std::list<P>& altitude, int count) {
		if(m_surface.empty())
			return false;
		std::vector<double> y(count);
		std::vector<double> z0(count);
		m_spline.linspace(m_surface[0].y(),m_surface[m_surface.size() - 1].y(), y, count);
		if(m_spline.evaluate(y, z0, 0)) {
			for(size_t i = 0; i < y.size(); ++i)
				altitude.emplace_back(0, y[i], z0[i], 0);
			return true;
		}
		return false;
	}

	/**
	 * Compute and return the 2nd derivative (acceleration) of the current trajectory,
	 * and return it as a list of P, where y occurs every step from the start to end
	 * points and z is the velocity.
	 *
	 * @param count The number of equally spaced abscissae to compute the acceleration on.
	 */
	bool splineAcceleration(std::list<P>& acceleration, int count) {
		if(m_surface.empty())
			return false;
		std::vector<double> y(count);
		std::vector<double> z2(count);
		m_spline.linspace(m_surface[0].y(),m_surface[m_surface.size() - 1].y(), y, count);
		if(m_spline.evaluate(y, z2, 2)) {
			for(size_t i = 0; i < y.size(); ++i)
				acceleration.emplace_back(0, y[i], z2[i], 0);
			return true;
		}
		return false;
	}

	/**
	 * Set the source of points.
	 *
	 * @param psrc The PointSource.
	 */
	void setPointSource(uav::geog599::PointSource<P>* psrc) {
		m_ptSource = psrc;
	}

	/**
	 * Set the point filter.
	 *
	 * @param pfilt The point filter.
	 */
	void setPointFilter(uav::geog599::filter::PointFilter<P>* pfilt) {
		m_ptFilter = pfilt;
	}

	/**
	 * Set the start point. This is used to collapse the incoming points into a 2D (y-z)
	 * representation by distance from this origin.
	 *
	 * @param pt The start point.
	 */
	void setStartPoint(P& pt) {
		m_start = pt;
	}

	/**
	 * Return the most recent y-coordinate from the received
	 * 2D points.
	 */
	double lastY() const {
		return m_lastY;
	}

	/**
	 * Process points from the PointSource. Apply filters,
	 * collapse to 2D.
	 */
	bool processPoints(const P& startPt) {
		// Get the available points and sort them into the points list.
		std::list<P> pts;
		if(m_ptSource->getPoints(pts)) {
			for(P& pt : pts) {
				pt.to2D(startPt);
				m_lastY = pt.y();
				m_ptSorter.insert(pt, m_points);
			}
			std::list<P> surf(m_points.begin(), m_points.end());
			m_ptFilter->filter(surf);
			m_surface.assign(surf.begin(), surf.end());
		}
		std::cout << m_points.size() << ", " << m_surface.size() << "\n";
		return m_surface.size() > 0;
	}

	/**
	 * Generate the trajectory from the filtered point-set.
	 */
	bool generateTrajectory() {
		if(!m_points.empty()){
			if(m_spline.fit(m_surface, m_weight, m_smooth))
				return true;
		}
		return false;
	}

	/**
	 * Compute the spline on the current filtered point-set.
	 */
	bool compute() {
		if(!processPoints(m_start))
			return false;
		if(!generateTrajectory())
			return false;
		return true;
	}

	/**
	 * Get the altitude of the trajectory at the given y-coordinate.
	 *
	 * @return The altitude of the trajectory at the given y-coordinate.
	 */
	bool getTrajectoryAltitude(double y, double& z) {
		return m_spline.evaluate(y, z, 0);
	}

	/**
	 * Write some internal state to the output stream.
	 *
	 * @param str The output stream.
	 */
	void write(std::ostream& str) {
		std::vector<double> y;
		std::vector<double> z;
		std::vector<double> z0;
		std::vector<double> z1;
		std::vector<double> z2;
		for(size_t i = 0; i < m_surface.size(); ++i) {
			const P& pt = m_surface[i];
			y.push_back(pt.y());
			z.push_back(pt.z());
		}
		if(!m_spline.evaluate(y, z0, 0))
			z0.resize(z.size());
		if(!m_spline.evaluate(y, z1, 1))
			z1.resize(z.size());
		if(!m_spline.evaluate(y, z2, 2))
			z2.resize(z.size());
		for(size_t i = 0; i < y.size(); ++i) {
			str << y[i] << "," << z[i] << ","<< z0[i] << "," << z1[i] << "," << z2[i] << "\n";
		}
	}

	/**
	 * Write the calculated values to a file in the given directory.
	 *
	 * @param A target directory. Must exist.
	 */
	void writeToFile(const std::string& dir) {
		std::ofstream ostr;
		{
			std::stringstream ss;
			ss << "traj_" << (int) (smooth() * 1000) << "_" << (int) (weight() * 1000) << ".csv";
			ostr.open(ss.str());
			write(ostr);
		}
	}

	/**
	 * Destroy the TrajectoryPlanner.
	 */
	~TrajectoryPlanner() {
	}

};

} // geog599
} // uav



#endif /* INCLUDE_TRAJECTORYPLANNER_HPP_ */
