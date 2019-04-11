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
#include <unordered_set>
#include <map>

#include "math/Smoother.hpp"
#include "math/SplineSmoother.hpp"
#include "math/IDWSmoother.hpp"
#include "geog599/PointSorter.hpp"
#include "geog599/PointSource.hpp"
#include "geog599/PointFilter.hpp"
#include "geog599/ConcaveHull.hpp"
#include "ds/Octree.hpp"

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
	bool operator<(const Pt& p) const {
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
	bool operator>(const Pt& p) const {
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
	bool operator<=(const Pt& p) const {
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
	bool operator>=(const Pt& p) const {
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

using namespace uav::math;
using namespace uav::geog599;

/**
 * The trajectory planner reads a stream of Cartesian points from a real or simulated
 * LiDAR device and develops a surface-following trajectory using cubic splines.
 */
template <class P>
class TrajectoryPlanner {
private:
	PointSource<P>* m_ptSource;			///!< A source for 3D points, either fake or real.
	PointSorter<P> m_ptSorter;			///!< A 2D sorter for the point stream.

	std::list<P> m_all;					///!< The entire received pointset.
	std::list<P> m_points;				///!< The list of current filtered points, including new ones.
	std::list<P> m_surface;				///!< The list of surface points extracted from points list.
	std::vector<double> m_coeffs;		///!< The spline coefficients.
	std::vector<double> m_knots;		///!< The spline knots.
	std::vector<double> m_derivs;		///!< End derivatives.

	double m_weight;						///!< The weight param for smoothing.
	double m_smooth;						///!< The smooth param for smoothing.
	double m_alpha;

	/* TODO: For threading.
	bool m_running;							///!< True if the planner is currently running.
	bool m_procComplete;					///!< True when processing is complete.
	bool m_genComplete;						///!< True when generation is complete.
	*/

	P m_start;								///!< The start-point for the trajectory.
	double m_lastY;							///!< The y-coordinate from the most recent 2D point.

	double m_splineY;							///!< The current spline y boundary.
	int m_splineIdx;							///!< The current spline index.
	double m_blockSize;							///!< The length of trajectory sections that can be finalized. Used to calculate the spline index.
	int m_finalIndex;							///!< The last finalized index.
	bool m_blockWise;							///!< If true, a smoother is constructed for each block.
	std::map<int, Smoother<P>*> m_smoothers;	///!< Computes and stores the spline coefficients.


public:

	/**
	 * Default constructor.
	 *
	 * @param blockSize This is the horizontal length of a block of points that will be
	 * 					"finalized" and have a static trajectory computed on it. Subsequent
	 * 					trajectories will be constrained using the end state of the previous.
	 */
	TrajectoryPlanner(double blockSize = 2) :
		m_ptSource(nullptr),
		m_weight(1), m_smooth(0.5), m_alpha(10),
		//m_running(false),
		//m_procComplete(false), m_genComplete(false),
		m_lastY(0),
		m_splineY(0), m_splineIdx(0),
		m_blockSize(blockSize), m_finalIndex(-1), m_blockWise(false) {
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

	void setAlpha(double a) {
		m_alpha = a;
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
	 * Return a copy of the vector containing the points which constitute the hull-filtered surface.
	 *
	 * @return A copy of the vector containing the points which constitute the hull-filtered surface.
	 */
	const std::list<P>& surface() const {
		return m_surface;
	}

	/**
	 * Return a copy of the current point-set.
	 *
	 * @return A copy of the current point-set.
	 */
	const std::list<P>& points() const {
		return m_points;
	}

	/**
	 * Return a copy of the entire point-set.
	 *
	 * @return A copy of the entire point-set.
	 */
	const std::list<P>& allPoints() const {
		return m_all;
	}

	/**
	 * Return a list of the knots found by the spline algorithm, or an empty list
	 * if they are not available.
	 *
	 * @return A list of the knots found by the spline algorithm.
	 */
	const std::vector<P>& knots() const {
		return m_smoothers.at(0).pknots();
	}

	/**
	 * Return a reference to the SmoothSpline instance owned by this class.
	 *
	 * @return A reference to the SmoothSpline instance owned by this class.
	 */
	const std::map<int, Smoother<P>*>& splines() {
		return m_smoothers;
	}

	/**
	 * Compute and return the 1st derivative (velocity) of the current trajectory,
	 * and return it as a list of P, where y occurs every step from the start to end
	 * points and z is the velocity.
	 *
	 * @param count The number of equally spaced abscissae to compute the velocity on.
	 */
	bool splineVelocity(std::list<P>& velocity, int count) {
		/*
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
		*/
		return false;
	}

	/**
	 * Compute and return the 0th derivative (altitude) of the current trajectory,
	 * and return it as a list of P, where y occurs every step from the start to end
	 * points and z is the velocity.
	 *
	 * @param count The number of equally spaced abscissae to compute the altitude on.
	 */
	bool splineAltitude(std::list<P>& altitude, double spacing) {
		using namespace uav::math;
		if(m_smoothers.empty())
			return false;
		Smoother<P>* spline = m_smoothers.at(0);
		std::vector<double> y = Util::linspace(spline->min(), spline->max(), spacing);
		std::vector<double> z0(y.size());
		if(spline->evaluate(y, z0, 0)) {
			for(size_t j = 0; j < y.size(); ++j)
				altitude.emplace_back(0, y[j], z0[j], 0);
			return true;
		}
		return false;
	}

	bool splineAltitude(double x, double& y) {
		using namespace uav::math;
		if(m_smoothers.empty())
			return false;
		Smoother<P>* spline = m_smoothers.at(0);
		static std::vector<double> y0(1);
		static std::vector<double> z0(1);
		y0[0] = x;
		if(spline->evaluate(y0, z0, 0)) {
			y = z0[0];
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
		/*
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
		*/
		return false;
	}

	/**
	 * Set the source of points.
	 *
	 * @param psrc The PointSource.
	 */
	void setPointSource(PointSource<P>* psrc) {
		m_ptSource = psrc;
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

	std::unordered_set<size_t> m_yseen;	///<! To find and pertub duplicate y-values.

	/**
	 * Build and return a smoother according to the configuration.
	 */
	Smoother<P>* buildSmoother() {
		int smootherType = 0;
		m_blockWise = false;
		switch(smootherType) {
		case 0:
		{
			double radius = m_blockSize;
			double exponent = 1.5;
			double spacing = 2;
			IDWSmoother<P>* s1 = new IDWSmoother<P>(radius, exponent, spacing);
			return s1;
		}
		case 1:
		{
			SplineSmoother<P>* s2 = new SplineSmoother<P>(3, 1, 2);
			return s2;
		}
		case 2:
		{
			m_blockWise = true;
			SplineSmoother<P>* s3 = new SplineSmoother<P>(3, 1, 2);
			return s3;
		}
		default:
			throw std::runtime_error("No smoother defined.");
		}
	}

	/**
	 * Process points from the PointSource. Apply filters,
	 * collapse to 2D.
	 *
	 * @param start The start point of the flight.
	 * @param current The current point of the flight. TODO: Could be the finalization point.
	 */
	bool processPoints() {

		// Get all the available points.
		std::list<P> pts;
		if(m_ptSource->getPoints(pts)) {
			for(P& pt : pts) {
				if(pt.y() < m_splineY)
					continue;	// Skip points behind the finalization.
				pt.to2D(m_start);
				while(true) {
					size_t yy = (size_t) (pt.y() * 100000000);
					if (m_yseen.find(yy) == m_yseen.end()) {
						m_yseen.insert(yy);
						break;
					} else {
						pt.y(pt.y() - 1.0/10000000.0);
					}
				}
				m_lastY = pt.y();
				m_all.push_back(pt);	// TODO: Only useful for display.
				m_ptSorter.insert(pt, m_points);
			}
			return ConcaveHull<P>::buildHull(m_points, m_surface, m_alpha);
		}
		return false;
	}

	/**
	 * Generate the trajectory from the filtered point-set.
	 */
	bool generateTrajectory() {

		if(m_surface.empty())
			return false;

		// If a smoother isn't available, build a new one.
		if(m_smoothers.find(m_splineIdx) == m_smoothers.end())
			m_smoothers.emplace(std::make_pair(m_splineIdx, buildSmoother()));

		Smoother<P>* spline = m_smoothers[m_splineIdx];

		std::list<P> surface;
		double lastY;
		if(m_blockWise) {
			auto first = m_surface.end();
			do {
				--first;
			} while(m_splineY < first->y() && first != m_surface.begin());
			surface.assign(first, m_surface.end());
			lastY = m_surface.back().y();
		} else {
			surface.assign(m_surface.begin(), m_surface.end());
		}

		// Try to compute the spline. If it fails, don't update the index or boundary position.
		if(spline->fit(surface.begin(), surface.end())) {
			std::vector<double> kts = spline->knots();
			m_knots.insert(m_knots.end(), kts.begin(), kts.end());
			//m_coeffs.assign(m_spline.coefficients().begin(), m_spline.coefficients().end());
			//m_spline.derivatives(block.endPos, {0, 1});
			if(m_blockWise) {
				++m_splineIdx;
				m_splineY = lastY;
			}
			return true;
		}
		return false;
	}

	/**
	 * Compute the spline on the current filtered point-set.
	 *
	 * @param current The current position of the vehicle.
	 */
	bool compute(const P& current) {
		// Set the y coordinate behind which the point cloud is final; nothing
		// can be added and the trajectory can't be changed.
		int final = std::max(0, (int) (current.y() / m_blockSize));
		bool finalize = false;
		if((final - 1) > m_finalIndex) {
			m_finalIndex = final - 1;
			finalize = true;
		}
		processPoints();
		if(finalize && !generateTrajectory())
			return false;
		return true;
	}

	/**
	 * Get the altitude of the trajectory at the given y-coordinate.
	 *
	 * @return The altitude of the trajectory at the given y-coordinate.
	 */
	bool getTrajectoryAltitude(double y, double& z) {
		if(m_smoothers.find(m_splineIdx) != m_smoothers.end()) {
			Smoother<P>* spline = m_smoothers.at(m_splineIdx);
			return spline->evaluate(y, z, 0);
		}
		return false;
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
		std::vector<P> surface = surface();
		for(size_t i = 0; i < surface.size(); ++i) {
			const P& pt = surface[i];
			y.push_back(pt.y());
			z.push_back(pt.z());
		}
		for(auto& it : m_smoothers) {
			Smoother<P>* spline = it.second;
			if(!spline->evaluate(y, z0, 0))
				z0.resize(z.size());
			if(!spline->evaluate(y, z1, 1))
				z1.resize(z.size());
			if(!spline->evaluate(y, z2, 2))
				z2.resize(z.size());
		}
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
