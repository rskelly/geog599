/*
 * ScanPointFilter.hpp
 *
 *  Created on: Feb 8, 2019
 *      Author: rob
 */

#ifndef INCLUDE_SCANPOINTFILTER_HPP_
#define INCLUDE_SCANPOINTFILTER_HPP_

#include <list>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "PointFilter.hpp"

namespace uav {

template <class P>
class ScanPointFilter : public uav::PointFilter<P> {
private:
	double m_startX, m_startY;
	double m_endX, m_endY;
	double m_laserAngle, m_scanAngle;
	double m_altitude;

	double m_planeWidth;
	Eigen::Hyperplane m_plane;
	Eigen::ParametrizedLine m_line;

public:

	/**
	 * @param filename A LAS file.
	 * @param startx The starting x coordinate.
	 * @param starty The starting y coordinate.
	 * @param endx The ending x coordinate.
	 * @param endy The ending y coordinate.
	 * @param laserAngle The laser angle.
	 * @param scanAngle The scan angle.
	 * @param altitude Nominal platform altitude.
	 */
	ScanPointFilter(double startx, double starty, double endx, double endy,
			double laserAngle, double scanAngle, double altitude) :
			m_startX(startx), m_startY(starty),
			m_endX(endx), m_endY(endy),
			m_laserAngle(laserAngle), m_scanAngle(scanAngle),
			m_altitude(altitude) {

		using namespace Eigen;
		// Hypotenuse (angled range).
		double h = m_altitude / std::sin(m_laserAngle);
		Matrix3d rot = AngleAxisf(m_laserAngle, Vector3f::UnitX());
		Vector3d norm = Vector3d(0, 0, 1) * rot;
		Matrix3d dir = norm * AngleAxisf(90.0 * M_PI / 180.0, Vector3f::UnitX());
		Vector3d orig = Vector3d(m_startX, m_startY, 0);
		m_planeWidth = m_scanAngle * h;
		m_plane = Eigen::Hyperplane(norm, orig);
		m_line = Eigen::ParametrizedLine(orig, dir);
	}

	void doFilter(std::list<P>& pts) {
		using namespace Eigen;
		double minDist = 0.01;
		Pt start(startx, starty);
		std::list<P> out;
		for(P& pt : pts) {
			Vector3d pv(pt.x(), pt.y(), pt.z());
			// Check the distance from the plane, then check the that
			// the point is within 1/2 plane width of the centerline.
			if(m_plane.signedDistance(pv) < minDist
					&& m_line.distance(pv) < m_planeWidth / 2.0) {
				// Add the 2d version of the point to the filtered list.
				out.push_back(pt.to2d(start));
			}
		}
	}

};

} // uav



#endif /* INCLUDE_SCANPOINTFILTER_HPP_ */
