/*
 * PlaneFilter.hpp
 *
 *  Created on: Feb 9, 2019
 *      Author: rob
 */

#ifndef INCLUDE_PLANEFILTER_HPP_
#define INCLUDE_PLANEFILTER_HPP_

#include <list>

#include <Eigen/Geometry>

#include "geog599/PointFilter.hpp"
#include "ds/Octree.hpp"

namespace uav {
namespace geog599 {
namespace filter {

template <class P>
class PlaneFilter : public uav::geog599::filter::PointFilter<P> {
private:
	const Eigen::Hyperplane<double, 3>* m_plane;
	const Eigen::ParametrizedLine<double, 3>* m_line;
	const uav::ds::Octree<P>* m_tree;
	double m_planeWidth;
	double m_maxDist;

protected:

	void doFilter(std::list<P>& pts) {
		std::list<P> lst;
		m_tree->planeSearch(*m_plane, m_maxDist, lst);
		for(P& p : lst) {
			Eigen::Vector3d pv(p.x(), p.y(), p.z());
			if(m_line->distance(pv) <= m_planeWidth / 2)
				pts.push_back(p);
		}
	}

public:

	/**
	 * Construct a Planefilter using the plane width and maximum point distance.
	 *
	 * @param planeWidth 	The width of the plane perpendicular to its axis.
	 * @param maxDist 		The maximum distance for a point from the plane for it to be
	 * 						captured by the search.
	 */
	PlaneFilter(double planeWidth, double maxDist) :
		uav::geog599::filter::PointFilter<P>(),
		m_plane(nullptr),
		m_line(nullptr),
		m_tree(nullptr),
		m_planeWidth(planeWidth),
		m_maxDist(maxDist) {}

	/**
	 * Set the plane. Items within maxDist of the plane will be captured.
	 *
	 * @param plane An Eigen::Hyperplane.
	 */
	void setPlane(const Eigen::Hyperplane<double, 3>* plane) {
		m_plane = plane;
	}

	/**
	 * Set the line that constitues the centreline of the plane. This provides a means
	 * of limiting the distance of queried points from the plane's axis.
	 *
	 * @param line An Eigen::ParametrizedLine.
	 */
	void setLine(const Eigen::ParametrizedLine<double, 3>* line) {
		m_line = line;
	}

	/**
	 * Set the Octree used to search for points.
	 *
	 * @param tree An Octree.
	 */
	void setOctree(const uav::ds::Octree<P>* tree) {
		m_tree = tree;
	}

	/**
	 * Destroy the filter.
	 */
	~PlaneFilter() {
	}
};

} // filter
} // geog599
} // uav

#endif /* INCLUDE_PLANEFILTER_HPP_ */
