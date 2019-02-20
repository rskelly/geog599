/*
 * PlaneFilter.hpp
 *
 *  Created on: Feb 9, 2019
 *      Author: rob
 */

#ifndef INCLUDE_PLANEFILTER_HPP_
#define INCLUDE_PLANEFILTER_HPP_

#include <Eigen/Geometry>

#include "PointFilter.hpp"
#include "Octree.hpp"

namespace uav {

template <class P>
class PlaneFilter : public uav::PointFilter<P> {
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
		if(lst.size())
			std::cerr << "Plane: " << lst.size() << "\n";
		for(P& p : lst) {
			Eigen::Vector3d pv(p.x(), p.y(), p.z());
			if(m_line->distance(pv) <= m_planeWidth / 2)
				pts.push_back(p);
		}
	}

public:
	PlaneFilter(double planeWidth, double maxDist) :
		m_plane(nullptr),
		m_line(nullptr),
		m_tree(nullptr),
		m_planeWidth(planeWidth),
		m_maxDist(maxDist) {}

	void setPlane(const Eigen::Hyperplane<double, 3>* plane) {
		m_plane = plane;
	}

	void setLine(const Eigen::ParametrizedLine<double, 3>* line) {
		m_line = line;
	}

	void setOctree(const uav::ds::Octree<P>* tree) {
		m_tree = tree;
	}
};

} // uav

#endif /* INCLUDE_PLANEFILTER_HPP_ */
