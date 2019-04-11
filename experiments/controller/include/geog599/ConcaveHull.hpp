/*
 * CONCAVEHULL.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_CONCAVEHULL_HPP_
#define INCLUDE_CONCAVEHULL_HPP_

#include <cmath>
#include <vector>
#include <list>

#include "PointFilter.hpp"

namespace uav {
namespace geog599 {

namespace hullutils {

	/**
	 * Determine what side of the line joining p0 and p1, p is on.
	 */
	template <class P>
	double cross(const P& p1, const P& p2, const P& p3) {
		double cross = (p2.y() - p1.y()) * (p3.z() - p1.z()) - (p2.z() - p1.z()) * (p3.y() - p1.y());
		return cross;
	}

	/**
	 * Return the length of the segment joining p0 and p1.
	 */
	template <class P>
	double length(const P& p0, const P& p1) {
		double length = std::pow(p0.y() - p1.y(), 2.0) + std::pow(p0.z() - p1.z(), 2.0);
		return length;
	}

	template <class P>
	double lengthY(const P& p0, const P& p1) {
		double length = std::pow(p0.y() - p1.y(), 2.0);
		return length;
	}

} // hullutils


/**
 * Filters a point list by performing a concave hull operation on the top surface and
 * preserving only the vertices of the hull. If the alpha parameter is 0, a standard
 * Andrew's Monotone Scan algorithm is used, otherwise the segment length is limited,
 * producing a degenerate "concave" hull, wherein the segments can be shorter than alpha, but
 * not longer. Smaller alpha will have the effect of following the surface more closely,
 * but too small an alpha may break the algorithm, if the point cloud is clustered at
 * distances larger than alpha.
 *
 */
template <class P>
class ConcaveHull {
public:

	static bool buildHull(std::list<P>& pts, std::list<P>& hull, double alpha) {

		if(pts.size() < 3)
			return false;

		using namespace hullutils;

		alpha *= alpha;

		auto iter = pts.begin();
		std::vector<P> hull0;

		// Compute concave hull.
		do {
			while(hull0.size() >= 2
					&& cross(hull0[hull0.size() - 2], hull0[hull0.size() - 1], *iter) >= 0
					&& lengthY(hull0[hull0.size() - 2], *iter) <= alpha
					) {
				hull0.pop_back();
			}
			hull0.push_back(*iter);
		} while(++iter != pts.end());

		if(hull0.size() >= 2) {
			// Write the new items into the old list.
			hull.assign(hull0.begin(), hull0.end());
			return true;
		}
		return false;
	}

};

} // geog599
} // uav



#endif /* INCLUDE_CONCAVEHULL_HPP_ */
