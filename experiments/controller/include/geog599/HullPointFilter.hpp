/*
 * HullPointFilter.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_HULLPOINTFILTER_HPP_
#define INCLUDE_HULLPOINTFILTER_HPP_

#include <cmath>
#include <vector>
#include <list>

#include "PointFilter.hpp"

namespace uav {
namespace geog599 {
namespace filter {

namespace hullutils {

	/**
	 * Determine what side of the line joining p0 and p1, p is on.
	 */
	template <class P>
	int cross(const P& p0, const P& p1, const P& p) {
		int cross = (p0.y() - p.y()) * (p1.z() - p.z()) - (p0.z() - p.z()) * (p1.y() - p.y());
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
class HullPointFilter : public uav::geog599::filter::PointFilter<P> {
private:
	double m_alpha; ///!< The alpha value.

protected:

	void doFilter(std::list<P>& pts) {

		if(pts.size() < 3)
			return;

		using namespace hullutils;

		auto iter = pts.begin();
		std::vector<P> hull;
		hull.push_back(*iter);
		++iter;
		hull.push_back(*iter);
		++iter;

		// Compute concave hull.
		do {
			while(hull.size() >= 2
					&& cross(hull[hull.size() - 2], hull[hull.size() - 1], *iter) >= 0
					&& lengthY(hull[hull.size() - 2], *iter) <= (m_alpha * m_alpha)) {
				hull.pop_back();
			}
			hull.push_back(*iter);
		} while(++iter != pts.end());

		// Write the new items into the old list.
		pts.assign(hull.begin(), hull.end());
	}

public:
	/**
	 * Create a HullPointFilter with optional alpha value.
	 *
	 * @param alpha Controls the maximum segment length of the hull. Zero is no limit.
	 */
	HullPointFilter(double alpha = 0) :
		uav::geog599::filter::PointFilter<P>(),
		m_alpha(alpha) {}

	/**
	 * Destroy the filter.
	 */
	~HullPointFilter() {
	}

};

} // filter
} // geog599
} // uav



#endif /* INCLUDE_HULLPOINTFILTER_HPP_ */
