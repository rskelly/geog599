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

#include "PointFilter.hpp"

namespace uav {

namespace hullutils {

	/**
	 * Return a reference to the list item, by index.
	 */
	template <class P>
	const P& item(const std::list<P>& pts, size_t idx) {
		auto iter = pts.begin();
		std::advance(iter, idx);
		return *iter;
	}

	/**
	 * Determine what side of the line joining p0 and p1, p is on.
	 */
	template <class P>
	int cross(const P* p0, const P* p1, const P* p) {
	  return (p0->y() - p->y()) * (p1->z() - p->z()) - (p0->z() - p->z()) * (p1->y() - p->y());
	}

	/**
	 * Return the length of the segment joining p0 and p1.
	 */
	template <class P>
	double length(const P& p0, const P& p1) {
		return std::pow(p0->y() - p1->y(), 2.0) + std::pow(p0->z() - p1->z(), 2.0);
	}

}


/**
 * Filters a point list by performing a convex hull operation on the top surface and
 * preserving only the vertices of the hull. If the alpha parameter is 0, a standard
 * Andrew's algorithm is used, otherwise the segment length is limited, producing a
 * degenerate "concave" hull, whereing the segments can be shorter than alpha, but
 * not longer. Smaller alpha will have the effect of following the surface more closely,
 * but too small an alpha may break the algorithm, if the point cloud is clustered at
 * distances larger than alpha.
 *
 */
template <class P>
class HullPointFilter : public uav::PointFilter<P> {
private:
	double m_alpha; ///!< The alpha value.

protected:

	void doFilter(std::list<P*>& pts) {

		if(pts.size() < 3)
			return;

		using namespace hullutils;

		std::vector<size_t> hull;
		hull.push_back(0);
		hull.push_back(1);

		if(m_alpha <= 0) {
			// If there's no alpha, do a straight hull.
			for(size_t i = 2; i < pts.size(); ++i) {
				while(hull.size() >= 2
						&& cross(item(pts, hull[hull.size() - 2]), item(pts, hull[hull.size() - 1]), item(pts, i)))
					hull.pop_back();
			}
		} else {
			// If there's an alpha do a degenerate hull. It could have been one loop but we save a bit of
			// time and visual complexity this way.
			for(size_t i = 2; i < pts.size(); ++i) {
				while(hull.size() >= 2
						&& cross(item(pts, hull[hull.size() - 2]), item(pts, hull[hull.size() - 1]), item(pts, i))
						&& length(item(pts, hull[hull.size() - 2]), item(pts, i)) <= (m_alpha * m_alpha))
					hull.pop_back();
			}
		}

		// Move the kept vertices into a new list.
		std::list<P*> newHull;
		for(size_t i = 0; i < hull.size(); ++i)
			newHull.push_back(item(pts, hull[i]));

		// Write the new items into the old list.
		pts.swap(newHull);
	}

public:
	/**
	 * Create a HullPointFilter with optional alpha value.
	 *
	 * @param alpha Controls the maximum segment length of the hull. Zero is no limit.
	 */
	HullPointFilter(double alpha = 0) :
		m_alpha(alpha) {}

};

} // uav



#endif /* INCLUDE_HULLPOINTFILTER_HPP_ */
