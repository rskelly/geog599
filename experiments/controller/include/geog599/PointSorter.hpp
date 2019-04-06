/*
 * Sorter.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_POINTSORTER_HPP_
#define INCLUDE_POINTSORTER_HPP_

#include <list>

namespace uav {
namespace geog599 {

/**
 * Inserts a point into the appropriate location in the point list.
 */
template <class P>
class PointSorter {
public:

	/**
	 * Insert the point into the list in such a way as to preserve the lexicographic sorting order.
	 * The y dimension is first, z second. The position of the insertion is preserved between calls.
	 * The point object is copied into the list.
	 *
	 * TODO: Find a way to remember the position between calls?
	 *
	 * @param pt A point object. Must have a < operator on y and z.
	 * @param pts A list of points.
	 */
	void insert(const P& pt, std::list<P>& pts) {
		if(pts.empty() || pt >= pts.back()) {
			pts.push_back(pt);
		} else if(pt < pts.front()) {
			pts.push_front(pt);
		} else {
			auto it = pts.end();	// TODO: rbegin doesn't seem to work here.
			do {
				--it;
			} while(pt < *it && it != pts.begin());
			if(pt < *it && it == pts.begin()) {
				pts.push_front(pt);
			} else {
				++it;
				pts.insert(it, pt);
			}
		}
	}

};

} // geog599
} // uav



#endif /* INCLUDE_POINTSORTER_HPP_ */
