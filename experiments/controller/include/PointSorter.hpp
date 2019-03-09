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

/**
 * Inserts a point into the appropriate location in the point list.
 */
template <class P>
class PointSorter {
private:
	size_t m_curIdx; ///!< The most recent insertion index. Subsequent insertion points are located starting here.

public:

	PointSorter() :
		m_curIdx(0) {}

	/**
	 * Insert the point into the list in such a way as to preserve the lexicographic sorting order.
	 * The y dimension is first, z second. The position of the insertion is preserved between calls.
	 * The point object is copied into the list.
	 *
	 * @param pt A point object. Must have a < operator on y and z.
	 * @param pts A list of points.
	 */
	void insert(const P& pt, std::list<P>& pts) {
		pts.push_back(pt);
		pts.sort();
	}

};

} // uav



#endif /* INCLUDE_POINTSORTER_HPP_ */
