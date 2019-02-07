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
	unsigned long m_curIdx; ///!< The most recent insertion index. Subsequent insertion points are located starting here.

public:

	/**
	 * Insert the point into the list in such a way as to preserve the lexicographic sorting order.
	 * The y dimension is first, z second. The position of the insertion is preserved between calls.
	 * The point object is copied into the list.
	 *
	 * @param pt A point object. Must have a public y and z property.
	 * @param pts A list of points.
	 */
	void insert(const P& pt, std::list<P>& pts) {
		auto end = pts.end();
		auto begin = pts.begin();
		auto iter = begin + m_curIdx;
		double iy = (*iter).y, iz = (*iter).z;
		if(pt.y >= iy) {
			while(pt.y >= (iy = (*iter).y) && pt.z > (iz = (*iter).z) && iter != begin) {
				--iter;
				--m_curIdx;
			}
		} else {
			while(pt.y <= (iy = (*iter).y) && pt.z < (iz = (*iter).z) && iter != end) {
				++iter;
				++m_curIdx;
			}
		}
		pts.insert(iter, pt);
	}

};

} // uav



#endif /* INCLUDE_POINTSORTER_HPP_ */
