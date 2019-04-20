/*
 * GeomPointFilter.hpp
 *
 *  Created on: Feb 7, 2019
 *      Author: rob
 */

#ifndef INCLUDE_GEOMPOINTFILTER_HPP_
#define INCLUDE_GEOMPOINTFILTER_HPP_

#include <list>

#include "geog599/PointFilter.hpp"

namespace uav {
namespace geog599 {
namespace filter {

/**
 * Filters a point cloud according to some geometric predicate.
 */
template <class P>
class GeomPointFilter : public uav::geog599::filter::PointFilter<P> {
private:
	double m_minY;	///<! The minimum y-coordinate passed.

protected:

	void doFilter(std::list<P>& pts) {
		std::list<P> lst;
		for(const P& pt : pts) {
			if(pt.y() >= m_minY)
				lst.push_back(pt);
		}
		pts.swap(lst);
	}

public:

	/**
	 * Default constructor.
	 */
	GeomPointFilter() :
		uav::geog599::filter::PointFilter<P>(),
		m_minY(0) {}

	/**
	 * Set the minimum y coordinate. All points with y lower than this are removed.
	 *
	 * @param miny The minimum y coordinate.
	 */
	void setMinY(double miny) {
		m_minY = miny;
	}

	/**
	 * Destroy the filter.
	 */
	~GeomPointFilter() {
	}

};


} // filter
} // geog599
} // uav



#endif /* INCLUDE_GEOMPOINTFILTER_HPP_ */
